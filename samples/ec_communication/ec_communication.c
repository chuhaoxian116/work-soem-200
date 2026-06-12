#define _GNU_SOURCE 
/*
 * This software is dual-licensed under GPLv3 and a commercial
 * license. See the file LICENSE.md distributed with this software for
 * full license information.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>
#include <limits.h>
#include <pthread.h>
#include <sys/mman.h>

#include "soem/soem.h"

#define EC_TIMEOUTMON 500
#define NSEC_PER_SEC  1000000000
#define US_PER_NSEC   1000

static uint8 IOmap[4096];
static OSAL_THREAD_HANDLE threadrt;
static OSAL_THREAD_HANDLE thread1;
static int expectedWKC;
static int wkc;
static int mappingdone, dorun, inOP, run, dowkccheck;
static int currentgroup = 0;
static int warmup_cycles = 2000;
// 默认1ms周期（1000000ns），可通过命令行参数修改
static int64_t cycletime = 1000000;

static ecx_contextt ctx;

// ==============================================
// 通讯周期抖动统计变量（RT线程独占写入）
// ==============================================
static int64_t total_cycles = 0;          // 总通讯周期数
static int64_t total_time_ns = 0;         // 总运行时间(ns)
static int64_t max_cycle_ns = 0;          // 最大周期(ns)
static int64_t min_cycle_ns = INT64_MAX;  // 最小周期(ns)
static int64_t current_cycle_ns = 0;      // 当前周期(ns)
static int64_t current_jitter_ns = 0;     // 当前抖动(ns)
static int64_t total_jitter_ns = 0;       // 总抖动(ns)
static int64_t last_cycle_ts = 0;         // 上一周期时间戳(ns)
static int64_t max_dc_error_ns = 0;       // 历史最大DC误差(ns)
static int64_t total_dc_error_ns = 0;     // 总DC误差绝对值(ns)

// ==============================================
// PDO结构体定义（必须与映射顺序完全一致）
// ==============================================
typedef struct {
    uint16_t control_word;        // 0x6040:00h 控制字 UNSIGNED16
    int8_t operation_mode;        // 0x6060:00h 操作模式 INTEGER8
    int32_t target_position;      // 0x607A:00h 目标位置 INTEGER32
} __attribute__((packed)) RxPDO_t;

typedef struct {
    uint16_t error_code;          // 0x603F:00h 错误码 UNSIGNED16
    uint16_t status_word;         // 0x6041:00h 状态字 UNSIGNED16
    int8_t operation_mode_display;// 0x6061:00h 操作模式显示 INTEGER8
    int32_t actual_position;      // 0x6064:00h 位置反馈 INTEGER32
} __attribute__((packed)) TxPDO_t;

// 全局PDO指针（供所有线程访问）
RxPDO_t *rx_pdo;
TxPDO_t *tx_pdo;

/* add ns to ec_timet */
void add_time_ns(ec_timet *ts, int64 addtime)
{
   ec_timet addts;

   addts.tv_nsec = addtime % NSEC_PER_SEC;
   addts.tv_sec = (addtime - addts.tv_nsec) / NSEC_PER_SEC;
   osal_timespecadd(ts, &addts, ts);
}

static float pgain = 0.01f;
static float igain = 0.00002f;
/* set linux sync point 500us later than DC sync, just as example */
static int64 syncoffset = 500000;
static int64 timeerror;

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
   static int64 integral = 0;
   int64 delta;
   delta = (reftime - syncoffset) % cycletime;
   if (delta > (cycletime / 2))
   {
      delta = delta - cycletime;
   }
   timeerror = -delta;

   // 记录历史最大DC误差（取绝对值）跳过前面200周期
    if (total_cycles > warmup_cycles) {
        int64_t abs_dc_error = llabs(timeerror);

        if (abs_dc_error > max_dc_error_ns)
            max_dc_error_ns = abs_dc_error;

        total_dc_error_ns += abs_dc_error;
    }

   integral += timeerror;
   *offsettime = (int64)((timeerror * pgain) + (integral * igain));
}

// ==============================================
// SDO辅助函数（仅用于PDO配置）
// ==============================================
int sdo_write(ecx_contextt *ctx, uint16_t slave, uint16_t index, uint8_t subindex, void *data, int size) {
    int wkc = ecx_SDOwrite(ctx, slave, index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
    if (wkc <= 0) {
        printf("ERROR: SDO write failed! Index: 0x%04X:%02X, WKC: %d\n", index, subindex, wkc);
        return -1;
    }
    usleep(10000);
    return 0;
}

// 信号处理函数（优雅退出）
void signal_handler(int sig) {
    run = 0;
    dorun = 0;
    inOP = 0;
}

/* Cyclic RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void)
{
   ec_timet ts;
   int ht;
   static int64_t toff = 0;
   struct timespec now;

   dorun = 0;
   while (!mappingdone && run)
   {
      osal_usleep(100);
   }
   osal_get_monotonic_time(&ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   ecx_send_processdata(&ctx);

   // 初始化单调时钟时间戳（不受系统时间调整影响）
   clock_gettime(CLOCK_MONOTONIC_RAW, &now);
   last_cycle_ts = now.tv_sec * NSEC_PER_SEC + now.tv_nsec;

   while (run)
   {
      /* calculate next cycle start */
      add_time_ns(&ts, cycletime + toff);
      /* wait to cycle start */
      osal_monotonic_sleep(&ts);

      if (dorun > 0)
      {
         // ==============================================
         // 高精度周期与抖动计算（RT线程内执行，无锁）
         // ==============================================
         clock_gettime(CLOCK_MONOTONIC_RAW, &now);
         int64_t current_ts = now.tv_sec * NSEC_PER_SEC + now.tv_nsec;
         current_cycle_ns = current_ts - last_cycle_ts;
         last_cycle_ts = current_ts;

         // 跳过第一个不准确的周期
         if (total_cycles > 0) {
             total_time_ns += current_cycle_ns;

             // 更新最大/最小周期
             if (current_cycle_ns > max_cycle_ns) max_cycle_ns = current_cycle_ns;
             if (current_cycle_ns < min_cycle_ns) min_cycle_ns = current_cycle_ns;

             // 计算抖动：当前周期与平均周期的绝对偏差
             int64_t avg_cycle_ns = total_time_ns / total_cycles;
             current_jitter_ns = llabs(current_cycle_ns - avg_cycle_ns);
             total_jitter_ns += current_jitter_ns;
         }

         total_cycles++;

         // ==============================================
         // 原有EtherCAT通讯逻辑
         // ==============================================
         wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
         dowkccheck = (wkc == expectedWKC) ? 0 : dowkccheck + 1;

         if (ctx.slavelist[0].hasdc && (wkc > 0))
         {
            /* calculate toff to get linux time and DC synced */
            ec_sync(ctx.DCtime, cycletime, &toff);
         }
         ecx_mbxhandler(&ctx, 0, 4);
         ecx_send_processdata(&ctx);
      }
   }
}

/* Slave error handler */
OSAL_THREAD_FUNC ecatcheck(void)
{
   int slaveix;

   while (1)
   {
      if (inOP && ((dowkccheck > 2) || ctx.grouplist[currentgroup].docheckstate))
      {
         /* one or more slaves are not responding */
         ctx.grouplist[currentgroup].docheckstate = FALSE;
         ecx_readstate(&ctx);
         for (slaveix = 1; slaveix <= ctx.slavecount; slaveix++)
         {
            ec_slavet *slave = &ctx.slavelist[slaveix];

            if ((slave->group == currentgroup) && (slave->state != EC_STATE_OPERATIONAL))
            {
               ctx.grouplist[currentgroup].docheckstate = TRUE;
               if (slave->state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
               {
                  printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slaveix);
                  slave->state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                  ecx_writestate(&ctx, slaveix);
               }
               else if (slave->state == EC_STATE_SAFE_OP)
               {
                  printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slaveix);
                  slave->state = EC_STATE_OPERATIONAL;
                  if (slave->mbxhandlerstate == ECT_MBXH_LOST) slave->mbxhandlerstate = ECT_MBXH_CYCLIC;
                  ecx_writestate(&ctx, slaveix);
               }
               else if (slave->state > EC_STATE_NONE)
               {
                  if (ecx_reconfig_slave(&ctx, slaveix, EC_TIMEOUTMON) >= EC_STATE_PRE_OP)
                  {
                     slave->islost = FALSE;
                     printf("MESSAGE : slave %d reconfigured\n", slaveix);
                  }
               }
               else if (!slave->islost)
               {
                  /* re-check state */
                  ecx_statecheck(&ctx, slaveix, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                  if (slave->state == EC_STATE_NONE)
                  {
                     slave->islost = TRUE;
                     slave->mbxhandlerstate = ECT_MBXH_LOST;
                     /* zero input data for this slave */
                     if (slave->Ibytes)
                     {
                        memset(slave->inputs, 0x00, slave->Ibytes);
                     }
                     printf("ERROR : slave %d lost\n", slaveix);
                  }
               }
            }
            if (slave->islost)
            {
               if (slave->state <= EC_STATE_INIT)
               {
                  if (ecx_recover_slave(&ctx, slaveix, EC_TIMEOUTMON))
                  {
                     slave->islost = FALSE;
                     printf("MESSAGE : slave %d recovered\n", slaveix);
                  }
               }
               else
               {
                  slave->islost = FALSE;
                  printf("MESSAGE : slave %d found\n", slaveix);
               }
            }
         }
         if (!ctx.grouplist[currentgroup].docheckstate)
            printf("OK : all slaves resumed OPERATIONAL.\n");
         dowkccheck = 0;
      }
      osal_usleep(10000);
   }
}

// ==============================================
// 手动PDO配置函数（完全来自你的ec_pdoconfig.c）
// ==============================================
bool configure_pdo(uint16_t slave) {
    printf("\nConfiguring manual PDO mapping...\n");
    
    uint8_t zero = 0;
    uint8_t one = 1;
    uint32_t mapping;
    uint16_t pdo_index;
    uint8_t rx_entries = 3;
    uint8_t tx_entries = 4;

    // 确保从站处于PRE-OP状态才能配置PDO
    ecx_statecheck(&ctx, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    if (ctx.slavelist[slave].state != EC_STATE_PRE_OP) {
        printf("ERROR: Slave %d not in PRE-OP state, cannot configure PDO\n", slave);
        return false;
    }
    printf("Slave %d entered PRE-OP state\n", slave);

    // 配置RxPDO 0x1600
    printf("Configuring RxPDO 0x1600...\n");
    sdo_write(&ctx, slave, 0x1600, 0x00, &zero, sizeof(uint8_t));
    
    mapping = 0x60400010; // 控制字
    sdo_write(&ctx, slave, 0x1600, 0x01, &mapping, sizeof(uint32_t));
    
    mapping = 0x60600008; // 操作模式
    sdo_write(&ctx, slave, 0x1600, 0x02, &mapping, sizeof(uint32_t));
    
    mapping = 0x607A0020; // 目标位置
    sdo_write(&ctx, slave, 0x1600, 0x03, &mapping, sizeof(uint32_t));
    
    sdo_write(&ctx, slave, 0x1600, 0x00, &rx_entries, sizeof(uint8_t));
    
    // 配置RxPDO分配表 0x1C12
    sdo_write(&ctx, slave, 0x1C12, 0x00, &zero, sizeof(uint8_t));
    pdo_index = 0x1600;
    sdo_write(&ctx, slave, 0x1C12, 0x01, &pdo_index, sizeof(uint16_t));
    sdo_write(&ctx, slave, 0x1C12, 0x00, &one, sizeof(uint8_t));

    // 配置TxPDO 0x1A00
    printf("Configuring TxPDO 0x1A00...\n");
    sdo_write(&ctx, slave, 0x1A00, 0x00, &zero, sizeof(uint8_t));
    
    mapping = 0x603F0010; // 错误码
    sdo_write(&ctx, slave, 0x1A00, 0x01, &mapping, sizeof(uint32_t));
    
    mapping = 0x60410010; // 状态字
    sdo_write(&ctx, slave, 0x1A00, 0x02, &mapping, sizeof(uint32_t));
    
    mapping = 0x60610008; // 操作模式显示
    sdo_write(&ctx, slave, 0x1A00, 0x03, &mapping, sizeof(uint32_t));
    
    mapping = 0x60640020; // 实际位置
    sdo_write(&ctx, slave, 0x1A00, 0x04, &mapping, sizeof(uint32_t));
    
    sdo_write(&ctx, slave, 0x1A00, 0x00, &tx_entries, sizeof(uint8_t));
    
    // 配置TxPDO分配表 0x1C13
    sdo_write(&ctx, slave, 0x1C13, 0x00, &zero, sizeof(uint8_t));
    pdo_index = 0x1A00;
    sdo_write(&ctx, slave, 0x1C13, 0x01, &pdo_index, sizeof(uint16_t));
    sdo_write(&ctx, slave, 0x1C13, 0x00, &one, sizeof(uint8_t));

    printf("PDO mapping configuration completed successfully\n");
    return true;
}


/* Transition network to operational state */
void ecatbringup(char *ifname)
{
   printf("EtherCAT Startup\n");
   int rv = ecx_init(&ctx, ifname);
   if (rv)
   {
      ecx_config_init(&ctx);
      if (ctx.slavecount > 0)
      {
        ec_groupt *group = &ctx.grouplist[0];

        uint16_t slave = 1; // 目标从站地址

        printf("Found %d EtherCAT slave(s)\n", ctx.slavecount);
        printf("Slave %d: %s\n", slave, ctx.slavelist[slave].name);
        printf("Vendor: 0x%08X, Product: 0x%08X, Revision: 0x%08X\n",
            ctx.slavelist[slave].eep_man,
            ctx.slavelist[slave].eep_id,
            ctx.slavelist[slave].eep_rev);

        // ==============================================
        // 关键：在ecx_config_map_group之前执行手动PDO配置
        // ==============================================
        if (!configure_pdo(slave)) {
            ecx_close(&ctx);
            return;
        }
         // 映射PDO到IOmap
         ecx_config_map_group(&ctx, IOmap, 0);
         expectedWKC = (group->outputsWKC * 2) + group->inputsWKC;

         // 验证PDO大小
         printf("\nPDO Mapping Information:\n");
         printf("Slave %d outputs offset: %d bytes, outputs length: %d bytes\n",
                slave, ctx.slavelist[slave].Ooffset,
                ctx.slavelist[slave].Obytes);
         printf("Slave %d inputs offset: %d bytes, inputs length: %d bytes\n",
                slave, ctx.slavelist[slave].Ioffset,
                ctx.slavelist[slave].Ibytes);

         if (ctx.slavelist[slave].Obytes != sizeof(RxPDO_t)) {
             printf("WARNING: RxPDO size mismatch! Expected %zu bytes, got %d bytes\n",
                    sizeof(RxPDO_t), ctx.slavelist[slave].Obytes);
         }
         if (ctx.slavelist[slave].Ibytes != sizeof(TxPDO_t)) {
             printf("WARNING: TxPDO size mismatch! Expected %zu bytes, got %d bytes\n",
                    sizeof(TxPDO_t), ctx.slavelist[slave].Ibytes);
         }

         // 初始化全局PDO指针
         rx_pdo = (RxPDO_t*)ctx.slavelist[slave].outputs;
         tx_pdo = (TxPDO_t*)ctx.slavelist[slave].inputs;

         /* Configure distributed clocks */
         mappingdone = 1;
         ecx_configdc(&ctx);

         /* Add all CoE slaves to cyclic mailbox handler */
         int sdoslave = -1;
         for (int si = 1; si <= ctx.slavecount; si++)
         {
            ec_slavet *slave = &ctx.slavelist[si];
            if (slave->CoEdetails > 0)
            {
               ecx_slavembxcyclic(&ctx, si);
               sdoslave = si;
               printf(" Slave %d added to cyclic mailbox handler\n", si);
            }
         }

         /* Let network sync to clocks */
         dorun = 1;
         osal_usleep(1000000);

         /* Go to operational state */
         ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
         ecx_writestate(&ctx, 0);
         ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

         if (ctx.slavelist[0].state != EC_STATE_OPERATIONAL)
         {
            ecx_readstate(&ctx);
            for (int si = 1; si <= ctx.slavecount; si++)
            {
               ec_slavet *slave = &ctx.slavelist[si];
               if (slave->state != EC_STATE_OPERATIONAL)
               {
                  printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         si,
                         slave->state,
                         slave->ALstatuscode,
                         ec_ALstatuscode2string(slave->ALstatuscode));
               }
            }
         }
         else
         {
            int size;

            printf("EtherCAT OP\n");
            inOP = TRUE;
            run = TRUE;

            printf("\nRunning... (Ctrl+C exit)\n");
            printf("----------------------------------------------------------------------------------------------------------------------------------\n");
            printf("Cycle | WKC | Err | Stat | Op | Pos | DCcur(ns) | DCmax(us) | DCavg(us) | Cur(us) | Jit(us) | Max(us) | Min(us) | Avg(us) | AvgJit(us)\n");
            printf("----------------------------------------------------------------------------------------------------------------------------------\n");
            while (run)
            {
                int64_t avg_cycle_us  = 0;
                int64_t avg_jitter_us = 0;
                double  dc_max_us     = (double)max_dc_error_ns / US_PER_NSEC;  // 历史最大DC误差(us)
                double  dc_avg_us     = 0;                                      // 平均DC误差(us)

                if (total_cycles > 0)
                    avg_cycle_us = total_time_ns / total_cycles / US_PER_NSEC; // 平均通讯周期(us)
                if (total_cycles > 1)
                    avg_jitter_us = total_jitter_ns / (total_cycles - 1) / US_PER_NSEC; // 平均抖动(us)
                if (total_cycles > 0)
                    dc_avg_us = (double)total_dc_error_ns / total_cycles / US_PER_NSEC;
                /*
                * 实时行打印字段说明：
                * Cycle      : 总通讯周期计数
                * WKC        : Working Counter 工作计数器
                * Err        : 驱动器错误码
                * Stat       : 驱动器状态字
                * Op         : 当前运行模式
                * Pos        : 实际位置反馈
                * DCcur(ns)  : 当前DC同步误差(纳秒，带正负)
                * DCmax(us)  : 运行以来最大DC误差(微秒)
                * DCavg(us)  : 运行以来平均DC误差(微秒)
                * Cur(us)    : 当前单次通讯周期(微秒)
                * Jit(us)    : 当前周期抖动(微秒)
                * Max(us)    : 历史最大通讯周期(微秒)
                * Min(us)    : 历史最小通讯周期(微秒)
                * Avg(us)    : 平均通讯周期(微秒)
                * AvgJit(us) : 平均周期抖动(微秒)
                */
                printf("%5" PRId64 " | %3d | %04X | %04X | %2d | %8d | %10" PRId64 " | %8.2f | %8.2f | %7" PRId64 " | %7" PRId64 " | %7" PRId64 " | %7" PRId64 " | %7" PRId64 " | %7" PRId64 "\r",
                    total_cycles,
                    wkc,
                    tx_pdo ? tx_pdo->error_code : 0,
                    tx_pdo ? tx_pdo->status_word : 0,
                    tx_pdo ? tx_pdo->operation_mode_display : 0,
                    tx_pdo ? tx_pdo->actual_position : 0,
                    timeerror,
                    dc_max_us,
                    dc_avg_us,
                    current_cycle_ns / US_PER_NSEC,
                    current_jitter_ns / US_PER_NSEC,
                    max_cycle_ns / US_PER_NSEC,
                    min_cycle_ns / US_PER_NSEC,
                    avg_cycle_us,
                    avg_jitter_us);

                fflush(stdout);
               osal_usleep(200000); // 200ms刷新一次显示
            }
            printf("\n---------------------------------------------------------------------------------------------------------\n");
           
         }
         printf("\nShutting down EtherCAT master...\n");
         dorun = 0;
         inOP = FALSE;
         osal_usleep(100000);

         // 最终汇总
        int64_t avg_cycle_us = 0;
        int64_t avg_jitter_us = 0;
        double dc_avg_us = 0;
        double dc_cur_us = (double)llabs(timeerror) / US_PER_NSEC; // 当前DC误差转us
        double dc_max_us = (double)max_dc_error_ns / US_PER_NSEC;  // 最大DC误差转us

        if (total_cycles > 0)
        {
            avg_cycle_us = total_time_ns / total_cycles / US_PER_NSEC;
            dc_avg_us    = (double)total_dc_error_ns / total_cycles / US_PER_NSEC;
        }
        if (total_cycles > 1)
            avg_jitter_us = total_jitter_ns / (total_cycles - 1) / US_PER_NSEC;

        printf("\n========== Final Statistics ==========\n");
        printf("Total cycles    : %" PRId64 "\n", total_cycles);       // 总运行周期数
        printf("Avg cycle       : %" PRId64 " us\n", avg_cycle_us);    // 平均通讯周期
        printf("Max cycle       : %" PRId64 " us\n", max_cycle_ns / US_PER_NSEC); // 最大周期
        printf("Min cycle       : %" PRId64 " us\n", min_cycle_ns / US_PER_NSEC); // 最小周期
        printf("Avg jitter      : %" PRId64 " us\n", avg_jitter_us);   // 平均周期抖动
        printf("DC Current Err  : %.2f us\n", dc_cur_us);              // 当前DC同步误差
        printf("DC Max Err      : %.2f us\n", dc_max_us);               // 最大DC同步误差
        printf("DC Avg Err      : %.2f us\n", dc_avg_us);               // 平均DC同步误差
        printf("======================================\n");

         /* Go to SAFE_OP */
         printf("EtherCAT to SAFE_OP\n");
         ctx.slavelist[0].state = EC_STATE_SAFE_OP;
         ecx_writestate(&ctx, 0);
         ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

         /* Go to INIT state */
         printf("EtherCAT to INIT\n");
         ctx.slavelist[0].state = EC_STATE_INIT;
         ecx_writestate(&ctx, 0);
         ecx_statecheck(&ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE);
      }
   }
}

int main(int argc, char *argv[])
{
   printf("SOEM EtherCAT Master with Manual PDO Configuration\n");
   printf("Default cycle time: 1ms\n\n");

   // 全局内存锁定（必须在所有内存分配前执行）
   if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
       printf("WARNING: Failed to lock memory (need root privileges). Performance may be degraded.\n");
   } else {
       printf("SUCCESS: All memory locked into RAM (no swap)\n");
   }

   // 初始化全局变量
   rx_pdo = NULL;
   tx_pdo = NULL;
   run = 1;
   mappingdone = 0;
   dorun = 0;
   inOP = 0;

   // 注册信号处理函数
   signal(SIGINT, signal_handler);
   signal(SIGTERM, signal_handler);

   if (argc > 2) {
      cycletime = atoi(argv[2]) * 1000;
      printf("Using custom cycle time: %d us\n", atoi(argv[2]));
   }

   if (argc > 1)
   {
      /* create process data thread */
      osal_thread_create_rt(&threadrt, 128000, &ecatthread, NULL);
      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread1, 128000, &ecatcheck, NULL);

    // ✅ 正确：强制类型转换（把pthread_t*类型的变量转换成pthread_t）
    struct sched_param param;
    param.sched_priority = 99;
    if (pthread_setschedparam((pthread_t)threadrt, SCHED_FIFO, &param) != 0) {
        printf("WARNING: Failed to set RT thread priority to 99\n");
    } else {
        printf("SUCCESS: RT thread priority set to 99 (SCHED_FIFO)\n");
    }

    // ✅ 正确：强制类型转换
    if (pthread_setname_np((pthread_t)threadrt, "rt_eccomm") != 0) {
        printf("WARNING: Failed to set RT thread name\n");
    } else {
        printf("SUCCESS: RT thread renamed to 'rt_eccomm'\n");
    }

    // ✅ 正确：强制类型转换
    if (pthread_setname_np((pthread_t)thread1, "ec_check") != 0) {
        printf("WARNING: Failed to set error handler thread name\n");
    }

      /* bringup network */
      ecatbringup(argv[1]);
   }
   else
   {
      ec_adaptert *adapter = NULL;
      ec_adaptert *head = NULL;
      printf("Usage: ec_sample ifname1 [cycletime]\n");
      printf("ifname = eth0 for example\n");
      printf("cycletime in us\n");

      printf("\nAvailable adapters:\n");
      head = adapter = ec_find_adapters();
      while (adapter != NULL)
      {
         printf("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(head);
   }

   printf("End program\n");
   return (0);
}
