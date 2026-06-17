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
static int64_t total_cycles = 0;         // 总通讯周期数
static int64_t total_time_ns = 0;        // 总运行时间(ns)
static int64_t max_cycle_ns = 0;         // 最大周期(ns)
static int64_t min_cycle_ns = INT64_MAX; // 最小周期(ns)
static int64_t current_cycle_ns = 0;     // 当前周期(ns)
static int64_t current_jitter_ns = 0;    // 当前抖动(ns)
static int64_t total_jitter_ns = 0;      // 总抖动(ns)
static int64_t last_cycle_ts = 0;        // 上一周期时间戳(ns)
static int64_t max_dc_error_ns = 0;      // 历史最大DC误差(ns)
static int64_t total_dc_error_ns = 0;    // 总DC误差绝对值(ns)

// ==============================================
// PDO结构体定义（100%对齐DCDemo/驱动器默认顺序）
// ==============================================
// RxPDO（主站→从站，输出）13字节 / 104bit
// 顺序：目标位置(32bit) → 目标速度(32bit) → 控制字(16bit) → 目标力矩(16bit) → 操作模式(8bit)
typedef struct
{
   int32_t target_position; // 0x607A:00 目标位置 DINT
   int32_t target_velocity; // 0x60FF:00 目标速度 DINT
   uint16_t control_word;   // 0x6040:00 控制字 UINT
   int16_t target_torque;   // 0x6071:00 目标力矩 INT
   int8_t operation_mode;   // 0x6060:00 操作模式 SINT
} __attribute__((packed)) RxPDO_t;

// TxPDO（从站→主站，输入）15字节 / 120bit
// 顺序：实际位置(32bit) → 错误码(16bit) → 实际速度(32bit) → 状态字(16bit) → 实际力矩(16bit) → 操作模式显示(8bit)
typedef struct
{
   int32_t actual_position;       // 0x6064:00 实际位置 DINT
   uint16_t error_code;           // 0x603F:00 错误码 UINT
   int32_t actual_velocity;       // 0x606C:00 实际速度 DINT
   uint16_t status_word;          // 0x6041:00 状态字 UINT
   int16_t actual_torque;         // 0x6077:00 实际力矩 INT
   int8_t operation_mode_display; // 0x6061:00 模式显示 SINT
} __attribute__((packed)) TxPDO_t;

// 全局PDO指针
RxPDO_t *rx_pdo;
TxPDO_t *tx_pdo;

// ==============================================
// Motion Control 命令结构体
// ==============================================
typedef struct
{
   volatile int32_t target_position;
   volatile int32_t target_velocity;
   volatile uint16_t control_word;
   volatile int16_t target_torque;
   volatile int8_t operation_mode;
} MotionCmd_t;

static MotionCmd_t motion_cmd;

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
   if (total_cycles > warmup_cycles)
   {
      int64_t abs_dc_error = llabs(timeerror);

      if (abs_dc_error > max_dc_error_ns)
         max_dc_error_ns = abs_dc_error;

      total_dc_error_ns += abs_dc_error;
   }

   integral += timeerror;
   *offsettime = (int64)((timeerror * pgain) + (integral * igain));
}

// ==============================================
// SDO辅助函数
// ==============================================
int sdo_write(ecx_contextt *ctx, uint16_t slave, uint16_t index, uint8_t subindex, void *data, int size)
{
   int wkc = ecx_SDOwrite(ctx, slave, index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
   if (wkc <= 0)
   {
      printf("ERROR: SDO write failed! Index: 0x%04X:%02X, WKC: %d\n", index, subindex, wkc);
      return -1;
   }
   usleep(10000);
   return 0;
}

int sdo_read(ecx_contextt *ctx, uint16_t slave, uint16_t index, uint8_t subindex, void *data, int size)
{
   int size_read = size;

   int wkc = ecx_SDOread(ctx, slave, index, subindex, FALSE, &size_read, data, EC_TIMEOUTRXM);

   if (wkc <= 0)
   {
      printf("ERROR: SDO read failed! Index: 0x%04X:%02X, WKC: %d\n", index, subindex, wkc);
      return -1;
   }

   return size_read;
}

void debug_read_csp_related_params(uint16_t slave)
{
   printf("\n========== CSP Related SDO Debug ==========\n");

   uint16_t u16 = 0;
   uint32_t u32 = 0;
   int32_t  i32 = 0;
   int size = 0;

   // Pn000 控制模式，必须是 7：EtherCAT 运动控制模式
   size = sizeof(u16);
   if (ecx_SDOread(&ctx, slave, 0x2000, 0x01, FALSE, &size, &u16, EC_TIMEOUTRXM) > 0)
      printf("Pn000 / 2000:01 Control mode        = %u\n", u16);

   // 软件限位
   size = sizeof(i32);
   if (ecx_SDOread(&ctx, slave, 0x607D, 0x01, FALSE, &size, &i32, EC_TIMEOUTRXM) > 0)
      printf("607D:01 Min software limit          = %d\n", i32);

   size = sizeof(i32);
   if (ecx_SDOread(&ctx, slave, 0x607D, 0x02, FALSE, &size, &i32, EC_TIMEOUTRXM) > 0)
      printf("607D:02 Max software limit          = %d\n", i32);

   // 最大速度限制
   size = sizeof(u32);
   if (ecx_SDOread(&ctx, slave, 0x607F, 0x00, FALSE, &size, &u32, EC_TIMEOUTRXM) > 0)
      printf("607F:00 Max profile velocity        = %u\n", u32);

   size = sizeof(u32);
   if (ecx_SDOread(&ctx, slave, 0x6080, 0x00, FALSE, &size, &u32, EC_TIMEOUTRXM) > 0)
      printf("6080:00 Max motor speed             = %u\n", u32);

   // 电子齿轮
   size = sizeof(u32);
   if (ecx_SDOread(&ctx, slave, 0x6091, 0x01, FALSE, &size, &u32, EC_TIMEOUTRXM) > 0)
      printf("6091:01 Gear numerator              = %u\n", u32);

   size = sizeof(u32);
   if (ecx_SDOread(&ctx, slave, 0x6091, 0x02, FALSE, &size, &u32, EC_TIMEOUTRXM) > 0)
      printf("6091:02 Gear denominator            = %u\n", u32);

   // Following error 配置
   size = sizeof(u32);
   if (ecx_SDOread(&ctx, slave, 0x6065, 0x00, FALSE, &size, &u32, EC_TIMEOUTRXM) > 0)
      printf("6065:00 Following error window      = %u\n", u32);

   size = sizeof(u16);
   if (ecx_SDOread(&ctx, slave, 0x6066, 0x00, FALSE, &size, &u16, EC_TIMEOUTRXM) > 0)
      printf("6066:00 Following error timeout     = %u\n", u16);

   // 位置偏置
   size = sizeof(i32);
   if (ecx_SDOread(&ctx, slave, 0x60B0, 0x00, FALSE, &size, &i32, EC_TIMEOUTRXM) > 0)
      printf("60B0:00 Position offset             = %d\n", i32);

   // 转矩限制
   size = sizeof(u16);
   if (ecx_SDOread(&ctx, slave, 0x60E0, 0x00, FALSE, &size, &u16, EC_TIMEOUTRXM) > 0)
      printf("60E0:00 Positive torque limit       = %u\n", u16);

   size = sizeof(u16);
   if (ecx_SDOread(&ctx, slave, 0x60E1, 0x00, FALSE, &size, &u16, EC_TIMEOUTRXM) > 0)
      printf("60E1:00 Negative torque limit       = %u\n", u16);

   printf("===========================================\n\n");
}


bool configure_csp_params(uint16_t slave)
{
    printf("\nConfiguring CSP parameters...\n");

   ecx_statecheck(&ctx, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
   if (ctx.slavelist[slave].state != EC_STATE_PRE_OP)
   {
      printf("ERROR: Slave %d not in PRE-OP state, cannot configure PDO\n", slave);
      return false;
   }
   printf("Slave %d entered PRE-OP state\n", slave);

    int8_t mode_csp = 8;

   /*
    * 60C2:01 Interpolation time period value
    * 60C2:02 Interpolation time index
    *
    * cycle = 60C2:01 * 10^(60C2:02) s
    *
    * 1ms => 1, -3
    * 2ms => 2, -3
    *
    * 当前这台驱动 1C32/1C33 读回是 2000000ns，
    * 所以这里必须跟随 cycletime 写成 2ms，否则 SAFE_OP 会报 Invalid DC SYNC configuration。
    */
   uint8_t interp_period = (uint8_t)(cycletime / 1000000);
   int8_t  interp_index  = -3;

    /*
     * 先通过 SDO 写一次 6060 = 8。
     * 后面 PDO 里也会周期性写 6060 = 8。
     */
    if (sdo_write(&ctx, slave, 0x6060, 0x00, &mode_csp, sizeof(mode_csp)) != 0)
    {
        printf("WARNING: write 6060 CSP mode failed\n");
    }

    /*
     * 很多驱动 CSP/IP 都要求 60C2 周期匹配主站周期。
     */
    if (sdo_write(&ctx, slave, 0x60C2, 0x01, &interp_period, sizeof(interp_period)) != 0)
    {
        printf("WARNING: write 60C2:01 failed\n");
    }

    if (sdo_write(&ctx, slave, 0x60C2, 0x02, &interp_index, sizeof(interp_index)) != 0)
    {
        printf("WARNING: write 60C2:02 failed\n");
    }

    printf("CSP params set: 6060=8, 60C2:01=%u, 60C2:02=%d, cycle=%" PRId64 " ns",
          interp_period, interp_index, cycletime);
    return true;
}

bool configure_sm_dc_sync(uint16_t slave)
{
   printf("Reading SM DC sync objects 0x1C32/0x1C33...\n");

   /*
    * 注意：这台驱动的 1C32:02 / 1C33:02 是只读的。
    * 上一次强行写 1000000ns 后，读回仍是 2000000ns，
    * 然后我们又把 Sync0 配成 1000000ns，最终 SAFE_OP 报：
    * Invalid DC SYNC configuration。
    *
    * 所以这里不要强行写 1C32/1C33 的 cycle time。
    * 正确做法：读取驱动支持/当前配置的同步周期，再让主站 cycletime 和 Sync0 跟随它。
    */

   uint16_t sm2_sync_type = 0;
   uint32_t sm2_cycle_time = 0;
   uint32_t sm2_shift_time = 0;

   uint16_t sm3_sync_type = 0;
   uint32_t sm3_cycle_time = 0;
   uint32_t sm3_shift_time = 0;

   if (sdo_read(&ctx, slave, 0x1C32, 0x01,
                &sm2_sync_type, sizeof(sm2_sync_type)) > 0)
   {
      printf("Read 1C32:01 = %u", sm2_sync_type);
   }

   if (sdo_read(&ctx, slave, 0x1C32, 0x02,
                &sm2_cycle_time, sizeof(sm2_cycle_time)) > 0)
   {
      printf("Read 1C32:02 = %" PRIu32 " ns", sm2_cycle_time);
   }

   if (sdo_read(&ctx, slave, 0x1C32, 0x03,
                &sm2_shift_time, sizeof(sm2_shift_time)) > 0)
   {
      printf("Read 1C32:03 = %" PRIu32 " ns", sm2_shift_time);
   }

   if (sdo_read(&ctx, slave, 0x1C33, 0x01,
                &sm3_sync_type, sizeof(sm3_sync_type)) > 0)
   {
      printf("Read 1C33:01 = %u", sm3_sync_type);
   }

   if (sdo_read(&ctx, slave, 0x1C33, 0x02,
                &sm3_cycle_time, sizeof(sm3_cycle_time)) > 0)
   {
      printf("Read 1C33:02 = %" PRIu32 " ns", sm3_cycle_time);
   }

   if (sdo_read(&ctx, slave, 0x1C33, 0x03,
                &sm3_shift_time, sizeof(sm3_shift_time)) > 0)
   {
      printf("Read 1C33:03 = %" PRIu32 " ns", sm3_shift_time);
   }

   if (sm2_sync_type != 2 || sm3_sync_type != 2)
   {
      printf("WARNING: SM sync type is not DC SYNC0. SM2=%u SM3=%u",
             sm2_sync_type, sm3_sync_type);
   }

   if ((sm2_cycle_time > 0) && (sm3_cycle_time > 0) && (sm2_cycle_time != sm3_cycle_time))
   {
      printf("WARNING: SM2/SM3 cycle mismatch: SM2=%" PRIu32 " ns, SM3=%" PRIu32 " ns",
             sm2_cycle_time, sm3_cycle_time);
   }

   /*
    * 以 SM2 输出同步周期为准。
    * 你的日志中这里读回 2000000ns，所以主站必须切换到 2ms。
    */
   if (sm2_cycle_time > 0)
   {
      if ((uint32_t)cycletime != sm2_cycle_time)
      {
         printf("Adjust master cycletime: %" PRId64 " ns -> %" PRIu32 " ns according to 1C32:02",
                cycletime, sm2_cycle_time);

         cycletime = sm2_cycle_time;
         syncoffset = cycletime / 2;
      }
   }
   else
   {
      printf("WARNING: 1C32:02 is 0 or unreadable, keep cycletime=%" PRId64 " ns", cycletime);
   }

   printf("SM DC sync check done. Final master cycletime=%" PRId64 " ns, syncoffset=%" PRId64 " ns",
          cycletime, syncoffset);

   return true;
}

// 信号处理函数（优雅退出）
void signal_handler(int sig)
{
   run = 0;
   dorun = 0;
   inOP = 0;
}

// ==============================================
// 核心业务逻辑：状态机 + 老化运动（RT线程内调用，无阻塞）
// ==============================================
void runWork()
{
    static int step = 0;
    static bool step_printed[6] = {false};

    static int32_t base_pos = 0;
    static int32_t csp_target = 0;
    static int dir = 1;
    static int hold_cycles = 0;

    const int32_t CSP_STEP_PER_CYCLE = 20;      // 每 1ms 增加 20 count，先低速测试
    const int32_t CSP_MOVE_RANGE     = 20000;   // 正负 20000 count 往复
    const int     CSP_HOLD_CYCLES    = 500;   // 500 * 2ms = 1s

    switch (step)
    {
        case 0:
        {
            // 故障复位阶段也保持 mode=8，避免刚开始写 mode=0 后又切换
            motion_cmd.control_word = 0x0086;
            motion_cmd.operation_mode = 8;
            motion_cmd.target_position = tx_pdo->actual_position;
            motion_cmd.target_velocity = 0;
            motion_cmd.target_torque = 0;

            if (!step_printed[0]) {
                step_printed[0] = true;
                printf("[Step 0] Fault reset sent, CW:0x%04X, SW:0x%04X, pos:%d, mode:%d\n",
                       motion_cmd.control_word,
                       tx_pdo->status_word,
                       tx_pdo->actual_position,
                       tx_pdo->operation_mode_display);
            }
            step = 100;
        }
        break;

        case 100:
        {
            motion_cmd.control_word = 0x0006;   // Shutdown
            motion_cmd.operation_mode = 8;      // CSP
            motion_cmd.target_position = tx_pdo->actual_position;
            motion_cmd.target_velocity = 0;
            motion_cmd.target_torque = 0;

            if ((tx_pdo->status_word & 0x006F) == 0x0021) {
                if (!step_printed[1]) {
                    step_printed[1] = true;
                    printf("[Step 100] Ready to switch on, CW:0x%04X, SW:0x%04X, pos:%d, mode:%d\n",
                           motion_cmd.control_word,
                           tx_pdo->status_word,
                           tx_pdo->actual_position,
                           tx_pdo->operation_mode_display);
                }
                step = 200;
            }
        }
        break;

        case 200:
        {
            motion_cmd.control_word = 0x0007;   // Switch on
            motion_cmd.operation_mode = 8;
            motion_cmd.target_position = tx_pdo->actual_position;
            motion_cmd.target_velocity = 0;
            motion_cmd.target_torque = 0;

            if ((tx_pdo->status_word & 0x006F) == 0x0023) {
                if (!step_printed[2]) {
                    step_printed[2] = true;
                    printf("[Step 200] Switched on, CW:0x%04X, SW:0x%04X, pos:%d, mode:%d\n",
                           motion_cmd.control_word,
                           tx_pdo->status_word,
                           tx_pdo->actual_position,
                           tx_pdo->operation_mode_display);
                }
                step = 300;
            }
        }
        break;

        case 300:
        {
            motion_cmd.control_word = 0x000F;   // Enable operation
            motion_cmd.operation_mode = 8;
            motion_cmd.target_position = tx_pdo->actual_position;
            motion_cmd.target_velocity = 0;
            motion_cmd.target_torque = 0;

            if ((tx_pdo->status_word & 0x006F) == 0x0027) {
                if (!step_printed[3]) {
                    step_printed[3] = true;
                    printf("[Step 300] Operation enabled, CW:0x%04X, SW:0x%04X, pos:%d, mode:%d\n",
                           motion_cmd.control_word,
                           tx_pdo->status_word,
                           tx_pdo->actual_position,
                           tx_pdo->operation_mode_display);
                }
                step = 400;
            }
        }
        break;

        case 400:
        {
            base_pos = tx_pdo->actual_position;
            csp_target = base_pos;
            dir = 1;
            hold_cycles = CSP_HOLD_CYCLES;

            motion_cmd.control_word = 0x000F;
            motion_cmd.operation_mode = 8;
            motion_cmd.target_position = csp_target;
            motion_cmd.target_velocity = 0;
            motion_cmd.target_torque = 0;

            if (!step_printed[4]) {
                step_printed[4] = true;
                printf("[Step 400] CSP armed, CW:0x%04X, SW:0x%04X, base:%d, mode:%d\n",
                       motion_cmd.control_word,
                       tx_pdo->status_word,
                       base_pos,
                       tx_pdo->operation_mode_display);
            }
            step = 401;
        }
        break;

        case 401:
        {
            motion_cmd.operation_mode = 8;
            motion_cmd.target_velocity = 0;
            motion_cmd.target_torque = 0;

            if (hold_cycles > 0) {
                hold_cycles--;
                motion_cmd.target_position = csp_target;


                
                break;
            }
            
            motion_cmd.control_word = 0x000F;
            csp_target += dir * CSP_STEP_PER_CYCLE;

            if (csp_target >= base_pos + CSP_MOVE_RANGE) {
               csp_target = base_pos + CSP_MOVE_RANGE;
               dir = -1;
            } else if (csp_target <= base_pos - CSP_MOVE_RANGE) {
               csp_target = base_pos - CSP_MOVE_RANGE;
               dir = 1;
            }

            motion_cmd.target_position = csp_target;
        }
        break;

        default:
            step = 0;
            break;
    }
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

   // 初始化单调时钟时间戳
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
         // 高精度周期与抖动计算
         // ==============================================
         clock_gettime(CLOCK_MONOTONIC_RAW, &now);
         int64_t current_ts = now.tv_sec * NSEC_PER_SEC + now.tv_nsec;
         current_cycle_ns = current_ts - last_cycle_ts;
         last_cycle_ts = current_ts;

         if (total_cycles > 0)
         {
            total_time_ns += current_cycle_ns;

            if (current_cycle_ns > max_cycle_ns) max_cycle_ns = current_cycle_ns;
            if (current_cycle_ns < min_cycle_ns) min_cycle_ns = current_cycle_ns;

            int64_t avg_cycle_ns = total_time_ns / total_cycles;
            current_jitter_ns = llabs(current_cycle_ns - avg_cycle_ns);
            total_jitter_ns += current_jitter_ns;
         }

         total_cycles++;
         /* ================================
          * DC VALIDATION (IMPORTANT)
          * ================================ */
         if (total_cycles % 1000 == 0)
         {
            printf("DC TIME: %ld ns\n", ctx.DCtime);
         }
         // ==============================================
         // EtherCAT 通讯：收帧
         // ==============================================
         wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
         dowkccheck = (wkc == expectedWKC) ? 0 : dowkccheck + 1;

         if (ctx.slavelist[1].hasdc && (wkc > 0))
         {
            ec_sync(ctx.DCtime, cycletime, &toff);
         }

         // ==============================================
         // 业务逻辑 + PDO输出更新
         // ==============================================
         if (rx_pdo && inOP && tx_pdo)
         {
            runWork();

            // 完整写入所有PDO字段（顺序与结构体严格一致）
            rx_pdo->target_position = motion_cmd.target_position;
            rx_pdo->target_velocity = motion_cmd.target_velocity;
            rx_pdo->control_word = motion_cmd.control_word;
            rx_pdo->target_torque = motion_cmd.target_torque;
            rx_pdo->operation_mode = motion_cmd.operation_mode;
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
                  ecx_statecheck(&ctx, slaveix, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                  if (slave->state == EC_STATE_NONE)
                  {
                     slave->islost = TRUE;
                     slave->mbxhandlerstate = ECT_MBXH_LOST;
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
// 手动PDO配置（100%对齐DCDemo顺序）
// ==============================================
bool configure_pdo(uint16_t slave)
{
   printf("\nConfiguring manual PDO mapping (align to DCDemo)...\n");

   uint8_t zero = 0;
   uint8_t one = 1;
   uint32_t mapping;
   uint16_t pdo_index;
   uint8_t rx_entries = 5;
   uint8_t tx_entries = 6;

   ecx_statecheck(&ctx, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
   if (ctx.slavelist[slave].state != EC_STATE_PRE_OP)
   {
      printf("ERROR: Slave %d not in PRE-OP state, cannot configure PDO\n", slave);
      return false;
   }
   printf("Slave %d entered PRE-OP state\n", slave);

   // ========== RxPDO 0x1600（主站→从站） ==========
   printf("Configuring RxPDO 0x1600...\n");
   sdo_write(&ctx, slave, 0x1600, 0x00, &zero, sizeof(uint8_t));

   mapping = 0x607A0020; // 1. 目标位置 32bit
   sdo_write(&ctx, slave, 0x1600, 0x01, &mapping, sizeof(uint32_t));

   mapping = 0x60FF0020; // 2. 目标速度 32bit
   sdo_write(&ctx, slave, 0x1600, 0x02, &mapping, sizeof(uint32_t));

   mapping = 0x60400010; // 3. 控制字 16bit
   sdo_write(&ctx, slave, 0x1600, 0x03, &mapping, sizeof(uint32_t));

   mapping = 0x60710010; // 4. 目标力矩 16bit
   sdo_write(&ctx, slave, 0x1600, 0x04, &mapping, sizeof(uint32_t));

   mapping = 0x60600008; // 5. 操作模式 8bit
   sdo_write(&ctx, slave, 0x1600, 0x05, &mapping, sizeof(uint32_t));

   sdo_write(&ctx, slave, 0x1600, 0x00, &rx_entries, sizeof(uint8_t));

   // RxPDO分配表 0x1C12
   sdo_write(&ctx, slave, 0x1C12, 0x00, &zero, sizeof(uint8_t));
   pdo_index = 0x1600;
   sdo_write(&ctx, slave, 0x1C12, 0x01, &pdo_index, sizeof(uint16_t));
   sdo_write(&ctx, slave, 0x1C12, 0x00, &one, sizeof(uint8_t));

   // ========== TxPDO 0x1A00（从站→主站） ==========
   printf("Configuring TxPDO 0x1A00...\n");
   sdo_write(&ctx, slave, 0x1A00, 0x00, &zero, sizeof(uint8_t));

   mapping = 0x60640020; // 1. 实际位置 32bit
   sdo_write(&ctx, slave, 0x1A00, 0x01, &mapping, sizeof(uint32_t));

   mapping = 0x603F0010; // 2. 错误码 16bit
   sdo_write(&ctx, slave, 0x1A00, 0x02, &mapping, sizeof(uint32_t));

   mapping = 0x606C0020; // 3. 实际速度 32bit
   sdo_write(&ctx, slave, 0x1A00, 0x03, &mapping, sizeof(uint32_t));

   mapping = 0x60410010; // 4. 状态字 16bit
   sdo_write(&ctx, slave, 0x1A00, 0x04, &mapping, sizeof(uint32_t));

   mapping = 0x60770010; // 5. 实际力矩 16bit
   sdo_write(&ctx, slave, 0x1A00, 0x05, &mapping, sizeof(uint32_t));

   mapping = 0x60610008; // 6. 操作模式显示 8bit
   sdo_write(&ctx, slave, 0x1A00, 0x06, &mapping, sizeof(uint32_t));

   sdo_write(&ctx, slave, 0x1A00, 0x00, &tx_entries, sizeof(uint8_t));

   // TxPDO分配表 0x1C13
   sdo_write(&ctx, slave, 0x1C13, 0x00, &zero, sizeof(uint8_t));
   pdo_index = 0x1A00;
   sdo_write(&ctx, slave, 0x1C13, 0x01, &pdo_index, sizeof(uint16_t));
   sdo_write(&ctx, slave, 0x1C13, 0x00, &one, sizeof(uint8_t));

   printf("PDO mapping configuration completed\n");
   return true;
}

/* Transition network to operational state */
void ecatbringup(char *ifname)
{
   printf("EtherCAT Startup\n");
   int rv = ecx_init(&ctx, ifname);
   if (!rv)
   {
      printf("ERROR: ecx_init failed on %s\n", ifname);
      return;
   }

   ecx_config_init(&ctx);
   if (ctx.slavecount <= 0)
   {
      printf("ERROR: no EtherCAT slaves found\n");
      ecx_close(&ctx);
      return;
   }

   ec_groupt *group = &ctx.grouplist[0];
   uint16_t slave = 1;

   printf("Found %d EtherCAT slave(s)\n", ctx.slavecount);
   printf("Slave %d: %s\n", slave, ctx.slavelist[slave].name);
   printf("Vendor: 0x%08X, Product: 0x%08X, Revision: 0x%08X\n",
          ctx.slavelist[slave].eep_man,
          ctx.slavelist[slave].eep_id,
          ctx.slavelist[slave].eep_rev);

   /*
    * 1. PDO remap must be done in PRE_OP.
    * 注意：先不要写 60C2=1ms。
    * 这台驱动的 1C32/1C33 实际读回 2ms，必须先读取 SM DC 周期。
    */
   if (!configure_pdo(slave))
   {
      ecx_close(&ctx);
      return;
   }

   /*
    * 2. 读取 1C32/1C33。
    * 如果驱动读回 2000000ns，则自动把全局 cycletime 改成 2ms。
    */
   configure_sm_dc_sync(slave);

   /*
    * 3. 现在再配置 CSP 参数。
    * 60C2 会根据最终 cycletime 自动写：
    * 1ms => 60C2:01=1, 60C2:02=-3
    * 2ms => 60C2:01=2, 60C2:02=-3
    */
   configure_csp_params(slave);

   /* 2. Map PDO to IOmap */
   ecx_config_map_group(&ctx, IOmap, 0);
   expectedWKC = (group->outputsWKC * 2) + group->inputsWKC;

   printf("\nPDO Mapping Information:\n");
   printf("Slave %d outputs offset: %d bytes, outputs length: %d bytes\n",
          slave, ctx.slavelist[slave].Ooffset, ctx.slavelist[slave].Obytes);
   printf("Slave %d inputs offset: %d bytes, inputs length: %d bytes\n",
          slave, ctx.slavelist[slave].Ioffset, ctx.slavelist[slave].Ibytes);
   printf("Expected WKC: %d\n", expectedWKC);

   if (ctx.slavelist[slave].Obytes != sizeof(RxPDO_t))
   {
      printf("WARNING: RxPDO size mismatch! Expected %zu bytes, got %d bytes\n",
             sizeof(RxPDO_t), ctx.slavelist[slave].Obytes);
   }
   if (ctx.slavelist[slave].Ibytes != sizeof(TxPDO_t))
   {
      printf("WARNING: TxPDO size mismatch! Expected %zu bytes, got %d bytes\n",
             sizeof(TxPDO_t), ctx.slavelist[slave].Ibytes);
   }

   rx_pdo = (RxPDO_t *)ctx.slavelist[slave].outputs;
   tx_pdo = (TxPDO_t *)ctx.slavelist[slave].inputs;

   memset((void *)&motion_cmd, 0, sizeof(motion_cmd));

   /*
    * 先写一份安全PDO输出。
    * 注意：此时还没有进入 OP，inOP 也还是 0，所以 runWork 不会运行。
    */
   if (rx_pdo)
   {
      rx_pdo->target_position = 0;
      rx_pdo->target_velocity = 0;
      rx_pdo->control_word = 0x0000;
      rx_pdo->target_torque = 0;
      rx_pdo->operation_mode = 8; /* CSP */
   }

   /* 3. Configure Distributed Clocks */
   printf("\nConfiguring Distributed Clocks...\n");
   ecx_configdc(&ctx);

   /* Enable Sync0 for every DC capable slave */
   for (int i = 1; i <= ctx.slavecount; i++)
   {
      if (ctx.slavelist[i].hasdc)
      {
         printf("Enable DC Sync0 Slave %d, cycle = %" PRId64 " ns\n", i, cycletime);
         ecx_dcsync0(&ctx, i, TRUE, cycletime, 0);
      }
      else
      {
         printf("Slave %d has no DC support\n", i);
      }
   }

   /*
    * 4. Add CoE slaves to cyclic mailbox handler.
    * This is optional for pure PDO, but keep your original behavior.
    */
   for (int si = 1; si <= ctx.slavecount; si++)
   {
      ec_slavet *slv = &ctx.slavelist[si];

      printf("Slave %d hasdc: %d, DCactive: %d\n",
             si, slv->hasdc, slv->DCactive);

      if (slv->CoEdetails > 0)
      {
         ecx_slavembxcyclic(&ctx, si);
         printf("Slave %d added to cyclic mailbox handler\n", si);
      }
   }

   /*
    * 5. 明确请求 SAFE_OP。
    * 这是这次修复的关键：不要从 PRE_OP/配置阶段直接跳 OP。
    */
   printf("\nRequest SAFE_OP...\n");
   ctx.slavelist[0].state = EC_STATE_SAFE_OP;
   ecx_writestate(&ctx, 0);

   if (ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP)
   {
      printf("ERROR: Failed to enter SAFE_OP\n");
      ecx_readstate(&ctx);

      for (int si = 1; si <= ctx.slavecount; si++)
      {
         ec_slavet *slv = &ctx.slavelist[si];
         printf("Slave %d State=0x%02X StatusCode=0x%04X : %s\n",
                si,
                slv->state,
                slv->ALstatuscode,
                ec_ALstatuscode2string(slv->ALstatuscode));
      }

      ecx_close(&ctx);
      return;
   }

   printf("SAFE_OP OK\n");

   /*
    * 6. SAFE_OP 下先交换PDO，读到实际位置。
    * CSP启动前，TargetPosition 必须先等于 ActualPosition，防止一进OP就跳变。
    */
   printf("PDO warmup in SAFE_OP start...\n");

   for (int i = 0; i < 300; i++)
   {
      ecx_send_processdata(&ctx);
      wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
      osal_usleep(1000);
   }

   if (tx_pdo && rx_pdo)
   {
      motion_cmd.target_position = tx_pdo->actual_position;
      motion_cmd.target_velocity = 0;
      motion_cmd.control_word = 0x0000;
      motion_cmd.target_torque = 0;
      motion_cmd.operation_mode = 8;

      rx_pdo->target_position = motion_cmd.target_position;
      rx_pdo->target_velocity = motion_cmd.target_velocity;
      rx_pdo->control_word = motion_cmd.control_word;
      rx_pdo->target_torque = motion_cmd.target_torque;
      rx_pdo->operation_mode = motion_cmd.operation_mode;

      printf("Initial CSP target position = actual position = %d\n",
             tx_pdo->actual_position);
   }

   for (int i = 0; i < 100; i++)
   {
      ecx_send_processdata(&ctx);
      wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
      osal_usleep(1000);
   }

   printf("PDO warmup done, WKC=%d, expectedWKC=%d\n", wkc, expectedWKC);

   /*
    * 7. 现在才允许 RT 线程开始周期PDO。
    * 这里 inOP 仍然是 0，所以 RT线程只做 PDO send/receive，不会执行 runWork。
    */
   mappingdone = 1;
   dorun = 1;

   /* Let RT thread run several cycles in SAFE_OP before OP request */
   osal_usleep(300000);

   /*
    * 8. Request OP.
    * 注意：广播写状态时必须设置 ctx.slavelist[0].state。
    * 你之前只设置 slave[1].state，ecx_writestate(&ctx, 0) 不可靠。
    */
   printf("\nRequest OPERATIONAL...\n");
   ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
   ecx_writestate(&ctx, 0);

   if (ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
   {
      printf("ERROR: Failed to enter OPERATIONAL\n");

      dorun = 0;
      osal_usleep(100000);

      ecx_readstate(&ctx);
      for (int si = 1; si <= ctx.slavecount; si++)
      {
         ec_slavet *slv = &ctx.slavelist[si];

         if (slv->state != EC_STATE_OPERATIONAL)
         {
            printf("Slave %d State=0x%02X StatusCode=0x%04X : %s\n",
                   si,
                   slv->state,
                   slv->ALstatuscode,
                   ec_ALstatuscode2string(slv->ALstatuscode));
         }
      }

      ecx_close(&ctx);
      return;
   }

   /*
    * 9. OP confirmed. 从这里开始才允许 runWork 执行402/CSP控制。
    */
   inOP = TRUE;
   run = TRUE;

   printf("\nEtherCAT OP OK\n");
   printf("DC status after OP:\n");
   ecx_readstate(&ctx);
   for (int si = 1; si <= ctx.slavecount; si++)
   {
      printf("Slave %d State=0x%02X hasdc=%d DCactive=%d pdelay=%d ns\n",
             si,
             ctx.slavelist[si].state,
             ctx.slavelist[si].hasdc,
             ctx.slavelist[si].DCactive,
             ctx.slavelist[si].pdelay);
   }

   debug_read_csp_related_params(slave);

   while (run)
   {
      int64_t avg_cycle_us = 0;
      int64_t avg_jitter_us = 0;
      double dc_max_us = (double)max_dc_error_ns / US_PER_NSEC;
      double dc_avg_us = 0;

      if (total_cycles > 0)
      {
         avg_cycle_us = total_time_ns / total_cycles / US_PER_NSEC;
         dc_avg_us = (double)total_dc_error_ns / total_cycles / US_PER_NSEC;
      }

      if (total_cycles > 1)
      {
         avg_jitter_us = total_jitter_ns / (total_cycles - 1) / US_PER_NSEC;
      }

      printf("\n");
      printf("========================================\n");
      printf("Cycle Count      : %" PRId64 "\n", total_cycles);
      printf("WKC              : %d / expected %d\n", wkc, expectedWKC);
      printf("DC TIME          : %" PRId64 " ns\n", (int64_t)ctx.DCtime);

      if (tx_pdo)
      {
         uint16_t sw = tx_pdo->status_word;
         printf("Error Code       : 0x%04X\n", tx_pdo->error_code);
         printf("Status Word      : 0x%04X\n", sw);
         printf("Operation Mode   : %d\n", tx_pdo->operation_mode_display);
         printf("Actual Position  : %d\n", tx_pdo->actual_position);
         printf("Actual Velocity  : %d\n", tx_pdo->actual_velocity);
         printf("SW bit10 TargetReached : %d\n", (sw & 0x0400) ? 1 : 0);
         printf("SW bit11 InternalLimit : %d\n", (sw & 0x0800) ? 1 : 0);
         printf("SW bit12 DriveFollows  : %d\n", (sw & 0x1000) ? 1 : 0);
         printf("SW bit13 FollowingErr? : %d\n", (sw & 0x2000) ? 1 : 0);
      }

      if (rx_pdo)
      {
         printf("Control Word     : 0x%04X\n", rx_pdo->control_word);
         printf("Target Position  : %d\n", rx_pdo->target_position);
         printf("Target Velocity  : %d\n", rx_pdo->target_velocity);
         printf("Target Torque    : %d\n", rx_pdo->target_torque);
         printf("Target Mode      : %d\n", rx_pdo->operation_mode);
      }

      static int32_t sdo_target_position = 0;
      static int32_t sdo_actual_position = 0;

      if (tx_pdo && rx_pdo)
      {
         int size = sizeof(sdo_target_position);
         if (ecx_SDOread(&ctx, 1, 0x607A, 0x00, FALSE,
                        &size, &sdo_target_position, EC_TIMEOUTRXM) > 0)
         {
            printf("SDO 607A Target Position : %d\n", sdo_target_position);
         }

         size = sizeof(sdo_actual_position);
         if (ecx_SDOread(&ctx, 1, 0x6064, 0x00, FALSE,
                        &size, &sdo_actual_position, EC_TIMEOUTRXM) > 0)
         {
            printf("SDO 6064 Actual Position : %d\n", sdo_actual_position);
         }

         printf("PDO 607A Target Position : %d\n", rx_pdo->target_position);
         printf("PDO 6064 Actual Position : %d\n", tx_pdo->actual_position);
      }

      printf("\n");
      printf("Current Cycle    : %" PRId64 " us\n", current_cycle_ns / US_PER_NSEC);
      printf("Max Cycle        : %" PRId64 " us\n", max_cycle_ns / US_PER_NSEC);
      printf("Min Cycle        : %" PRId64 " us\n", min_cycle_ns / US_PER_NSEC);
      printf("Avg Cycle        : %" PRId64 " us\n", avg_cycle_us);
      printf("Avg Jitter       : %" PRId64 " us\n", avg_jitter_us);

      printf("\n");
      printf("Current DC Error : %.2f us\n", (double)timeerror / US_PER_NSEC);
      printf("Max DC Error     : %.2f us\n", dc_max_us);
      printf("Avg DC Error     : %.2f us\n", dc_avg_us);
      printf("========================================\n");

      osal_usleep(1000000 * 1);
   }

   printf("\nShutting down EtherCAT master...\n");

   dorun = 0;
   inOP = FALSE;
   osal_usleep(100000);

   int64_t avg_cycle_us = 0;
   int64_t avg_jitter_us = 0;
   double dc_avg_us = 0;
   double dc_cur_us = (double)llabs(timeerror) / US_PER_NSEC;
   double dc_max_us = (double)max_dc_error_ns / US_PER_NSEC;

   if (total_cycles > 0)
   {
      avg_cycle_us = total_time_ns / total_cycles / US_PER_NSEC;
      dc_avg_us = (double)total_dc_error_ns / total_cycles / US_PER_NSEC;
   }
   if (total_cycles > 1)
   {
      avg_jitter_us = total_jitter_ns / (total_cycles - 1) / US_PER_NSEC;
   }

   printf("\n========== Final Statistics ==========\n");
   printf("Total cycles    : %" PRId64 "\n", total_cycles);
   printf("Avg cycle       : %" PRId64 " us\n", avg_cycle_us);
   printf("Max cycle       : %" PRId64 " us\n", max_cycle_ns / US_PER_NSEC);
   printf("Min cycle       : %" PRId64 " us\n", min_cycle_ns / US_PER_NSEC);
   printf("Avg jitter      : %" PRId64 " us\n", avg_jitter_us);
   printf("DC Current Err  : %.2f us\n", dc_cur_us);
   printf("DC Max Err      : %.2f us\n", dc_max_us);
   printf("DC Avg Err      : %.2f us\n", dc_avg_us);
   printf("======================================\n");

   printf("EtherCAT to SAFE_OP\n");
   ctx.slavelist[0].state = EC_STATE_SAFE_OP;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

   printf("EtherCAT to INIT\n");
   ctx.slavelist[0].state = EC_STATE_INIT;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE);

   ecx_close(&ctx);
}

int main(int argc, char *argv[])
{
   printf("SOEM EtherCAT Master (Aging Mode, align DCDemo)\n");
   printf("Default cycle time: 1ms\n\n");

   // 全局内存锁定
   if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
   {
      printf("WARNING: Failed to lock memory (need root privileges). Performance may be degraded.\n");
   }
   else
   {
      printf("SUCCESS: All memory locked into RAM (no swap)\n");
   }

   // 初始化全局变量
   rx_pdo = NULL;
   tx_pdo = NULL;
   run = 1;
   mappingdone = 0;
   dorun = 0;
   inOP = 0;
   memset((void *)&motion_cmd, 0, sizeof(motion_cmd));

   // 注册信号处理函数
   signal(SIGINT, signal_handler);
   signal(SIGTERM, signal_handler);

   if (argc > 2)
   {
      cycletime = atoi(argv[2]) * 1000;
      printf("Using custom cycle time: %d us\n", atoi(argv[2]));
   }

   if (argc > 1)
   {
      /* create process data thread */
      osal_thread_create_rt(&threadrt, 128000, &ecatthread, NULL);
      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread1, 128000, &ecatcheck, NULL);

      // 设置RT线程优先级与名称
      struct sched_param param;
      param.sched_priority = 99;
      if (pthread_setschedparam((pthread_t)threadrt, SCHED_FIFO, &param) != 0)
      {
         printf("WARNING: Failed to set RT thread priority to 99\n");
      }
      else
      {
         printf("SUCCESS: RT thread priority set to 99 (SCHED_FIFO)\n");
      }

      if (pthread_setname_np((pthread_t)threadrt, "rt_eccomm") != 0)
      {
         printf("WARNING: Failed to set RT thread name\n");
      }
      else
      {
         printf("SUCCESS: RT thread renamed to 'rt_eccomm'\n");
      }

      if (pthread_setname_np((pthread_t)thread1, "ec_check") != 0)
      {
         printf("WARNING: Failed to set error handler thread name\n");
      }
      else
      {
         printf("SUCCESS: Error handler thread renamed to 'ec_check'\n");
      }

      /* bringup network */
      ecatbringup(argv[1]);
   }
   else
   {
      ec_adaptert *adapter = NULL;
      ec_adaptert *head = NULL;
      printf("Usage: %s ifname [cycletime]\n", argv[0]);
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