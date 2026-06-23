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


/*
 * Clean baseline notes
 * --------------------
 * This file keeps the current working SOEM application flow unchanged:
 *   PRE-OP PDO remap -> PDO map -> ESC DC config -> SAFE-OP warmup
 *   -> OP -> CiA402/CSP command sequence.
 *
 * The optional IGH reference-clock test keeps all changes in this demo and
 * sends one cyclic frame in the order observed in the IGH capture:
 *   FPWR(DC system time low 32 bits) + FRMW(DC time low 32 bits) + LRW(PDO).
 *
 * Cleanup policy:
 *   - Do not gate CSP motion on status-word bit12; match the working IgH demo.
 *   - Remove dead/unsafe helper code that is not called.
 *   - Keep one-shot startup diagnostics behind macros.
 *   - Disable OP-loop SDO debug by default to avoid mailbox noise during CSP.
 */
#define ENABLE_STARTUP_ESC_DEBUG      0
#define ENABLE_STARTUP_COE_DC_DIAG   1
#define ENABLE_OP_SDO_DEBUG          0
#define ENABLE_CYCLE_DC_TIME_PRINT   0
#define ENABLE_IGH_REFERENCE_SYNC    1
#define ENABLE_ENI_DC_PREINIT        0
#define ENABLE_SAFEOP_DC_PRECHECK     0

/*
 * DC verification policy
 * ----------------------
 * Allow enough clock-distribution cycles before OP. The elapsed verification
 * time is DC_VERIFY_STABLE_CYCLES multiplied by the selected master cycle.
 *
 * With ENABLE_IGH_REFERENCE_SYNC, the cyclic PDO frame excludes SOEM mailbox-status
 * bytes and sends LRW length = Obytes + Ibytes. For your mapping this should be
 * 13 + 15 = 28 bytes, matching the IGH capture.
 *
 * It does not change PDO mapping, CiA402/CSP control, or DC register setup.
 */
#define DC_VERIFY_LIMIT_NS           100000LL   /* 100 us */
#define DC_VERIFY_STABLE_CYCLES      200
#define DC_VERIFY_TIMEOUT_MS         12000     /* 12s */
#define DC_VERIFY_PRINT_EVERY        500
#define DC_VERIFY_REQUIRE_PASS       0          /* 1: stop before OP if verification fails */

/* Match the IGH startup sequence: converge 0x092C before enabling SYNC0. */
#define DC_START_DIFF_LIMIT_NS       10000LL
#define DC_START_TIMEOUT_MS          12000
#define DC_START_PRINT_EVERY         500

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
static int warmup_cycles = 20;
/* 对齐已验证可正常运动的 IGH myproject：默认 1 ms 周期。 */
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
/*
 * IGH 抓包中 application time 与 SYNC0 start time 同相。
 * ecx_dcsync0() 的 shift 也为 0，因此不再引入 2/3 周期相移。
 */
static int64 syncoffset = 0;
static int64 timeerror;

#if ENABLE_IGH_REFERENCE_SYNC
typedef struct
{
   int pending_idx;
   uint16_t fpwr_wkc_offset;
   uint16_t frmw_data_offset;
   uint16_t frmw_wkc_offset;
   uint16_t lrw_data_offset;
   uint16_t process_length;
   int64_t application_time_ns;
   int64_t sent_application_time_ns;
   bool application_time_valid;
} IghCycleState_t;

static IghCycleState_t igh_cycle = {
   .pending_idx = -1
};
#endif

static void update_dc_error(int64 reftime)
{
   int64 delta = (reftime - syncoffset) % cycletime;

   if (delta > (cycletime / 2))
   {
      delta -= cycletime;
   }
   else if (delta < -(cycletime / 2))
   {
      delta += cycletime;
   }

   timeerror = -delta;

   if (total_cycles > warmup_cycles)
   {
      int64_t abs_dc_error = llabs(timeerror);

      if (abs_dc_error > max_dc_error_ns)
         max_dc_error_ns = abs_dc_error;

      total_dc_error_ns += abs_dc_error;
   }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
   static int64 integral = 0;

   (void)cycletime;
   update_dc_error(reftime);

   integral += timeerror;
   *offsettime = (int64)((timeerror * pgain) + (integral * igain));
}

#if ENABLE_IGH_REFERENCE_SYNC
static int64_t igh_ec_time_to_ns(const ec_timet *ts)
{
   return ((int64_t)ts->tv_sec * NSEC_PER_SEC) + ts->tv_nsec;
}

static void igh_set_application_time_from_ts(const ec_timet *ts)
{
   /*
    * Match the IgH application pattern:
    *   ecrt_master_application_time(master, wakeup_time_ns)
    * IgH uses CLOCK_MONOTONIC based wake-up time, not EtherCAT epoch time.
    * The previous SOEM test generated an independent wall/EtherCAT-epoch based
    * counter, so the reference-clock FPWR was not phase-locked to the actual
    * cyclic wake-up instant.
    */
   igh_cycle.application_time_ns = igh_ec_time_to_ns(ts);
   igh_cycle.application_time_valid = true;
}

static void igh_init_application_time(void)
{
   ec_timet now;

   osal_get_monotonic_time(&now);
   igh_set_application_time_from_ts(&now);

   printf("IGH reference application time initialized from monotonic wake-up time: %" PRId64
          " ns, phase=%" PRId64 " ns\n",
          igh_cycle.application_time_ns,
          igh_cycle.application_time_ns % cycletime);
}

static int send_igh_processdata(void)
{
   ec_groupt *group = &ctx.grouplist[currentgroup];
   uint32_t app_time_le;
   uint32_t dc_time_le = 0;
   uint32_t log_addr;
   uint16_t w1;
   uint16_t w2;
   uint16_t process_length;
   uint16_t dc_slave;
   uint8_t idx;

   if (igh_cycle.pending_idx >= 0)
   {
      return 0;
   }

   if (group->blockLRW || !group->hasdc || !group->DCnext)
   {
      return 0;
   }

   /*
    * Keep the cyclic LRW payload equal to pure PDO data.
    * Your PDO mapping is:
    *   RxPDO = 13 bytes
    *   TxPDO = 15 bytes
    * Therefore the IGH-aligned LRW length must be 28 bytes.
    *
    * Do NOT use group->IOsegment[0] as LRW length here, because SOEM may
    * include one extra mailbox/status byte in the segment and produce 29 bytes.
    */
   process_length = (uint16_t)(group->Obytes + group->Ibytes);
   if (!process_length || !group->outputs ||
       !group->nsegments || process_length > group->IOsegment[0])
   {
      return 0;
   }

   if (!igh_cycle.application_time_valid)
   {
      igh_init_application_time();
   }

   if (group->mbxstatus && group->mbxstatuslength)
   {
      memset(group->mbxstatus, 0, group->mbxstatuslength);
   }

   igh_cycle.sent_application_time_ns = igh_cycle.application_time_ns;
   app_time_le = htoel((uint32_t)igh_cycle.sent_application_time_ns);
   igh_cycle.application_time_ns += cycletime;

   dc_slave = ctx.slavelist[group->DCnext].configadr;
   log_addr = group->logstartaddr;
   w1 = LO_WORD(log_addr);
   w2 = HI_WORD(log_addr);
   idx = ecx_getindex(&ctx.port);

   /*
    * IGH capture order:
    *   1) FPWR : write low 32-bit application time to ESC DC system time
    *   2) FRMW : read-modify-write low 32-bit ESC DC system time
    *   3) LRW  : process data, length = 13 + 15 = 28 bytes
    */
   ecx_setupdatagram(&ctx.port, &ctx.port.txbuf[idx],
                     EC_CMD_FPWR, idx, dc_slave, ECT_REG_DCSYSTIME,
                     sizeof(app_time_le), &app_time_le);
   igh_cycle.fpwr_wkc_offset = EC_HEADERSIZE + sizeof(app_time_le);

   igh_cycle.frmw_data_offset =
      ecx_adddatagram(&ctx.port, &ctx.port.txbuf[idx],
                      EC_CMD_FRMW, idx, TRUE,
                      dc_slave, ECT_REG_DCSYSTIME,
                      sizeof(dc_time_le), &dc_time_le);
   igh_cycle.frmw_wkc_offset =
      igh_cycle.frmw_data_offset + sizeof(dc_time_le);

   igh_cycle.lrw_data_offset =
      ecx_adddatagram(&ctx.port, &ctx.port.txbuf[idx],
                      EC_CMD_LRW, idx, FALSE,
                      w1, w2, process_length, group->outputs);
   igh_cycle.process_length = process_length;

   if (ecx_outframe_red(&ctx.port, idx) <= 0)
   {
      ecx_setbufstat(&ctx.port, idx, EC_BUF_EMPTY);
      return 0;
   }

   igh_cycle.pending_idx = idx;
   return 1;
}

static int receive_igh_processdata(int timeout)
{
   ec_groupt *group = &ctx.grouplist[currentgroup];
   uint8_t *rxbuf;
   uint16_t le_wkc;
   uint16_t fpwr_wkc;
   uint16_t frmw_wkc;
   uint16_t lrw_wkc;
   uint32_t le_dc_low;
   uint32_t dc_low;
   uint64_t anchor;
   uint64_t candidate;
   uint8_t idx;
   int frame_wkc;

   if (igh_cycle.pending_idx < 0)
   {
      return EC_NOFRAME;
   }

   idx = (uint8_t)igh_cycle.pending_idx;
   frame_wkc = ecx_waitinframe(&ctx.port, idx, timeout);
   if (frame_wkc <= EC_NOFRAME)
   {
      ecx_setbufstat(&ctx.port, idx, EC_BUF_EMPTY);
      igh_cycle.pending_idx = -1;
      return EC_NOFRAME;
   }

   rxbuf = ctx.port.rxbuf[idx];
   if (rxbuf[EC_CMDOFFSET] != EC_CMD_FPWR)
   {
      ecx_setbufstat(&ctx.port, idx, EC_BUF_EMPTY);
      igh_cycle.pending_idx = -1;
      return EC_NOFRAME;
   }

   memcpy(group->outputs,
          &rxbuf[igh_cycle.lrw_data_offset],
          igh_cycle.process_length);

   memcpy(&le_wkc,
          &rxbuf[igh_cycle.fpwr_wkc_offset],
          sizeof(le_wkc));
   fpwr_wkc = etohs(le_wkc);

   memcpy(&le_wkc,
          &rxbuf[igh_cycle.frmw_wkc_offset],
          sizeof(le_wkc));
   frmw_wkc = etohs(le_wkc);

   memcpy(&le_wkc,
          &rxbuf[igh_cycle.lrw_data_offset + igh_cycle.process_length],
          sizeof(le_wkc));
   lrw_wkc = etohs(le_wkc);

   memcpy(&le_dc_low,
          &rxbuf[igh_cycle.frmw_data_offset],
          sizeof(le_dc_low));
   dc_low = etohl(le_dc_low);

   anchor = (uint64_t)igh_cycle.sent_application_time_ns;
   candidate = (anchor & 0xffffffff00000000ULL) | dc_low;
   if ((candidate + 0x80000000ULL) < anchor)
      candidate += 0x100000000ULL;
   else if (candidate > (anchor + 0x80000000ULL))
      candidate -= 0x100000000ULL;
   ctx.DCtime = (int64_t)candidate;

   if (((fpwr_wkc != 1) || (frmw_wkc != 1)) &&
       ((total_cycles % 1000) == 0))
   {
      printf("IGH DC datagram WKC mismatch: FPWR=%u FRMW=%u\n",
             fpwr_wkc, frmw_wkc);
   }

   ecx_setbufstat(&ctx.port, idx, EC_BUF_EMPTY);
   igh_cycle.pending_idx = -1;
   return lrw_wkc;
}

static int send_cyclic_processdata(void)
{
   return send_igh_processdata();
}

static int receive_cyclic_processdata(int timeout)
{
   return receive_igh_processdata(timeout);
}
#else
static int send_cyclic_processdata(void)
{
   return ecx_send_processdata(&ctx);
}

static int receive_cyclic_processdata(int timeout)
{
   return ecx_receive_processdata(&ctx, timeout);
}
#endif

/*
 * IGH keeps reference-clock frames running while it polls ESC register 0x092C,
 * and only enables SYNC0 after the system-time difference has converged. The
 * previous SOEM flow enabled SYNC0 immediately after ecx_configdc(), so the
 * later SAFE_OP phase check happened too late to reproduce the IGH startup.
 */
static bool wait_dc_start_convergence(uint16_t slave)
{
   ec_timet wakeup_time;
   uint16_t configadr = ctx.slavelist[slave].configadr;
   int timeout_cycles = (int)((DC_START_TIMEOUT_MS * 1000000LL) / cycletime);
   int32_t diff_le = 0;
   int32_t diff_ns = 0;
   int valid_samples = 0;

   printf("\n========== DC Start Convergence ==========\n");
   printf("Difference limit : %" PRId64 " ns\n", (int64_t)DC_START_DIFF_LIMIT_NS);
   printf("Timeout          : %d ms (%d cycles)\n",
          DC_START_TIMEOUT_MS, timeout_cycles);

   osal_get_monotonic_time(&wakeup_time);

   for (int i = 0; i < timeout_cycles && run; i++)
   {
      int dc_wkc;
      int64_t abs_diff;

      add_time_ns(&wakeup_time, cycletime);
      osal_monotonic_sleep(&wakeup_time);

#if ENABLE_IGH_REFERENCE_SYNC
      igh_set_application_time_from_ts(&wakeup_time);
#endif
      if (send_cyclic_processdata() > 0)
      {
         (void)receive_cyclic_processdata(EC_TIMEOUTRET);
      }

      diff_le = 0;
      dc_wkc = ecx_FPRD(&ctx.port, configadr, ECT_REG_DCSYSDIFF,
                        sizeof(diff_le), &diff_le, EC_TIMEOUTRET);
      if (dc_wkc <= 0)
      {
         if ((i % DC_START_PRINT_EVERY) == 0)
         {
            printf("DC start convergence: 0x092C read failed, wkc=%d\n", dc_wkc);
         }
         continue;
      }

      diff_ns = (int32_t)etohl((uint32_t)diff_le);
      abs_diff = llabs((int64_t)diff_ns);
      valid_samples++;

      if ((i % DC_START_PRINT_EVERY) == 0)
      {
         printf("DC start convergence: cycle=%d diff=%d ns abs=%" PRId64 " ns\n",
                i, diff_ns, abs_diff);
      }

      if (abs_diff <= DC_START_DIFF_LIMIT_NS)
      {
         printf("DC start convergence: PASS after %d cycles, diff=%d ns\n",
                i + 1, diff_ns);
         printf("==========================================\n\n");
         return true;
      }
   }

   printf("DC start convergence: FAIL, valid_samples=%d last_diff=%d ns\n",
          valid_samples, diff_ns);
   printf("==========================================\n\n");
   return false;
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

static uint16_t le16(const uint8_t *p)
{
   return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t le32(const uint8_t *p)
{
   return ((uint32_t)p[0]) |
          ((uint32_t)p[1] << 8) |
          ((uint32_t)p[2] << 16) |
          ((uint32_t)p[3] << 24);
}

static uint64_t le64(const uint8_t *p)
{
   return ((uint64_t)le32(p)) | ((uint64_t)le32(p + 4) << 32);
}

static int esc_read_bytes(uint16_t slave, uint16_t ado, void *buf, uint16_t len)
{
   uint16_t adp = ctx.slavelist[slave].configadr;
   int ret = ecx_FPRD(&ctx.port, adp, ado, len, buf, EC_TIMEOUTRET);

   if (ret <= 0)
   {
      printf("ERROR: ESC FPRD failed, slave=%u, ADP=0x%04X, ADO=0x%04X, len=%u, WKC=%d\n",
             slave, adp, ado, len, ret);
   }

   return ret;
}

void debug_read_esc_dc_regs(uint16_t slave, const char *tag)
{
   uint8_t b2[2] = {0};
   uint8_t b4[4] = {0};
   uint8_t b8[8] = {0};

   printf("\n========== ESC DC Register Debug: %s ==========\n", tag);

   printf("Slave configadr = 0x%04X\n", ctx.slavelist[slave].configadr);

   if (esc_read_bytes(slave, 0x0980, b2, 2) > 0)
   {
      printf("0x0980 DC activation word      raw=%02X %02X, le16=0x%04X\n",
             b2[0], b2[1], le16(b2));
      printf("       0x0980 byte             = 0x%02X\n", b2[0]);
      printf("       0x0981 sync activation  = 0x%02X\n", b2[1]);
   }

   if (esc_read_bytes(slave, 0x0990, b8, 8) > 0)
   {
      printf("0x0990 SYNC0 start time        raw=%02X %02X %02X %02X %02X %02X %02X %02X, le64=%" PRIu64 "\n",
             b8[0], b8[1], b8[2], b8[3], b8[4], b8[5], b8[6], b8[7], le64(b8));
   }

   if (esc_read_bytes(slave, 0x09A0, b8, 8) > 0)
   {
      printf("0x09A0 SYNC0 cycle time        raw=%02X %02X %02X %02X %02X %02X %02X %02X, le64=%" PRIu64 "\n",
             b8[0], b8[1], b8[2], b8[3], b8[4], b8[5], b8[6], b8[7], le64(b8));
   }

   if (esc_read_bytes(slave, 0x09A8, b2, 2) > 0)
   {
      printf("0x09A8 DC latch config         raw=%02X %02X, le16=0x%04X\n",
             b2[0], b2[1], le16(b2));
   }

   if (esc_read_bytes(slave, 0x0910, b8, 8) > 0)
   {
      printf("0x0910 DC system time          le64=%" PRIu64 "\n", le64(b8));
   }

   if (esc_read_bytes(slave, 0x0900, b4, 4) > 0)
   {
      printf("0x0900 Receive time port 0/1   raw=%02X %02X %02X %02X, le32=%" PRIu32 "\n",
             b4[0], b4[1], b4[2], b4[3], le32(b4));
   }

   if (esc_read_bytes(slave, 0x092C, b4, 4) > 0)
   {
      printf("0x092C DC system time diff     raw=%02X %02X %02X %02X, le32=%" PRIu32 "\n",
             b4[0], b4[1], b4[2], b4[3], le32(b4));
   }

   printf("================================================\n\n");
}



/*
 * Execute a small subset of ENI DC/ESC initialization commands.
 *
 * Purpose:
 *   Test whether the drive depends on ENI-style DC reset/filter/latch
 *   initialization before ecx_configdc() / ecx_dcsync0().
 *
 * This function does NOT configure PDO.
 * This function does NOT enable Sync0.
 * This function does NOT replace ecx_configdc() / ecx_dcsync0().
 *
 * Call once during startup, before:
 *   ecx_configdc()
 *   ecx_dcsync0()
 *
 * Do NOT call from the real-time cycle.
 */
static void execute_eni_dc_preinit(uint16_t slave)
{
   int wkc;
   uint16_t configadr = ctx.slavelist[slave].configadr;

   printf("\n========== ENI DC/ESC PreInit Test ==========\n");
   printf("Slave %u configadr = 0x%04X\n", slave, configadr);

   /*
    * ENI Master InitCmd:
    *   clear dc system time
    *   Cmd = BWR, Ado = 2320 = 0x0910, DataLength = 32
    *
    * Clear ESC DC system-time related register area before ecx_configdc().
    */
   {
      uint8_t zero32[32] = {0};

      wkc = ecx_BWR(&ctx.port,
                    0,
                    0x0910,
                    sizeof(zero32),
                    zero32,
                    EC_TIMEOUTRET3);

      printf("[ENI-DC] BWR  clear dc system time  0x0910 len=32      wkc=%d\n", wkc);
   }

   /*
    * ENI Master InitCmd:
    *   clear dc cycle cfg
    *   Cmd = BWR, Ado = 2433 = 0x0981, Data = 00
    *
    * 0x0981 is the Sync activation byte in the ESC DC activation area.
    */
   {
      uint8_t clear_cycle_cfg = 0x00;

      wkc = ecx_BWR(&ctx.port,
                    0,
                    0x0981,
                    sizeof(clear_cycle_cfg),
                    &clear_cycle_cfg,
                    EC_TIMEOUTRET3);

      printf("[ENI-DC] BWR  clear dc cycle cfg    0x0981 data=00     wkc=%d\n", wkc);
   }

   /*
    * ENI Master InitCmd:
    *   reset dc speed
    *   Cmd = BWR, Ado = 2352 = 0x0930, Data = 00 10
    *
    * Keep byte order exactly as ENI frame data.
    */
   {
      uint8_t reset_dc_speed[2] = {0x00, 0x10};

      wkc = ecx_BWR(&ctx.port,
                    0,
                    0x0930,
                    sizeof(reset_dc_speed),
                    reset_dc_speed,
                    EC_TIMEOUTRET3);

      printf("[ENI-DC] BWR  reset dc speed        0x0930 data=00 10  wkc=%d\n", wkc);
   }

   /*
    * ENI Master InitCmd:
    *   configure dc filter
    *   Cmd = BWR, Ado = 2356 = 0x0934, Data = 00 0C
    *
    * Keep byte order exactly as ENI frame data.
    */
   {
      uint8_t dc_filter[2] = {0x00, 0x0C};

      wkc = ecx_BWR(&ctx.port,
                    0,
                    0x0934,
                    sizeof(dc_filter),
                    dc_filter,
                    EC_TIMEOUTRET3);

      printf("[ENI-DC] BWR  configure dc filter   0x0934 data=00 0C  wkc=%d\n", wkc);
   }

   /*
    * ENI Slave InitCmd:
    *   clear DC activation
    *   Cmd = FPWR, Ado = 2432 = 0x0980, Data = 00 00
    *
    * ecx_dcsync0() will enable Sync0 again later.
    */
   {
      uint8_t clear_dc_activation[2] = {0x00, 0x00};

      wkc = ecx_FPWR(&ctx.port,
                     configadr,
                     0x0980,
                     sizeof(clear_dc_activation),
                     clear_dc_activation,
                     EC_TIMEOUTRET3);

      printf("[ENI-DC] FPWR clear DC activation   0x0980 data=00 00  wkc=%d\n", wkc);
   }

   /*
    * ENI Slave InitCmd:
    *   set DC latch cfg
    *   Cmd = FPWR, Ado = 2472 = 0x09A8, Data = 00 00
    */
   {
      uint8_t dc_latch_cfg[2] = {0x00, 0x00};

      wkc = ecx_FPWR(&ctx.port,
                     configadr,
                     0x09A8,
                     sizeof(dc_latch_cfg),
                     dc_latch_cfg,
                     EC_TIMEOUTRET3);

      printf("[ENI-DC] FPWR set DC latch cfg      0x09A8 data=00 00  wkc=%d\n", wkc);
   }

   printf("========== ENI DC/ESC PreInit Done ==========\n\n");
}

/*
 * Execute ENI-style DC Sync0 configuration WITHOUT calling ecx_dcsync0().
 *
 * Purpose:
 *   Test the exact ENI DC activation style:
 *     0x09A0 = CycleTime0
 *     0x0990 = 0
 *     0x0980 = 00 03
 *     0x09A8 = 00 00
 *
 * Difference from ecx_dcsync0():
 *   - ecx_dcsync0() calculates a future SYNC0 start time and writes 0x0990.
 *   - ENI writes SYNC0 start time as all zero.
 *
 * This function does NOT configure PDO.
 * This function does NOT replace ecx_configdc().
 * It is intentionally called AFTER ecx_configdc() and INSTEAD OF ecx_dcsync0().
 *
 * Do NOT call from the real-time cycle.
 */
static void put_le64_bytes(uint8_t dst[8], uint64_t value)
{
   dst[0] = (uint8_t)((value >> 0) & 0xFF);
   dst[1] = (uint8_t)((value >> 8) & 0xFF);
   dst[2] = (uint8_t)((value >> 16) & 0xFF);
   dst[3] = (uint8_t)((value >> 24) & 0xFF);
   dst[4] = (uint8_t)((value >> 32) & 0xFF);
   dst[5] = (uint8_t)((value >> 40) & 0xFF);
   dst[6] = (uint8_t)((value >> 48) & 0xFF);
   dst[7] = (uint8_t)((value >> 56) & 0xFF);
}

static bool execute_eni_manual_sync0(uint16_t slave)
{
   int wkc;
   bool ok = true;
   uint16_t configadr = ctx.slavelist[slave].configadr;

   printf("\n========== ENI Manual Sync0 Test ==========" "\n");
   printf("Slave %u configadr = 0x%04X\n", slave, configadr);
   printf("This test does NOT call ecx_dcsync0().\n");
   printf("It writes ENI-style DC registers directly.\n");

   /*
    * ENI Slave InitCmd:
    *   Comment : set DC cycle time
    *   Cmd     : FPWR
    *   Adp     : 1001
    *   Ado     : 2464 = 0x09A0
    *   Data    : 00 12 7A 00 00 00 00 00 for 8ms
    *
    * Use current cycletime so command line cycle argument still works.
    */
   {
      uint8_t dc_cycle_time[8];
      put_le64_bytes(dc_cycle_time, (uint64_t)cycletime);

      wkc = ecx_FPWR(&ctx.port,
                     configadr,
                     0x09A0,
                     sizeof(dc_cycle_time),
                     dc_cycle_time,
                     EC_TIMEOUTRET3);

      printf("[ENI-SYNC0] FPWR set DC cycle time 0x09A0 cycle=%" PRId64 " ns "
             "raw=%02X %02X %02X %02X %02X %02X %02X %02X wkc=%d\n",
             cycletime,
             dc_cycle_time[0], dc_cycle_time[1], dc_cycle_time[2], dc_cycle_time[3],
             dc_cycle_time[4], dc_cycle_time[5], dc_cycle_time[6], dc_cycle_time[7],
             wkc);

      if (wkc <= 0) ok = false;
   }

   /*
    * ENI Slave InitCmd:
    *   Comment : set DC start time
    *   Cmd     : FPWR
    *   Adp     : 1001
    *   Ado     : 2448 = 0x0990
    *   Data    : 00 00 00 00 00 00 00 00
    *
    * This is the main difference from ecx_dcsync0().
    */
   {
      uint8_t dc_start_time[8] = {0};

      wkc = ecx_FPWR(&ctx.port,
                     configadr,
                     0x0990,
                     sizeof(dc_start_time),
                     dc_start_time,
                     EC_TIMEOUTRET3);

      printf("[ENI-SYNC0] FPWR set DC start time  0x0990 raw=00 00 00 00 00 00 00 00 wkc=%d\n",
             wkc);

      if (wkc <= 0) ok = false;
   }

   /*
    * ENI Slave InitCmd:
    *   Comment : set DC activation
    *   Cmd     : FPWR
    *   Adp     : 1001
    *   Ado     : 2432 = 0x0980
    *   Data    : 00 03
    *
    * Important byte order:
    *   raw 0x0980 = 00
    *   raw 0x0981 = 03
    * This matches your previous readback where 0x0981 sync activation = 0x03.
    */
   {
      uint8_t dc_activation[2] = {0x00, 0x03};

      wkc = ecx_FPWR(&ctx.port,
                     configadr,
                     0x0980,
                     sizeof(dc_activation),
                     dc_activation,
                     EC_TIMEOUTRET3);

      printf("[ENI-SYNC0] FPWR set DC activation  0x0980 data=00 03 wkc=%d\n", wkc);

      if (wkc <= 0) ok = false;
   }

   /*
    * ENI Slave InitCmd:
    *   Comment : set DC latch cfg
    *   Cmd     : FPWR
    *   Adp     : 1001
    *   Ado     : 2472 = 0x09A8
    *   Data    : 00 00
    */
   {
      uint8_t dc_latch_cfg[2] = {0x00, 0x00};

      wkc = ecx_FPWR(&ctx.port,
                     configadr,
                     0x09A8,
                     sizeof(dc_latch_cfg),
                     dc_latch_cfg,
                     EC_TIMEOUTRET3);

      printf("[ENI-SYNC0] FPWR set DC latch cfg   0x09A8 data=00 00 wkc=%d\n", wkc);

      if (wkc <= 0) ok = false;
   }

   /*
    * Keep SOEM software-side flags consistent because the RT loop uses:
    *   ctx.slavelist[i].hasdc
    *   ctx.slavelist[i].DCactive
    *   ctx.DCtime from the process-data receive path
    */
   if (ok)
   {
      ctx.slavelist[slave].DCactive = TRUE;
      ctx.slavelist[slave].DCcycle = cycletime;
      ctx.slavelist[slave].DCshift = 0;
   }

   printf("ENI manual Sync0 result: %s\n", ok ? "OK" : "FAILED");
   printf("========== ENI Manual Sync0 Done ==========" "\n\n");

   return ok;
}


void dump_rpdo_bytes(const char *tag)
{
   if (!rx_pdo) return;

   uint8_t *p = (uint8_t *)rx_pdo;

   printf("%s RPDO raw:", tag);
   for (int i = 0; i < (int)sizeof(RxPDO_t); i++)
   {
      printf(" %02X", p[i]);
   }

   printf(" | pos=%d vel=%d cw=0x%04X torque=%d mode=%d\n",
          rx_pdo->target_position,
          rx_pdo->target_velocity,
          rx_pdo->control_word,
          rx_pdo->target_torque,
          rx_pdo->operation_mode);
}

void debug_read_sm_regs(uint16_t slave, const char *tag)
{
   uint8_t sm[8] = {0};

   printf("\n========== ESC SM Register Debug: %s ==========\n", tag);

   for (int smi = 0; smi < 4; smi++)
   {
      uint16_t ado = 0x0800 + smi * 8;

      memset(sm, 0, sizeof(sm));

      if (esc_read_bytes(slave, ado, sm, 8) > 0)
      {
         uint16_t start  = le16(&sm[0]);
         uint16_t length = le16(&sm[2]);
         uint8_t  ctrl   = sm[4];
         uint8_t  status = sm[5];
         uint8_t  enable = sm[6];
         uint8_t  pdi    = sm[7];

         printf("SM%d ADO=0x%04X raw=%02X %02X %02X %02X %02X %02X %02X %02X | "
                "Start=0x%04X Len=0x%04X Ctrl=0x%02X Status=0x%02X Enable=0x%02X PDI=0x%02X\n",
                smi,
                ado,
                sm[0], sm[1], sm[2], sm[3], sm[4], sm[5], sm[6], sm[7],
                start, length, ctrl, status, enable, pdi);
      }
   }

   printf("================================================\n\n");
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
//
// 对齐 DCDemo 的关键点：
//   1. 先将 TargetPosition 固定到当前 ActualPosition。
//   2. Enable 过程使用 0x86 -> 0x06 -> 0x07 -> 0x0F。
//   3. CSP 运动阶段保持 control word = 0x000F，不使用 0x001F。
//   4. target_velocity / target_torque 保持 0。
// ==============================================
void runWork()
{
   static int step = 0;
   static bool step_printed[8] = {false};

   static int32_t base_pos = 0;
   static int32_t csp_target = 0;
   static int dir = 1;
   static int hold_cycles = 0;
   static int wait_follow_cnt = 0;
   static int enable_state_cycles = 0;

   /*
    * Step-2 诊断版：先用小范围、低速度验证驱动是否真的接受 CSP 位置给定。
    * 之前 1ms 下约 20000 count/s，TargetPosition 已明显变化但 ActualPosition 不跟随；
    * 这里先降到 2000 count/s、±5000 count，避免诊断阶段给定跳得太大。
    */
   const int32_t CSP_SPEED_COUNTS_PER_SEC = 2000;
   int32_t csp_step_per_cycle =
      (int32_t)(((int64_t)CSP_SPEED_COUNTS_PER_SEC * cycletime) / NSEC_PER_SEC);
   if (csp_step_per_cycle < 1) csp_step_per_cycle = 1;

   const int32_t CSP_MOVE_RANGE     = 5000;
   const int     CSP_HOLD_CYCLES    = (int)(NSEC_PER_SEC / cycletime);
   const int     ENABLE_HOLD_CYCLES = (int)(NSEC_PER_SEC / cycletime);

   uint16_t sw = tx_pdo ? tx_pdo->status_word : 0;

   switch (step)
   {
      case 0:
      {
         /*
          * DCDemo 的关键点：先把目标位置固定为当前实际位置。
          * 后续 0x06 / 0x07 / 0x0F 阶段不要每周期改成 actual_position，
          * 否则目标会跟着反馈抖动，不完全等价于 DCDemo。
          */
         base_pos = tx_pdo->actual_position;
         csp_target = base_pos;

         motion_cmd.control_word = 0x0086;
         motion_cmd.operation_mode = 8;
         motion_cmd.target_position = csp_target;
         motion_cmd.target_velocity = 0;
         motion_cmd.target_torque = 0;

         if (!step_printed[0]) {
            step_printed[0] = true;
            printf("[Step 0] Fault reset sent, CW:0x%04X, SW:0x%04X, fixed target:%d, mode:%d\n",
                   motion_cmd.control_word,
                   sw,
                   csp_target,
                   tx_pdo->operation_mode_display);
         }

         enable_state_cycles = 0;
         step = 100;
      }
      break;

      case 100:
      {
         motion_cmd.control_word = 0x0006;
         motion_cmd.operation_mode = 8;
         motion_cmd.target_position = csp_target;
         motion_cmd.target_velocity = 0;
         motion_cmd.target_torque = 0;

         enable_state_cycles++;
         if (((sw & 0x006F) == 0x0021) &&
             (enable_state_cycles >= ENABLE_HOLD_CYCLES)) {
            if (!step_printed[1]) {
               step_printed[1] = true;
               printf("[Step 100] Ready to switch on, CW:0x%04X, SW:0x%04X, fixed target:%d, pos:%d, mode:%d\n",
                      motion_cmd.control_word,
                      sw,
                      csp_target,
                      tx_pdo->actual_position,
                      tx_pdo->operation_mode_display);
            }
            enable_state_cycles = 0;
            step = 200;
         }
      }
      break;

      case 200:
      {
         motion_cmd.control_word = 0x0007;
         motion_cmd.operation_mode = 8;
         motion_cmd.target_position = csp_target;
         motion_cmd.target_velocity = 0;
         motion_cmd.target_torque = 0;

         enable_state_cycles++;
         if (((sw & 0x006F) == 0x0023) &&
             (enable_state_cycles >= ENABLE_HOLD_CYCLES)) {
            if (!step_printed[2]) {
               step_printed[2] = true;
               printf("[Step 200] Switched on, CW:0x%04X, SW:0x%04X, fixed target:%d, pos:%d, mode:%d\n",
                      motion_cmd.control_word,
                      sw,
                      csp_target,
                      tx_pdo->actual_position,
                      tx_pdo->operation_mode_display);
            }
            enable_state_cycles = 0;
            step = 300;
         }
      }
      break;

      case 300:
      {
         motion_cmd.control_word = 0x000F;
         motion_cmd.operation_mode = 8;
         motion_cmd.target_position = csp_target;
         motion_cmd.target_velocity = 0;
         motion_cmd.target_torque = 0;

         bool operation_enabled = ((sw & 0x006F) == 0x0027);

         /*
          * 对齐已经能正常运动的 IgH demo：
          *   0x000F 保持固定周期后进入运动；
          *   6041 bit12 只作为观察信息打印，不作为进入运动的硬条件。
          *
          * 你的日志中 SW=0x0637 已经表示 Operation enabled，
          * 但 bit12 一直为 0，原逻辑会永久卡在 Step 300。
          */
         if (operation_enabled)
         {
            enable_state_cycles++;

            if (enable_state_cycles >= ENABLE_HOLD_CYCLES)
            {
               if (!step_printed[3])
               {
                  step_printed[3] = true;
                  printf("[Step 300] Operation enabled, CW:0x%04X, SW:0x%04X, bit12=%d, fixed target:%d, pos:%d, mode:%d\n",
                         motion_cmd.control_word,
                         sw,
                         (sw & 0x1000) ? 1 : 0,
                         csp_target,
                         tx_pdo->actual_position,
                         tx_pdo->operation_mode_display);
               }

               enable_state_cycles = 0;
               step = 400;
            }
         }
         else
         {
            enable_state_cycles = 0;

            if ((wait_follow_cnt++ % 1000) == 0)
            {
               printf("[Step 300] waiting operation enabled, CW:0x%04X, SW:0x%04X, op_en=%d, bit12=%d, target:%d, pos:%d\n",
                      motion_cmd.control_word,
                      sw,
                      operation_enabled ? 1 : 0,
                      (sw & 0x1000) ? 1 : 0,
                      csp_target,
                      tx_pdo->actual_position);
            }
         }
      }
      break;

      case 400:
      {
         /* 真正开始运动前，再以当前实际位置作为往复中心。 */
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
            printf("[Step 400] CSP motion armed, CW:0x%04X, SW:0x%04X, base:%d, mode:%d\n",
                   motion_cmd.control_word,
                   sw,
                   base_pos,
                   tx_pdo->operation_mode_display);
         }
         step = 401;
      }
      break;

      case 401:
      {
         motion_cmd.control_word = 0x000F;   // DCDemo / IgH 运动阶段保持 0x000F
         motion_cmd.operation_mode = 8;
         motion_cmd.target_torque = 0;

         if (hold_cycles > 0) {
            hold_cycles--;
            motion_cmd.target_position = csp_target;
            motion_cmd.target_velocity = 0;
            break;
         }

         csp_target += dir * csp_step_per_cycle;
         motion_cmd.target_velocity = dir * CSP_SPEED_COUNTS_PER_SEC;

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

/*
 * Read drive-side CoE DC diagnostic objects once during startup.
 * Do not call this from the real-time cycle or OP status loop.
 * The drive may report values different from ESC registers; this is diagnostic only.
 */
void debug_read_drive_dc_diag(uint16_t slave, const char *tag)
{
   uint16_t sm_sync_type = 0;
   uint16_t sm_event_missed = 0;
   uint16_t cycle_too_small = 0;
   uint8_t  sync_error = 0;
   uint32_t sync0_cycle = 0;
   uint32_t cycle_time = 0;

   printf("\n========== Drive CoE DC Diagnostic: %s ==========\n", tag);

   if (sdo_read(&ctx, slave, 0x1C32, 0x01, &sm_sync_type, sizeof(sm_sync_type)) > 0)
      printf("1C32:01 SM Sync Type        = %u\n", sm_sync_type);

   if (sdo_read(&ctx, slave, 0x1C32, 0x02, &cycle_time, sizeof(cycle_time)) > 0)
      printf("1C32:02 Cycle Time          = %u ns\n", cycle_time);

   if (sdo_read(&ctx, slave, 0x1C32, 0x0A, &sync0_cycle, sizeof(sync0_cycle)) > 0)
      printf("1C32:0A Sync0 Cycle Time    = %u ns\n", sync0_cycle);

   if (sdo_read(&ctx, slave, 0x1C32, 0x0B, &sm_event_missed, sizeof(sm_event_missed)) > 0)
      printf("1C32:0B SM-Event Missed     = %u\n", sm_event_missed);

   if (sdo_read(&ctx, slave, 0x1C32, 0x0C, &cycle_too_small, sizeof(cycle_too_small)) > 0)
      printf("1C32:0C Cycle Time TooSmall = %u\n", cycle_too_small);

   if (sdo_read(&ctx, slave, 0x1C32, 0x20, &sync_error, sizeof(sync_error)) > 0)
      printf("1C32:20 Sync Error          = %u\n", sync_error);

   memset(&sm_event_missed, 0, sizeof(sm_event_missed));
   memset(&cycle_too_small, 0, sizeof(cycle_too_small));
   memset(&sync_error, 0, sizeof(sync_error));
   memset(&sync0_cycle, 0, sizeof(sync0_cycle));
   memset(&cycle_time, 0, sizeof(cycle_time));
   memset(&sm_sync_type, 0, sizeof(sm_sync_type));


   if (sdo_read(&ctx, slave, 0x1C33, 0x01, &sm_sync_type, sizeof(sm_sync_type)) > 0)
      printf("1C33:01 SM Sync Type        = %u\n", sm_sync_type);

   if (sdo_read(&ctx, slave, 0x1C33, 0x02, &cycle_time, sizeof(cycle_time)) > 0)
      printf("1C33:02 Cycle Time          = %u ns\n", cycle_time);

   if (sdo_read(&ctx, slave, 0x1C33, 0x0A, &sync0_cycle, sizeof(sync0_cycle)) > 0)
      printf("1C33:0A Sync0 Cycle Time    = %u ns\n", sync0_cycle);

   if (sdo_read(&ctx, slave, 0x1C33, 0x0B, &sm_event_missed, sizeof(sm_event_missed)) > 0)
      printf("1C33:0B SM-Event Missed     = %u\n", sm_event_missed);

   if (sdo_read(&ctx, slave, 0x1C33, 0x0C, &cycle_too_small, sizeof(cycle_too_small)) > 0)
      printf("1C33:0C Cycle Time TooSmall = %u\n", cycle_too_small);

   if (sdo_read(&ctx, slave, 0x1C33, 0x20, &sync_error, sizeof(sync_error)) > 0)
      printf("1C33:20 Sync Error          = %u\n", sync_error);

   printf("=================================================\n\n");
}

/*
 * Check the reference-time phase echoed by the FPWR/FRMW pair in SAFE_OP.
 *
 * What this verifies:
 *   - RT process-data cycle is running.
 *   - WKC is equal to expectedWKC.
 *   - The reference time written by this application has the requested phase.
 *
 * What this does NOT prove by itself:
 *   - This is not an independent measurement of SYNC0/PDO alignment because
 *     FRMW reads the reference clock immediately after FPWR updates it.
 *   - The drive application layer has accepted CSP/DC synchronization.
 *     That still needs to be confirmed by 6041 bit12 = 1 and ActualPosition
 *     following TargetPosition.
 */
bool wait_dc_sync_stable(int64_t limit_ns, int stable_cycles, int timeout_ms)
{
   int stable_cnt = 0;
   int max_stable_cnt = 0;
   int valid_samples = 0;
   int good_samples = 0;
   int bad_wkc_samples = 0;
   int over_limit_samples = 0;
   int64_t max_abs_err = 0;
   int64_t sum_abs_err = 0;
   int timeout_cycles = (int)((timeout_ms * 1000000LL) / cycletime);

   printf("\n========== DC Reference Phase Precheck ==========\n");
   printf("Cycle time        : %" PRId64 " ns\n", cycletime);
   printf("Sync offset       : %" PRId64 " ns\n", syncoffset);
   printf("Error limit       : %" PRId64 " ns\n", limit_ns);
   printf("Need stable       : %d cycles\n", stable_cycles);
   printf("Timeout           : %d ms (%d cycles)\n", timeout_ms, timeout_cycles);
   printf("Expected WKC      : %d\n", expectedWKC);
   printf("=============================================\n");

   for (int i = 0; i < timeout_cycles && run; i++)
   {
      osal_usleep(cycletime / 1000);

      /*
       * Wait until the RT thread has entered the cyclic loop and has received
       * several frames. Before that, timeerror/wkc may still be stale.
       */
      if (total_cycles < 10)
      {
         continue;
      }

      int64_t abs_err = llabs(timeerror);
      valid_samples++;
      sum_abs_err += abs_err;

      if (abs_err > max_abs_err)
      {
         max_abs_err = abs_err;
      }

      if (wkc != expectedWKC)
      {
         bad_wkc_samples++;
         stable_cnt = 0;
      }
      else if (abs_err <= limit_ns)
      {
         good_samples++;
         stable_cnt++;
         if (stable_cnt > max_stable_cnt)
         {
            max_stable_cnt = stable_cnt;
         }
      }
      else
      {
         over_limit_samples++;
         stable_cnt = 0;
      }

      if ((i % DC_VERIFY_PRINT_EVERY) == 0)
      {
         int64_t avg_abs_err = valid_samples ? (sum_abs_err / valid_samples) : 0;

         printf("DC verify: cycle=%" PRId64
                ", err=%" PRId64 " ns"
                ", abs=%" PRId64 " ns"
                ", avg_abs=%" PRId64 " ns"
                ", max_abs=%" PRId64 " ns"
                ", stable=%d/%d"
                ", max_stable=%d"
                ", good=%d/%d"
                ", wkc=%d/%d"
                ", bad_wkc=%d"
                ", over_limit=%d\n",
                total_cycles,
                timeerror,
                abs_err,
                avg_abs_err,
                max_abs_err,
                stable_cnt,
                stable_cycles,
                max_stable_cnt,
                good_samples,
                valid_samples,
                wkc,
                expectedWKC,
                bad_wkc_samples,
                over_limit_samples);
      }

      if (stable_cnt >= stable_cycles)
      {
         int64_t avg_abs_err = valid_samples ? (sum_abs_err / valid_samples) : 0;

         printf("\n========== DC Reference Phase Precheck Result ==========\n");
         printf("Result            : PASS\n");
         printf("Final error       : %" PRId64 " ns\n", timeerror);
         printf("Avg abs error     : %" PRId64 " ns\n", avg_abs_err);
         printf("Max abs error     : %" PRId64 " ns\n", max_abs_err);
         printf("Stable cycles     : %d\n", stable_cnt);
         printf("Max stable cycles : %d\n", max_stable_cnt);
         printf("Good samples      : %d / %d\n", good_samples, valid_samples);
         printf("Bad WKC samples   : %d\n", bad_wkc_samples);
         printf("Over-limit samples: %d\n", over_limit_samples);
         printf("===================================================\n\n");
         return true;
      }
   }

   int64_t avg_abs_err = valid_samples ? (sum_abs_err / valid_samples) : 0;

   printf("\n========== DC Reference Phase Precheck Result ==========\n");
   printf("Result            : FAIL\n");
   printf("Final error       : %" PRId64 " ns\n", timeerror);
   printf("Avg abs error     : %" PRId64 " ns\n", avg_abs_err);
   printf("Max abs error     : %" PRId64 " ns\n", max_abs_err);
   printf("Stable cycles     : %d\n", stable_cnt);
   printf("Max stable cycles : %d\n", max_stable_cnt);
   printf("Good samples      : %d / %d\n", good_samples, valid_samples);
   printf("Bad WKC samples   : %d\n", bad_wkc_samples);
   printf("Over-limit samples: %d\n", over_limit_samples);
   printf("===================================================\n\n");

   return false;
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
   if (ts.tv_nsec >= NSEC_PER_SEC)
   {
      ts.tv_sec++;
      ts.tv_nsec -= NSEC_PER_SEC;
   }
#if ENABLE_IGH_REFERENCE_SYNC
   igh_set_application_time_from_ts(&ts);
#endif
   send_cyclic_processdata();

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
#if ENABLE_CYCLE_DC_TIME_PRINT
         if (total_cycles % 1000 == 0)
         {
            printf("DC TIME: %ld ns\n", ctx.DCtime);
         }
#endif
         // ==============================================
         // EtherCAT 通讯：收上一周期返回帧
         // IGH 抓包路径为 FPWR(DC32) + FRMW(DC32) + LRW(PDO)。
         // ==============================================
         wkc = receive_cyclic_processdata(EC_TIMEOUTRET);
         dowkccheck = (wkc == expectedWKC) ? 0 : dowkccheck + 1;

         if (ctx.slavelist[1].hasdc && (wkc > 0))
         {
#if ENABLE_IGH_REFERENCE_SYNC
            update_dc_error(ctx.DCtime);
            toff = 0;
#else
            ec_sync(ctx.DCtime, cycletime, &toff);
#endif
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

#if !ENABLE_IGH_REFERENCE_SYNC
         ecx_mbxhandler(&ctx, 0, 4);
#endif
#if ENABLE_IGH_REFERENCE_SYNC
         igh_set_application_time_from_ts(&ts);
#endif
         send_cyclic_processdata();
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
    * Keep this mapping aligned with DCDemo / ENI.
    *
    * Do not write 1C32/1C33 here:
    *   - This drive has shown SAFE_OP errors when forcing SM sync type.
    *   - ENI startup does not require CoE writes to 1C32/1C33.
    *   - We only read these objects as diagnostics later.
    */
   if (!configure_pdo(slave))
   {
      ecx_close(&ctx);
      return;
   }

   /* 2. Map PDO to IOmap */
   ecx_config_map_group(&ctx, IOmap, 0);
#if ENABLE_STARTUP_ESC_DEBUG
   debug_read_sm_regs(slave, "after ecx_config_map_group");
#endif
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

   /* Optional ENI reset/filter/latch experiment; disabled in the IGH baseline. */
#if ENABLE_ENI_DC_PREINIT
   execute_eni_dc_preinit(slave);
#else
   printf("ENI DC preinit disabled for IGH-aligned baseline\n");
#endif

#if ENABLE_STARTUP_ESC_DEBUG
   debug_read_esc_dc_regs(slave, "after optional ENI DC preinit, before ecx_configdc");
#endif

   printf("\nConfiguring Distributed Clocks...\n");
   ecx_configdc(&ctx);
#if ENABLE_STARTUP_ESC_DEBUG
   debug_read_esc_dc_regs(slave, "after ecx_configdc, before dcsync0");
#endif

#if ENABLE_IGH_REFERENCE_SYNC
   {
      uint32_t process_length = group->Obytes + group->Ibytes;

      if (group->blockLRW || !group->hasdc || !group->DCnext ||
          !group->outputs || !group->nsegments ||
          !process_length || process_length > group->IOsegment[0])
      {
         printf("ERROR: IO mapping is not compatible with the single-frame IGH reference path\n");
         ecx_close(&ctx);
         return;
      }

      printf("IGH reference cyclic frame: FPWR(DC32) + FRMW(DC32) + LRW(%u bytes)\n",
             process_length);
      printf("Mailbox status bytes excluded from LRW: %d\n",
             group->mbxstatuslength);
   }
#endif

   /*
    * The 0x092C convergence loop is only meaningful for the experimental
    * IGH-style custom cyclic frame. In the standard SOEM path, keep startup
    * simple: configdc() -> dcsync0() -> SAFE_OP warmup -> OP.
    */
#if ENABLE_IGH_REFERENCE_SYNC
   if (!wait_dc_start_convergence(slave))
   {
      fprintf(stderr, "WARNING: DC 0x092C did not converge before SYNC0 activation, continue for Step-3 observation\n");
   }
#else
   printf("Standard SOEM DC path: skip IGH-style 0x092C start convergence\n");
#endif

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
#if ENABLE_STARTUP_ESC_DEBUG
   debug_read_esc_dc_regs(slave, "after ecx_dcsync0");
#endif


#if ENABLE_STARTUP_COE_DC_DIAG
   debug_read_drive_dc_diag(slave, "before SAFE_OP");
#endif

   /* 4. The IGH comparison keeps mailbox traffic out of the PDO cycle. */
   for (int si = 1; si <= ctx.slavecount; si++)
   {
      ec_slavet *slv = &ctx.slavelist[si];

      printf("Slave %d hasdc: %d, DCactive: %d\n",
             si, slv->hasdc, slv->DCactive);

      if (slv->CoEdetails > 0)
      {
#if ENABLE_IGH_REFERENCE_SYNC
         printf("Slave %d cyclic mailbox handler disabled in IGH reference mode\n", si);
#else
         ecx_slavembxcyclic(&ctx, si);
         printf("Slave %d added to cyclic mailbox handler\n", si);
#endif
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
#if ENABLE_STARTUP_ESC_DEBUG
   debug_read_sm_regs(slave, "after SAFE_OP");
#endif

   /*
    * 6. SAFE_OP 下先交换PDO，读到实际位置。
    * CSP启动前，TargetPosition 必须先等于 ActualPosition，防止一进OP就跳变。
    */
   printf("PDO warmup in SAFE_OP start...\n");

   for (int i = 0; i < 300; i++)
   {
      send_cyclic_processdata();
      wkc = receive_cyclic_processdata(EC_TIMEOUTRET);
      osal_usleep(cycletime / 1000);
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
      send_cyclic_processdata();
      wkc = receive_cyclic_processdata(EC_TIMEOUTRET);
      osal_usleep(cycletime / 1000);
   }

   printf("PDO warmup done, WKC=%d, expectedWKC=%d\n", wkc, expectedWKC);

   /*
    * 7. 现在才允许 RT 线程开始周期PDO。
    * 这里 inOP 仍然是 0，所以 RT线程只做 PDO send/receive，不会执行 runWork。
    */
   mappingdone = 1;
   dorun = 1;

   /*
    * 7.1 Reference-time phase precheck in SAFE_OP.
    *
    * With ENABLE_IGH_REFERENCE_SYNC the cyclic frame is:
    *   FPWR(DC32) + FRMW(DC32) + LRW(process data)
    *
    * This only checks WKC and the reference-time phase echoed by FPWR/FRMW.
    * The decisive drive-side check remains status-word bit12 after 0x000F.
    */
#if ENABLE_SAFEOP_DC_PRECHECK
   bool dc_verify_ok = wait_dc_sync_stable(DC_VERIFY_LIMIT_NS,
                                           DC_VERIFY_STABLE_CYCLES,
                                           DC_VERIFY_TIMEOUT_MS);
#else
   bool dc_verify_ok = true;
   printf("SAFE_OP DC phase precheck skipped for Step-1 control validation\n");
#endif

#if ENABLE_STARTUP_ESC_DEBUG
   debug_read_esc_dc_regs(slave, "after long SAFE_OP DC verification");
#endif

#if ENABLE_STARTUP_COE_DC_DIAG
   debug_read_drive_dc_diag(slave, "after long SAFE_OP DC verification");
#endif

#if DC_VERIFY_REQUIRE_PASS
   if (!dc_verify_ok)
   {
      printf("ERROR: DC verification failed, stop before OP by policy.\n");
      dorun = 0;
      osal_usleep(100000);
      ecx_close(&ctx);
      return;
   }
#else
   if (!dc_verify_ok)
   {
      printf("WARNING: DC verification failed, continue to OP for observation because DC_VERIFY_REQUIRE_PASS=0.\n");
   }
#endif

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
#if ENABLE_STARTUP_ESC_DEBUG
   debug_read_sm_regs(slave, "after OP");
#endif
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
#if ENABLE_STARTUP_ESC_DEBUG
   debug_read_esc_dc_regs(slave, "after OP");
#endif

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
         printf("Actual Torque    : %d\n", tx_pdo->actual_torque);
         printf("SW bit10 TargetReached : %d\n", (sw & 0x0400) ? 1 : 0);
         printf("SW bit11 InternalLimit : %d\n", (sw & 0x0800) ? 1 : 0);
         printf("SW bit12 DriveFollows  : %d\n", (sw & 0x1000) ? 1 : 0);
         printf("SW bit13 FollowingErr? : %d\n", (sw & 0x2000) ? 1 : 0);
      }

      if (rx_pdo)
      {
         dump_rpdo_bytes("SOEM");
         printf("Control Word     : 0x%04X\n", rx_pdo->control_word);
         printf("Target Position  : %d\n", rx_pdo->target_position);
         printf("Target Velocity  : %d\n", rx_pdo->target_velocity);
         printf("Target Torque    : %d\n", rx_pdo->target_torque);
         printf("Target Mode      : %d\n", rx_pdo->operation_mode);
      }

#if ENABLE_OP_SDO_DEBUG
      /*
       * Optional mailbox debug.
       * Keep disabled during normal CSP testing to avoid OP-loop SDO traffic.
       */
      static uint16_t sdo_control_word = 0;
      static uint16_t sdo_status_word = 0;
      static int8_t sdo_operation_mode = 0;
      static int8_t sdo_operation_mode_display = 0;
      static int32_t sdo_target_position = 0;
      static int32_t sdo_actual_position = 0;
      static int32_t sdo_target_velocity = 0;

      if (tx_pdo && rx_pdo)
      {
         int size;

         size = sizeof(sdo_control_word);
         if (ecx_SDOread(&ctx, 1, 0x6040, 0x00, FALSE,
                         &size, &sdo_control_word, EC_TIMEOUTRXM) > 0)
         {
            printf("SDO 6040 Control Word    : 0x%04X\n", sdo_control_word);
         }

         size = sizeof(sdo_status_word);
         if (ecx_SDOread(&ctx, 1, 0x6041, 0x00, FALSE,
                         &size, &sdo_status_word, EC_TIMEOUTRXM) > 0)
         {
            printf("SDO 6041 Status Word     : 0x%04X\n", sdo_status_word);
         }

         size = sizeof(sdo_operation_mode);
         if (ecx_SDOread(&ctx, 1, 0x6060, 0x00, FALSE,
                         &size, &sdo_operation_mode, EC_TIMEOUTRXM) > 0)
         {
            printf("SDO 6060 Operation Mode  : %d\n", sdo_operation_mode);
         }

         size = sizeof(sdo_operation_mode_display);
         if (ecx_SDOread(&ctx, 1, 0x6061, 0x00, FALSE,
                         &size, &sdo_operation_mode_display, EC_TIMEOUTRXM) > 0)
         {
            printf("SDO 6061 Mode Display    : %d\n", sdo_operation_mode_display);
         }

         size = sizeof(sdo_target_position);
         if (ecx_SDOread(&ctx, 1, 0x607A, 0x00, FALSE,
                         &size, &sdo_target_position, EC_TIMEOUTRXM) > 0)
         {
            printf("SDO 607A Target Position : %d\n", sdo_target_position);
         }

         size = sizeof(sdo_target_velocity);
         if (ecx_SDOread(&ctx, 1, 0x60FF, 0x00, FALSE,
                         &size, &sdo_target_velocity, EC_TIMEOUTRXM) > 0)
         {
            printf("SDO 60FF Target Velocity : %d\n", sdo_target_velocity);
         }

         size = sizeof(sdo_actual_position);
         if (ecx_SDOread(&ctx, 1, 0x6064, 0x00, FALSE,
                         &size, &sdo_actual_position, EC_TIMEOUTRXM) > 0)
         {
            printf("SDO 6064 Actual Position : %d\n", sdo_actual_position);
         }
      }
#endif

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
   printf("SOEM EtherCAT Master (IGH-aligned CSP baseline)\n");
#if ENABLE_IGH_REFERENCE_SYNC
   printf("Cyclic mode: IGH-style FPWR(DC32) + FRMW(DC32) + LRW(PDO)\n\n");
#else
   printf("Cyclic mode: standard SOEM LRW(PDO) + FRMW(DC64)\n\n");
#endif

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
      cycletime = (int64_t)atoi(argv[2]) * 1000;
      if (cycletime <= 0)
      {
         fprintf(stderr, "ERROR: cycletime must be greater than zero\n");
         return 1;
      }
      syncoffset = 0;
   }

   if (argc > 3)
   {
      /*
       * 第三个参数用于扫相位：
       *   <= 100 : 认为是百分比，例如 20 / 50 / 66.666 / 80
       *   >  100 : 认为是 ns，例如 5333333
       */
      double arg_sync = atof(argv[3]);

      if ((arg_sync >= 0.0) && (arg_sync <= 100.0))
      {
         syncoffset = (int64)((double)cycletime * arg_sync / 100.0);
      }
      else if (arg_sync > 100.0)
      {
         syncoffset = (int64)arg_sync;
      }
      else
      {
         printf("WARNING: invalid syncoffset argument '%s', keep zero phase.\n", argv[3]);
      }

      if (syncoffset < 0) syncoffset = 0;
      if (syncoffset >= cycletime) syncoffset = syncoffset % cycletime;
   }

   printf("Using cycle time: %" PRId64 " us, syncoffset=%" PRId64 " ns (%.3f%% of cycle)\n",
          cycletime / 1000,
          syncoffset,
          ((double)syncoffset * 100.0) / (double)cycletime);

#if ENABLE_IGH_REFERENCE_SYNC
   printf("IGH-style reference-clock synchronization is ENABLED\n");
   if (cycletime != 1000000)
   {
      printf("NOTE: current master cycle is %" PRId64 " us\n",
             cycletime / 1000);
   }
#endif

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
      printf("Usage: %s ifname [cycletime_us] [sync_phase_percent_or_ns]\n", argv[0]);
      printf("ifname = eth0 for example\n");
      printf("default cycletime = 1000 us, default sync phase = 0\n");

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
