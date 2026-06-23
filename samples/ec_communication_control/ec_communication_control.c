#define _GNU_SOURCE
/*
 * Clean SOEM EtherCAT CSP demo
 *
 * Keep only:
 *   ecx_configdc()
 *   ecx_dcsync0()
 *   configure_pdo()
 *   ecx_config_map_group()
 *   SAFE_OP
 *   OP
 *
 * Periodic communication uses standard SOEM APIs only:
 *   ecx_send_processdata()
 *   ecx_receive_processdata()
 *
 * Manual PDO configuration is kept because this drive's EEPROM/default PDO
 * mapping is incomplete for the tested CSP mapping.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <limits.h>
#include <pthread.h>
#include <sys/mman.h>
#include <math.h>

#include "soem/soem.h"

#define EC_TIMEOUTMON 500
#define NSEC_PER_SEC  1000000000LL
#define US_PER_NSEC   1000LL

#define DEFAULT_CYCLE_TIME_NS 1000000LL   /* 1 ms */
#define SAFEOP_WARMUP_CYCLES  300
#define PRINT_PERIOD_CYCLES   1000

static uint8 IOmap[4096];
static ecx_contextt ctx;

static OSAL_THREAD_HANDLE threadrt;
static OSAL_THREAD_HANDLE threadcheck;

static volatile int run = 1;
static volatile int mappingdone = 0;
static volatile int dorun = 0;
static volatile int inOP = 0;

static int expectedWKC = 0;
static int wkc = 0;
static int dowkccheck = 0;
static int currentgroup = 0;

static int64_t cycletime = DEFAULT_CYCLE_TIME_NS;
static int64_t syncoffset = 0;
static int64_t timeerror = 0;

/* Basic timing statistics, intentionally small. */
static int64_t total_cycles = 0;
static int64_t last_cycle_ts = 0;
static int64_t current_cycle_ns = 0;
static int64_t max_cycle_ns = 0;
static int64_t min_cycle_ns = INT64_MAX;
static int64_t max_dc_error_ns = 0;

typedef struct
{
   int32_t target_position; /* 0x607A:00 */
   int32_t target_velocity; /* 0x60FF:00 */
   uint16_t control_word;   /* 0x6040:00 */
   int16_t target_torque;   /* 0x6071:00 */
   int8_t operation_mode;   /* 0x6060:00 */
} __attribute__((packed)) RxPDO_t;

typedef struct
{
   int32_t actual_position;       /* 0x6064:00 */
   uint16_t error_code;           /* 0x603F:00 */
   int32_t actual_velocity;       /* 0x606C:00 */
   uint16_t status_word;          /* 0x6041:00 */
   int16_t actual_torque;         /* 0x6077:00 */
   int8_t operation_mode_display; /* 0x6061:00 */
} __attribute__((packed)) TxPDO_t;

typedef struct
{
   int32_t target_position;
   int32_t target_velocity;
   uint16_t control_word;
   int16_t target_torque;
   int8_t operation_mode;
} MotionCmd_t;

static RxPDO_t *rx_pdo = NULL;
static TxPDO_t *tx_pdo = NULL;
static MotionCmd_t motion_cmd;

static void add_time_ns(ec_timet *ts, int64_t addtime)
{
   ec_timet addts;
   addts.tv_nsec = addtime % NSEC_PER_SEC;
   addts.tv_sec = (addtime - addts.tv_nsec) / NSEC_PER_SEC;
   osal_timespecadd(ts, &addts, ts);
}

static int64_t monotonic_raw_ns(void)
{
   struct timespec now;
   clock_gettime(CLOCK_MONOTONIC_RAW, &now);
   return (int64_t)now.tv_sec * NSEC_PER_SEC + now.tv_nsec;
}

/* PI correction from SOEM examples: sync Linux wakeup to DC time. */
static void ec_sync(int64 reftime, int64 cycletime_ns, int64 *offsettime)
{
   static int64 integral = 0;
   const double pgain = 0.01;
   const double igain = 0.00002;

   int64 delta = (reftime - syncoffset) % cycletime_ns;
   if (delta > (cycletime_ns / 2))
      delta -= cycletime_ns;
   else if (delta < -(cycletime_ns / 2))
      delta += cycletime_ns;

   timeerror = -delta;
   int64_t abs_err = llabs(timeerror);
   if (abs_err > max_dc_error_ns)
      max_dc_error_ns = abs_err;

   integral += timeerror;
   *offsettime = (int64)((timeerror * pgain) + (integral * igain));
}

static int sdo_write(uint16_t slave, uint16_t index, uint8_t subindex,
                     const void *data, int size)
{
   int w = ecx_SDOwrite(&ctx, slave, index, subindex, FALSE,
                        size, (void *)data, EC_TIMEOUTRXM);
   if (w <= 0)
   {
      printf("SDO write failed: 0x%04X:%02X, wkc=%d\n", index, subindex, w);
      return -1;
   }

   usleep(10000);
   return 0;
}

/*
 * Manual PDO configuration.
 * Keep this part because this drive needs explicit PDO remapping.
 */
static bool configure_pdo(uint16_t slave)
{
   uint8_t zero = 0;
   uint8_t one = 1;
   uint32_t mapping;
   uint16_t pdo_index;
   uint8_t rx_entries = 5;
   uint8_t tx_entries = 6;

   printf("Configuring PDO mapping...\n");

   ecx_statecheck(&ctx, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
   if (ctx.slavelist[slave].state != EC_STATE_PRE_OP)
   {
      printf("Slave %u is not PRE-OP, state=0x%02X\n",
             slave, ctx.slavelist[slave].state);
      return false;
   }

   /* RxPDO 0x1600: 607A, 60FF, 6040, 6071, 6060 */
   if (sdo_write(slave, 0x1600, 0x00, &zero, sizeof(zero)) != 0) return false;

   mapping = 0x607A0020; if (sdo_write(slave, 0x1600, 0x01, &mapping, sizeof(mapping)) != 0) return false;
   mapping = 0x60FF0020; if (sdo_write(slave, 0x1600, 0x02, &mapping, sizeof(mapping)) != 0) return false;
   mapping = 0x60400010; if (sdo_write(slave, 0x1600, 0x03, &mapping, sizeof(mapping)) != 0) return false;
   mapping = 0x60710010; if (sdo_write(slave, 0x1600, 0x04, &mapping, sizeof(mapping)) != 0) return false;
   mapping = 0x60600008; if (sdo_write(slave, 0x1600, 0x05, &mapping, sizeof(mapping)) != 0) return false;

   if (sdo_write(slave, 0x1600, 0x00, &rx_entries, sizeof(rx_entries)) != 0) return false;

   if (sdo_write(slave, 0x1C12, 0x00, &zero, sizeof(zero)) != 0) return false;
   pdo_index = 0x1600;
   if (sdo_write(slave, 0x1C12, 0x01, &pdo_index, sizeof(pdo_index)) != 0) return false;
   if (sdo_write(slave, 0x1C12, 0x00, &one, sizeof(one)) != 0) return false;

   /* TxPDO 0x1A00: 6064, 603F, 606C, 6041, 6077, 6061 */
   if (sdo_write(slave, 0x1A00, 0x00, &zero, sizeof(zero)) != 0) return false;

   mapping = 0x60640020; if (sdo_write(slave, 0x1A00, 0x01, &mapping, sizeof(mapping)) != 0) return false;
   mapping = 0x603F0010; if (sdo_write(slave, 0x1A00, 0x02, &mapping, sizeof(mapping)) != 0) return false;
   mapping = 0x606C0020; if (sdo_write(slave, 0x1A00, 0x03, &mapping, sizeof(mapping)) != 0) return false;
   mapping = 0x60410010; if (sdo_write(slave, 0x1A00, 0x04, &mapping, sizeof(mapping)) != 0) return false;
   mapping = 0x60770010; if (sdo_write(slave, 0x1A00, 0x05, &mapping, sizeof(mapping)) != 0) return false;
   mapping = 0x60610008; if (sdo_write(slave, 0x1A00, 0x06, &mapping, sizeof(mapping)) != 0) return false;

   if (sdo_write(slave, 0x1A00, 0x00, &tx_entries, sizeof(tx_entries)) != 0) return false;

   if (sdo_write(slave, 0x1C13, 0x00, &zero, sizeof(zero)) != 0) return false;
   pdo_index = 0x1A00;
   if (sdo_write(slave, 0x1C13, 0x01, &pdo_index, sizeof(pdo_index)) != 0) return false;
   if (sdo_write(slave, 0x1C13, 0x00, &one, sizeof(one)) != 0) return false;

   printf("PDO mapping done\n");
   return true;
}

/*
 * Latest motion/control logic:
 *   0x06 -> 0x07 -> 0x0F -> 10s sine CSP motion
 *   target_velocity = 0
 *   target_torque   = 0
 *   operation_mode  = 8
 */
static void runWork(void)
{
   enum {
      CONTROL_WAIT_STATUS = 0,
      CONTROL_ENABLE_06,
      CONTROL_ENABLE_07,
      CONTROL_ENABLE_15,
      CONTROL_SINE_MOTION,
   };

   static int control_state = CONTROL_WAIT_STATUS;
   static uint64_t control_state_cycles = 0;
   static uint64_t motion_cycles = 0;
   static int32_t sine_base_position = 0;
   static bool printed[5] = {false};

   const uint64_t enable_step_cycles = (uint64_t)(NSEC_PER_SEC / cycletime);
   const uint64_t sine_period_cycles = (uint64_t)(10000000000LL / cycletime);
   const int32_t sine_range_counts = 30000;
   const double two_pi = 6.28318530717958647692;

   if (!tx_pdo || !rx_pdo)
      return;

   uint16_t sw = tx_pdo->status_word;

   if (sw == 0)
   {
      motion_cmd.target_position = tx_pdo->actual_position;
      motion_cmd.target_velocity = 0;
      motion_cmd.control_word = 0x0000;
      motion_cmd.target_torque = 0;
      motion_cmd.operation_mode = 8;
      return;
   }

   switch (control_state)
   {
      case CONTROL_WAIT_STATUS:
         motion_cmd.target_position = tx_pdo->actual_position;
         motion_cmd.target_velocity = 0;
         motion_cmd.control_word = 0x0006;
         motion_cmd.target_torque = 0;
         motion_cmd.operation_mode = 8;
         if (!printed[0])
         {
            printed[0] = true;
            printf("State: ENABLE_06\n");
         }
         control_state_cycles = 0;
         control_state = CONTROL_ENABLE_06;
         break;

      case CONTROL_ENABLE_06:
         motion_cmd.target_position = tx_pdo->actual_position;
         motion_cmd.target_velocity = 0;
         motion_cmd.control_word = 0x0006;
         motion_cmd.target_torque = 0;
         motion_cmd.operation_mode = 8;
         if (++control_state_cycles >= enable_step_cycles)
         {
            if (!printed[1])
            {
               printed[1] = true;
               printf("State: ENABLE_07\n");
            }
            control_state_cycles = 0;
            control_state = CONTROL_ENABLE_07;
         }
         break;

      case CONTROL_ENABLE_07:
         motion_cmd.target_position = tx_pdo->actual_position;
         motion_cmd.target_velocity = 0;
         motion_cmd.control_word = 0x0007;
         motion_cmd.target_torque = 0;
         motion_cmd.operation_mode = 8;
         if (++control_state_cycles >= enable_step_cycles)
         {
            if (!printed[2])
            {
               printed[2] = true;
               printf("State: ENABLE_15\n");
            }
            control_state_cycles = 0;
            control_state = CONTROL_ENABLE_15;
         }
         break;

      case CONTROL_ENABLE_15:
         motion_cmd.target_position = tx_pdo->actual_position;
         motion_cmd.target_velocity = 0;
         motion_cmd.control_word = 0x000F;
         motion_cmd.target_torque = 0;
         motion_cmd.operation_mode = 8;
         if (++control_state_cycles >= enable_step_cycles)
         {
            sine_base_position = tx_pdo->actual_position;
            motion_cycles = 0;
            if (!printed[3])
            {
               printed[3] = true;
               printf("State: SINE_MOTION, base=%d, sw=0x%04X\n",
                      sine_base_position, sw);
            }
            control_state_cycles = 0;
            control_state = CONTROL_SINE_MOTION;
         }
         break;

      case CONTROL_SINE_MOTION:
      {
         double phase = two_pi * (double)motion_cycles / (double)sine_period_cycles;
         double offset = (1.0 - cos(phase)) * 0.5 * (double)sine_range_counts;

         motion_cmd.target_position = sine_base_position + (int32_t)(offset + 0.5);
         motion_cmd.target_velocity = 0;
         motion_cmd.control_word = 0x000F;
         motion_cmd.target_torque = 0;
         motion_cmd.operation_mode = 8;

         if (!printed[4])
         {
            printed[4] = true;
            printf("Motion started\n");
         }

         motion_cycles = (motion_cycles + 1) % sine_period_cycles;
         break;
      }

      default:
         control_state = CONTROL_WAIT_STATUS;
         break;
   }
}

static void signal_handler(int sig)
{
   (void)sig;
   run = 0;
   dorun = 0;
   inOP = 0;
}

OSAL_THREAD_FUNC_RT ecatthread(void)
{
   ec_timet ts;
   int ht;
   int64_t toff = 0;

   dorun = 0;
   while (!mappingdone && run)
      osal_usleep(100);

   osal_get_monotonic_time(&ts);
   ht = (ts.tv_nsec / 1000000) + 1;
   ts.tv_nsec = ht * 1000000;
   if (ts.tv_nsec >= NSEC_PER_SEC)
   {
      ts.tv_sec++;
      ts.tv_nsec -= NSEC_PER_SEC;
   }

   ecx_send_processdata(&ctx);
   last_cycle_ts = monotonic_raw_ns();

   while (run)
   {
      add_time_ns(&ts, cycletime + toff);
      osal_monotonic_sleep(&ts);

      if (!dorun)
         continue;

      int64_t now_ns = monotonic_raw_ns();
      current_cycle_ns = now_ns - last_cycle_ts;
      last_cycle_ts = now_ns;
      total_cycles++;

      if (current_cycle_ns > max_cycle_ns) max_cycle_ns = current_cycle_ns;
      if (current_cycle_ns < min_cycle_ns) min_cycle_ns = current_cycle_ns;

      wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
      dowkccheck = (wkc == expectedWKC) ? 0 : (dowkccheck + 1);

      if (ctx.slavelist[1].hasdc && (wkc > 0))
      {
         ec_sync(ctx.DCtime, cycletime, &toff);
      }

      if (rx_pdo && tx_pdo && inOP)
      {
         runWork();

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

OSAL_THREAD_FUNC ecatcheck(void)
{
   int slaveix;

   while (run)
   {
      if (inOP && ((dowkccheck > 2) || ctx.grouplist[currentgroup].docheckstate))
      {
         ctx.grouplist[currentgroup].docheckstate = FALSE;
         ecx_readstate(&ctx);

         for (slaveix = 1; slaveix <= ctx.slavecount; slaveix++)
         {
            ec_slavet *slave = &ctx.slavelist[slaveix];

            if ((slave->group == currentgroup) &&
                (slave->state != EC_STATE_OPERATIONAL))
            {
               ctx.grouplist[currentgroup].docheckstate = TRUE;

               if (slave->state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
               {
                  slave->state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                  ecx_writestate(&ctx, slaveix);
               }
               else if (slave->state == EC_STATE_SAFE_OP)
               {
                  slave->state = EC_STATE_OPERATIONAL;
                  ecx_writestate(&ctx, slaveix);
               }
               else if (slave->state > EC_STATE_NONE)
               {
                  if (ecx_reconfig_slave(&ctx, slaveix, EC_TIMEOUTMON) >= EC_STATE_PRE_OP)
                     slave->islost = FALSE;
               }
               else if (!slave->islost)
               {
                  ecx_statecheck(&ctx, slaveix, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                  if (slave->state == EC_STATE_NONE)
                     slave->islost = TRUE;
               }
            }

            if (slave->islost)
            {
               if (slave->state <= EC_STATE_INIT)
               {
                  if (ecx_recover_slave(&ctx, slaveix, EC_TIMEOUTMON))
                     slave->islost = FALSE;
               }
               else
               {
                  slave->islost = FALSE;
               }
            }
         }

         dowkccheck = 0;
      }

      osal_usleep(10000);
   }
}

static bool request_state(uint16_t state, const char *name)
{
   ctx.slavelist[0].state = state;
   ecx_writestate(&ctx, 0);

   if (ecx_statecheck(&ctx, 0, state, EC_TIMEOUTSTATE) != state)
   {
      printf("Failed to enter %s\n", name);
      ecx_readstate(&ctx);

      for (int si = 1; si <= ctx.slavecount; si++)
      {
         printf("Slave %d state=0x%02X AL=0x%04X %s\n",
                si,
                ctx.slavelist[si].state,
                ctx.slavelist[si].ALstatuscode,
                ec_ALstatuscode2string(ctx.slavelist[si].ALstatuscode));
      }

      return false;
   }

   printf("%s OK\n", name);
   return true;
}

static bool ecatbringup(char *ifname)
{
   uint16_t slave = 1;
   ec_groupt *group = &ctx.grouplist[0];

   printf("EtherCAT startup on %s\n", ifname);

   if (!ecx_init(&ctx, ifname))
   {
      printf("ecx_init failed\n");
      return false;
   }

   ecx_config_init(&ctx);
   if (ctx.slavecount <= 0)
   {
      printf("No EtherCAT slave found\n");
      return false;
   }

   printf("Found %d slave(s), slave1=%s, vendor=0x%08X, product=0x%08X\n",
          ctx.slavecount,
          ctx.slavelist[slave].name,
          ctx.slavelist[slave].eep_man,
          ctx.slavelist[slave].eep_id);

   /*
    * Clean required order:
    *   ecx_configdc()
    *   ecx_dcsync0()
    *   configure_pdo()
    *   ecx_config_map_group()
    *   SAFE_OP
    *   OP
    */
   printf("Configure DC before PDO remap\n");
   ecx_configdc(&ctx);

   for (int i = 1; i <= ctx.slavecount; i++)
   {
      if (ctx.slavelist[i].hasdc)
      {
         ecx_dcsync0(&ctx, i, TRUE, cycletime, 0);
      }
   }


   if (!configure_pdo(slave))
      return false;

   ecx_config_map_group(&ctx, IOmap, 0);
   expectedWKC = (group->outputsWKC * 2) + group->inputsWKC;

   rx_pdo = (RxPDO_t *)ctx.slavelist[slave].outputs;
   tx_pdo = (TxPDO_t *)ctx.slavelist[slave].inputs;

   if (!rx_pdo || !tx_pdo)
   {
      printf("PDO pointers are NULL\n");
      return false;
   }

   printf("PDO: out=%d bytes, in=%d bytes, expectedWKC=%d\n",
          ctx.slavelist[slave].Obytes,
          ctx.slavelist[slave].Ibytes,
          expectedWKC);

   memset(&motion_cmd, 0, sizeof(motion_cmd));

   rx_pdo->target_position = 0;
   rx_pdo->target_velocity = 0;
   rx_pdo->control_word = 0x0000;
   rx_pdo->target_torque = 0;
   rx_pdo->operation_mode = 8;

   if (!request_state(EC_STATE_SAFE_OP, "SAFE_OP"))
      return false;

   printf("SAFE_OP PDO warmup\n");
   for (int i = 0; i < SAFEOP_WARMUP_CYCLES; i++)
   {
      ecx_send_processdata(&ctx);
      wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
      osal_usleep(cycletime / 1000);
   }

   if (tx_pdo)
   {
      rx_pdo->target_position = tx_pdo->actual_position;
      rx_pdo->target_velocity = 0;
      rx_pdo->control_word = 0x0000;
      rx_pdo->target_torque = 0;
      rx_pdo->operation_mode = 8;

      motion_cmd.target_position = tx_pdo->actual_position;
      motion_cmd.target_velocity = 0;
      motion_cmd.control_word = 0x0000;
      motion_cmd.target_torque = 0;
      motion_cmd.operation_mode = 8;
   }

   for (int i = 0; i < 50; i++)
   {
      ecx_send_processdata(&ctx);
      wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
      osal_usleep(cycletime / 1000);
   }

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

   mappingdone = 1;
   dorun = 1;

   /* Let RT thread exchange valid PDO frames before OP. */
   osal_usleep(300000);

   if (!request_state(EC_STATE_OPERATIONAL, "OPERATIONAL"))
      return false;

   inOP = 1;

   printf("OP OK, start CSP control\n");

   while (run)
   {
      if ((total_cycles % PRINT_PERIOD_CYCLES) == 0)
      {
         uint16_t sw = tx_pdo ? tx_pdo->status_word : 0;
         printf("cycle=%" PRId64 " wkc=%d/%d sw=0x%04X bit12=%d "
                "pos=%d target=%d dc_err=%.2fus max_dc=%.2fus\n",
                total_cycles,
                wkc,
                expectedWKC,
                sw,
                (sw & 0x1000) ? 1 : 0,
                tx_pdo ? tx_pdo->actual_position : 0,
                rx_pdo ? rx_pdo->target_position : 0,
                (double)timeerror / (double)US_PER_NSEC,
                (double)max_dc_error_ns / (double)US_PER_NSEC);
      }

      osal_usleep(100000);
   }

   return true;
}

static void shutdown_ethercat(void)
{
   dorun = 0;
   inOP = 0;
   osal_usleep(100000);

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
   if (argc < 2)
   {
      printf("Usage: %s ifname [cycle_us]\n", argv[0]);
      return 1;
   }

   if (argc > 2)
      cycletime = (int64_t)atoi(argv[2]) * 1000LL;

   printf("SOEM clean API CSP demo, cycle=%" PRId64 " ns\n", cycletime);

   if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
      printf("WARNING: mlockall failed\n");

   signal(SIGINT, signal_handler);
   signal(SIGTERM, signal_handler);

   run = 1;
   mappingdone = 0;
   dorun = 0;
   inOP = 0;
   memset(&motion_cmd, 0, sizeof(motion_cmd));

   osal_thread_create_rt(&threadrt, 128000, &ecatthread, NULL);
   osal_thread_create(&threadcheck, 128000, &ecatcheck, NULL);

   struct sched_param param;
   param.sched_priority = 99;
   (void)pthread_setschedparam((pthread_t)threadrt, SCHED_FIFO, &param);
   (void)pthread_setname_np((pthread_t)threadrt, "rt_eccomm");
   (void)pthread_setname_np((pthread_t)threadcheck, "ec_check");

   bool ok = ecatbringup(argv[1]);

   run = 0;
   osal_usleep(200000);
   shutdown_ethercat();

   printf("End program, result=%s\n", ok ? "OK" : "FAILED");
   return ok ? 0 : 1;
}
