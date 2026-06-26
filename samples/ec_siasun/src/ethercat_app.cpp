#include "ethercat_app.h"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sched.h>
#include <sys/mman.h>

#include "axis_config.h"
#include "sdo_parameter.h"

namespace siasun {
namespace {

constexpr int kSafeOpWarmupCycles = 300;
constexpr int kPostSdoDelayUs = 10000;
constexpr int kEthercatMonitorTimeoutUs = 500;
constexpr uint64_t kCycleOverrunLimitNs =
    static_cast<uint64_t>(kCycleTimeNs * 3 / 2);
constexpr double kPi = 3.14159265358979323846;

enum class ServoControlState {
    WaitStatus,
    Enable06,
    Enable07,
    Enable0F,
    SineMotion,
};

struct ServoMotionControl {
    ServoControlState state = ServoControlState::WaitStatus;
    uint64_t state_cycles = 0;
    uint64_t motion_cycles = 0;
    int32_t base_position = 0;
};

struct QualityStats {
    bool active = false;
    uint64_t cycles = 0;
    uint64_t interval_samples = 0;
    uint64_t cycle_time_sum_ns = 0;
    uint64_t jitter_abs_sum_ns = 0;
    uint64_t severe_overruns = 0;
    uint64_t good_wkc_cycles = 0;
    uint64_t bad_wkc_cycles = 0;
    uint64_t no_frame_cycles = 0;
    int min_wkc = INT_MAX;
    int max_wkc = INT_MIN;
    uint64_t dc_valid_samples = 0;
    uint64_t dc_abs_sum_ns = 0;
    int64_t dc_time_error_ns = 0;
    int64_t max_dc_diff_ns = 0;
    int64_t start_time_ns = 0;
    int64_t last_cycle_time_ns = 0;
    int64_t min_cycle_ns = INT64_MAX;
    int64_t max_cycle_ns = 0;
    int64_t max_abs_jitter_ns = 0;
    uint64_t state_error_events = 0;
    uint64_t reconfiguration_events = 0;
    uint64_t recovery_events = 0;
};

ServoMotionControl g_motion_control {};
QualityStats g_quality_stats {};
int64_t g_sync_offset_ns = 0;

int64_t timespec_to_ns(const ec_timet &ts) {
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

int64_t monotonic_raw_ns() {
    ec_timet now {};
    osal_get_monotonic_time(&now);
    return timespec_to_ns(now);
}

double ns_to_us(int64_t ns) {
    return static_cast<double>(ns) / 1000.0;
}

void add_time_ns(ec_timet *ts, int64_t addtime) {
    ec_timet addts {};
    addts.tv_nsec = addtime % 1000000000LL;
    addts.tv_sec = (addtime - addts.tv_nsec) / 1000000000LL;
    osal_timespecadd(ts, &addts, ts);
}

void prefault_stack() {
    volatile unsigned char dummy[kMaxSafeStack];
    for (std::size_t i = 0; i < sizeof(dummy); ++i) {
        dummy[i] = 0;
    }
}

void ec_sync(int64 reftime,
             int64 cycletime_ns,
             int64 *offsettime,
             QualityStats &stats) {
    static int64 integral = 0;
    constexpr double kProportionalGain = 0.01;
    constexpr double kIntegralGain = 0.00002;

    int64 delta = (reftime - g_sync_offset_ns) % cycletime_ns;
    if (delta > (cycletime_ns / 2)) {
        delta -= cycletime_ns;
    } else if (delta < -(cycletime_ns / 2)) {
        delta += cycletime_ns;
    }

    stats.dc_time_error_ns = -delta;
    const int64_t abs_error = std::llabs(stats.dc_time_error_ns);
    ++stats.dc_valid_samples;
    stats.dc_abs_sum_ns += static_cast<uint64_t>(abs_error);
    stats.max_dc_diff_ns = std::max(stats.max_dc_diff_ns, abs_error);

    integral += stats.dc_time_error_ns;
    *offsettime = static_cast<int64>(
        (static_cast<double>(stats.dc_time_error_ns) * kProportionalGain) +
        (static_cast<double>(integral) * kIntegralGain));
}

int sdo_write(ecx_contextt *context,
              uint16_t slave,
              uint16_t index,
              uint8_t subindex,
              const void *data,
              int size) {
    const int wkc = ecx_SDOwrite(context,
                                 slave,
                                 index,
                                 subindex,
                                 FALSE,
                                 size,
                                 data,
                                 EC_TIMEOUTRXM);
    if (wkc <= 0) {
        std::fprintf(stderr,
                     "SDO write failed: slave=%u 0x%04X:%02X wkc=%d\n",
                     slave,
                     index,
                     subindex,
                     wkc);
        return -1;
    }
    osal_usleep(kPostSdoDelayUs);
    return 0;
}

template <std::size_t N>
int configure_pdo_mapping(ecx_contextt *context,
                          uint16_t slave,
                          uint16_t pdo,
                          const std::array<PdoEntry, N> &entries) {
    uint8_t zero = 0;
    if (sdo_write(context, slave, pdo, 0x00, &zero, sizeof(zero))) {
        return -1;
    }

    for (std::size_t i = 0; i < entries.size(); ++i) {
        const PdoEntry &entry = entries[i];
        const uint32_t mapping =
            (static_cast<uint32_t>(entry.index) << 16) |
            (static_cast<uint32_t>(entry.subindex) << 8) |
            entry.bits;
        if (sdo_write(context,
                      slave,
                      pdo,
                      static_cast<uint8_t>(i + 1),
                      &mapping,
                      sizeof(mapping))) {
            return -1;
        }
    }

    const uint8_t count = static_cast<uint8_t>(entries.size());
    return sdo_write(context, slave, pdo, 0x00, &count, sizeof(count));
}

int assign_pdo(ecx_contextt *context,
               uint16_t slave,
               uint16_t assignment,
               uint16_t pdo_index) {
    uint8_t zero = 0;
    uint8_t one = 1;
    return sdo_write(context, slave, assignment, 0x00, &zero, sizeof(zero)) ||
           sdo_write(context, slave, assignment, 0x01, &pdo_index,
                     sizeof(pdo_index)) ||
           sdo_write(context, slave, assignment, 0x00, &one, sizeof(one));
}

int configure_servo_pdos(ecx_contextt *context, uint16_t slave) {
    std::printf("[PDO] configuring servo %u mapping\n", slave);
    if (ecx_statecheck(context, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE) !=
        EC_STATE_PRE_OP) {
        std::fprintf(stderr, "servo %u is not PRE-OP, state=0x%02X\n",
                     slave,
                     context->slavelist[slave].state);
        return -1;
    }

    if (configure_pdo_mapping(context, slave, 0x1600, kServoRxPdoEntries) ||
        assign_pdo(context, slave, 0x1C12, 0x1600) ||
        configure_pdo_mapping(context, slave, 0x1A00, kServoTxPdoEntries) ||
        assign_pdo(context, slave, 0x1C13, 0x1A00)) {
        return -1;
    }
    return 0;
}

int configure_endio_pdos(ecx_contextt *context, uint16_t slave) {
    std::printf("[PDO] configuring EndIO %u mapping\n", slave);
    if (ecx_statecheck(context, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE) !=
        EC_STATE_PRE_OP) {
        std::fprintf(stderr, "EndIO %u is not PRE-OP, state=0x%02X\n",
                     slave,
                     context->slavelist[slave].state);
        return -1;
    }

    if (configure_pdo_mapping(context, slave, 0x1600, kEndIoRxPdoEntries) ||
        assign_pdo(context, slave, 0x1C12, 0x1600) ||
        configure_pdo_mapping(context, slave, 0x1A00, kEndIoTxPdoEntries) ||
        assign_pdo(context, slave, 0x1C13, 0x1A00)) {
        return -1;
    }
    return 0;
}

bool request_state(ecx_contextt *context, uint16_t state, const char *name) {
    context->slavelist[0].state = state;
    ecx_writestate(context, 0);
    if (ecx_statecheck(context, 0, state, EC_TIMEOUTSTATE) == state) {
        std::printf("%s OK\n", name);
        return true;
    }

    std::fprintf(stderr, "failed to enter %s\n", name);
    ecx_readstate(context);
    for (int slave = 1; slave <= context->slavecount; ++slave) {
        std::fprintf(stderr, "slave %d state=0x%02X AL=0x%04X %s\n",
                     slave,
                     context->slavelist[slave].state,
                     context->slavelist[slave].ALstatuscode,
                     ec_ALstatuscode2string(
                         context->slavelist[slave].ALstatuscode));
    }
    return false;
}

void zero_servo_output(ServoRxPdo *rx) {
    if (rx) {
        std::memset(rx, 0, sizeof(*rx));
    }
}

void zero_endio_output(EndIoRxPdo *rx) {
    if (rx) {
        std::memset(rx, 0, sizeof(*rx));
    }
}

void write_default_outputs(App &app) {
    for (ServoRxPdo *rx : app.servo_rx) {
        zero_servo_output(rx);
    }
    zero_endio_output(app.endio_rx);
}

bool all_slaves_operational(App &app) {
    ecx_readstate(&app.context);
    if (app.context.slavecount < static_cast<int>(kExpectedSlaveCount)) {
        return false;
    }

    for (int slave = 1; slave <= static_cast<int>(kExpectedSlaveCount);
         ++slave) {
        if (app.context.slavelist[slave].state != EC_STATE_OPERATIONAL) {
            return false;
        }
    }
    return true;
}

void hold_motion_servo_position(App &app, uint16_t control_word) {
    ServoRxPdo *rx = app.servo_rx[kMotionServoIndex];
    const ServoTxPdo *tx = app.servo_tx[kMotionServoIndex];
    if (!rx || !tx) {
        return;
    }

    rx->target_position = tx->actual_position;
    rx->digital_outputs = 0;
    rx->target_velocity = 0;
    rx->control_word = control_word;
    rx->target_torque = 0;
    rx->operation_mode = kDriveOperationMode;
    rx->safe_control = 0;
    rx->target_safe_position = 0;
    rx->user_output = 0;
}

void set_servo_control_state(ServoMotionControl &control,
                             ServoControlState state) {
    control.state = state;
    control.state_cycles = 0;
}

void update_motion_servo_control(App &app, ServoMotionControl &control) {
    ServoRxPdo *rx = app.servo_rx[kMotionServoIndex];
    const ServoTxPdo *tx = app.servo_tx[kMotionServoIndex];
    if (!rx || !tx) {
        return;
    }

    const uint16_t status_word = tx->status_word;
    const int32_t actual_position = tx->actual_position;
    const bool op_ready =
        app.in_op &&
        app.context.slavelist[kMotionServoIndex + 1].state ==
            EC_STATE_OPERATIONAL;
    if (!op_ready) {
        zero_servo_output(rx);
        return;
    }

    if (status_word == 0) {
        hold_motion_servo_position(app, 0x0000);
        return;
    }

    switch (control.state) {
    case ServoControlState::WaitStatus:
        hold_motion_servo_position(app, 0x0006);
        std::printf("Servo 6 control: start 0x0006 status=0x%04X actual=%d\n",
                    status_word,
                    actual_position);
        set_servo_control_state(control, ServoControlState::Enable06);
        break;

    case ServoControlState::Enable06:
        hold_motion_servo_position(app, 0x0006);
        if (++control.state_cycles >= kEnableStepCycles) {
            set_servo_control_state(control, ServoControlState::Enable07);
        }
        break;

    case ServoControlState::Enable07:
        hold_motion_servo_position(app, 0x0007);
        if (++control.state_cycles >= kEnableStepCycles) {
            set_servo_control_state(control, ServoControlState::Enable0F);
        }
        break;

    case ServoControlState::Enable0F:
        hold_motion_servo_position(app, 0x000F);
        if (++control.state_cycles >= kEnableStepCycles) {
            control.base_position = actual_position;
            control.motion_cycles = 0;
            std::printf("Servo 6 control: sine motion base=%d\n",
                        control.base_position);
            set_servo_control_state(control, ServoControlState::SineMotion);
        }
        break;

    case ServoControlState::SineMotion: {
        const double phase =
            2.0 * kPi *
            static_cast<double>(control.motion_cycles % kMotionPeriodCycles) /
            static_cast<double>(kMotionPeriodCycles);
        const double ramp =
            std::min(1.0,
                     static_cast<double>(control.motion_cycles) /
                         static_cast<double>(kMotionRampCycles));
        const int32_t offset = static_cast<int32_t>(
            std::sin(phase) * static_cast<double>(kMotionAmplitudeCounts) *
            ramp);

        rx->target_position = control.base_position + offset;
        rx->digital_outputs = 0;
        rx->target_velocity = 0;
        rx->control_word = 0x000F;
        rx->target_torque = 0;
        rx->operation_mode = kDriveOperationMode;
        rx->safe_control = 0;
        rx->target_safe_position = 0;
        rx->user_output = 0;
        ++control.motion_cycles;
        break;
    }
    }
}

void write_cycle_outputs(App &app, ServoMotionControl &control) {
    for (std::size_t i = 0; i < kServoCount; ++i) {
        if (i != kMotionServoIndex) {
            zero_servo_output(app.servo_rx[i]);
        }
    }
    update_motion_servo_control(app, control);
    zero_endio_output(app.endio_rx);
}

void quality_update_cycle(QualityStats &stats, int64_t now_ns) {
    ++stats.cycles;
    if (stats.start_time_ns == 0) {
        stats.start_time_ns = now_ns;
        stats.last_cycle_time_ns = now_ns;
        return;
    }

    const int64_t interval_ns = now_ns - stats.last_cycle_time_ns;
    const int64_t jitter_ns =
        std::llabs(interval_ns - static_cast<int64_t>(kCycleTimeNs));
    stats.last_cycle_time_ns = now_ns;
    ++stats.interval_samples;
    stats.cycle_time_sum_ns += static_cast<uint64_t>(interval_ns);
    stats.jitter_abs_sum_ns += static_cast<uint64_t>(jitter_ns);
    stats.min_cycle_ns = std::min(stats.min_cycle_ns, interval_ns);
    stats.max_cycle_ns = std::max(stats.max_cycle_ns, interval_ns);
    stats.max_abs_jitter_ns = std::max(stats.max_abs_jitter_ns, jitter_ns);
    if (static_cast<uint64_t>(interval_ns) > kCycleOverrunLimitNs) {
        ++stats.severe_overruns;
    }
}

void quality_update_wkc(QualityStats &stats, int wkc, int expected_wkc) {
    if (wkc == expected_wkc) {
        ++stats.good_wkc_cycles;
    } else {
        ++stats.bad_wkc_cycles;
    }
    if (wkc <= 0) {
        ++stats.no_frame_cycles;
    }
    stats.min_wkc = std::min(stats.min_wkc, wkc);
    stats.max_wkc = std::max(stats.max_wkc, wkc);
}

bool quality_update_after_op(App &app,
                             QualityStats &stats,
                             int64_t now_ns,
                             int wkc) {
    if (!stats.active) {
        if (!all_slaves_operational(app)) {
            return false;
        }
        stats.active = true;
        std::printf("communication quality statistics started: all 7 "
                    "slaves are OP\n");
    }

    quality_update_cycle(stats, now_ns);
    quality_update_wkc(stats, wkc, app.expected_wkc);
    return true;
}

void print_quality_report(App &app,
                          const QualityStats &stats,
                          bool final_report) {
    const char *title = final_report ? "Ctrl+C final" : "periodic";
    const double avg_period_us =
        stats.interval_samples > 0
            ? ns_to_us(static_cast<int64_t>(stats.cycle_time_sum_ns /
                                            stats.interval_samples))
            : 0.0;
    const double avg_jitter_us =
        stats.interval_samples > 0
            ? ns_to_us(static_cast<int64_t>(stats.jitter_abs_sum_ns /
                                            stats.interval_samples))
            : 0.0;
    const double success_rate =
        stats.cycles > 0
            ? static_cast<double>(stats.good_wkc_cycles) * 100.0 /
                  static_cast<double>(stats.cycles)
            : 0.0;
    const double dc_avg_abs_us =
        stats.dc_valid_samples > 0
            ? ns_to_us(static_cast<int64_t>(stats.dc_abs_sum_ns /
                                            stats.dc_valid_samples))
            : 0.0;

    std::printf("\n============================================================\n");
    std::printf("              SOEM communication quality report: %s\n", title);
    std::printf("============================================================\n");
    if (!stats.active) {
        std::printf("all slaves did not reach OP; no runtime samples\n");
        std::printf("============================================================\n");
        return;
    }

    std::printf("  cycles               : %12llu\n",
                static_cast<unsigned long long>(stats.cycles));
    std::printf("  nominal period       : %12.3f us\n",
                ns_to_us(kCycleTimeNs));
    std::printf("  avg period           : %12.3f us\n", avg_period_us);
    std::printf("  min / max period     : %12.3f / %.3f us\n",
                stats.interval_samples ? ns_to_us(stats.min_cycle_ns) : 0.0,
                stats.interval_samples ? ns_to_us(stats.max_cycle_ns) : 0.0);
    std::printf("  avg abs jitter       : %12.3f us\n", avg_jitter_us);
    std::printf("  max abs jitter       : %12.3f us\n",
                ns_to_us(stats.max_abs_jitter_ns));
    std::printf("  severe overruns      : %12llu\n",
                static_cast<unsigned long long>(stats.severe_overruns));
    std::printf("  expected WKC         : %12d\n", app.expected_wkc);
    std::printf("  good / bad cycles    : %12llu / %llu\n",
                static_cast<unsigned long long>(stats.good_wkc_cycles),
                static_cast<unsigned long long>(stats.bad_wkc_cycles));
    std::printf("  no frame cycles      : %12llu\n",
                static_cast<unsigned long long>(stats.no_frame_cycles));
    std::printf("  success rate         : %12.6f %%\n", success_rate);
    std::printf("  DC current / avg / max: %10.3f / %.3f / %.3f us\n",
                ns_to_us(stats.dc_time_error_ns),
                dc_avg_abs_us,
                ns_to_us(stats.max_dc_diff_ns));
    std::printf("  recovery events      : state=%llu reconfig=%llu recover=%llu\n",
                static_cast<unsigned long long>(stats.state_error_events),
                static_cast<unsigned long long>(stats.reconfiguration_events),
                static_cast<unsigned long long>(stats.recovery_events));

    for (std::size_t i = 0; i < kServoCount; ++i) {
        const ServoRxPdo *rx = app.servo_rx[i];
        const ServoTxPdo *tx = app.servo_tx[i];
        std::printf("  Servo %zu status/mode/pos/vel/err: "
                    "0x%04X / %d / %d / %d / 0x%08X\n",
                    i + 1,
                    tx ? tx->status_word : 0,
                    tx ? tx->operation_mode_display : 0,
                    tx ? tx->actual_position : 0,
                    tx ? tx->actual_velocity : 0,
                    tx ? tx->error_code : 0);
        std::printf("          target ctrl/mode/pos    : "
                    "0x%04X / %d / %d\n",
                    rx ? rx->control_word : 0,
                    rx ? rx->operation_mode : 0,
                    rx ? rx->target_position : 0);
    }
    std::printf("============================================================\n");
}

void validate_slave_identity(App &app) {
    for (std::size_t i = 0; i < kServoCount; ++i) {
        const ec_slavet &slave = app.context.slavelist[i + 1];
        if (slave.eep_man != kVendorId || slave.eep_id != kServoProductCode) {
            std::fprintf(stderr,
                         "warning: slave %zu expected servo vendor/product "
                         "0x%08X/0x%08X, got 0x%08X/0x%08X\n",
                         i + 1,
                         kVendorId,
                         kServoProductCode,
                         slave.eep_man,
                         slave.eep_id);
        }
    }

    const ec_slavet &endio = app.context.slavelist[kEndIoLogicalId];
    if (endio.eep_man != kVendorId || endio.eep_id != kEndIoProductCode) {
        std::fprintf(stderr,
                     "warning: EndIO expected vendor/product 0x%08X/0x%08X, "
                     "got 0x%08X/0x%08X\n",
                     kVendorId,
                     kEndIoProductCode,
                     endio.eep_man,
                     endio.eep_id);
    }
}

int assign_pdo_pointers(App &app) {
    for (std::size_t i = 0; i < kServoCount; ++i) {
        const int slave = static_cast<int>(i + 1);
        app.servo_rx[i] =
            reinterpret_cast<ServoRxPdo *>(app.context.slavelist[slave].outputs);
        app.servo_tx[i] =
            reinterpret_cast<ServoTxPdo *>(app.context.slavelist[slave].inputs);
        if (!app.servo_rx[i] || !app.servo_tx[i]) {
            std::fprintf(stderr, "servo %zu PDO pointers are NULL\n", i + 1);
            return -1;
        }
        std::printf("[PDO] Servo %zu out=%d bytes in=%d bytes\n",
                    i + 1,
                    app.context.slavelist[slave].Obytes,
                    app.context.slavelist[slave].Ibytes);
    }

    app.endio_rx = reinterpret_cast<EndIoRxPdo *>(
        app.context.slavelist[kEndIoLogicalId].outputs);
    app.endio_tx = reinterpret_cast<EndIoTxPdo *>(
        app.context.slavelist[kEndIoLogicalId].inputs);
    if (!app.endio_rx || !app.endio_tx) {
        std::fprintf(stderr, "EndIO PDO pointers are NULL\n");
        return -1;
    }
    std::printf("[PDO] EndIO out=%d bytes in=%d bytes\n",
                app.context.slavelist[kEndIoLogicalId].Obytes,
                app.context.slavelist[kEndIoLogicalId].Ibytes);
    return 0;
}

OSAL_THREAD_FUNC_RT ecatthread(void *arg) {
    App *app = static_cast<App *>(arg);
    ec_timet wakeup {};
    int64_t time_offset_ns = 0;
    uint64_t cycle = 0;

    while (!app->mapping_done && app->run) {
        osal_usleep(100);
    }

    osal_get_monotonic_time(&wakeup);
    const long next_ms = (wakeup.tv_nsec / 1000000L) + 1L;
    wakeup.tv_nsec = next_ms * 1000000L;
    if (wakeup.tv_nsec >= 1000000000L) {
        ++wakeup.tv_sec;
        wakeup.tv_nsec -= 1000000000L;
    }

    ecx_send_processdata(&app->context);
    g_quality_stats.last_cycle_time_ns = monotonic_raw_ns();
    g_quality_stats.start_time_ns = g_quality_stats.last_cycle_time_ns;

    while (app->run) {
        add_time_ns(&wakeup, kCycleTimeNs + time_offset_ns);
        osal_monotonic_sleep(&wakeup);
        if (!app->do_run) {
            continue;
        }

        app->last_wkc = ecx_receive_processdata(&app->context, EC_TIMEOUTRET);
        app->wkc_check_misses =
            (app->last_wkc == app->expected_wkc) ? 0
                                                 : (app->wkc_check_misses + 1);
        if (app->context.slavelist[1].hasdc && app->last_wkc > 0) {
            ec_sync(app->context.DCtime, kCycleTimeNs, &time_offset_ns,
                    g_quality_stats);
        }

        quality_update_after_op(*app, g_quality_stats, monotonic_raw_ns(),
                                app->last_wkc);
        if (app->in_op) {
            write_cycle_outputs(*app, g_motion_control);
        } else {
            write_default_outputs(*app);
        }
        if (cycle > 0 && g_quality_stats.active &&
            g_quality_stats.cycles % kQualityReportPeriodCycles == 0) {
            print_quality_report(*app, g_quality_stats, false);
        }

        ecx_mbxhandler(&app->context, 0, 4);
        ecx_send_processdata(&app->context);
        ++cycle;
    }
}

OSAL_THREAD_FUNC ecatcheck(void *arg) {
    App *app = static_cast<App *>(arg);

    while (app->run) {
        if (app->in_op &&
            (app->wkc_check_misses > 2 ||
             app->context.grouplist[app->current_group].docheckstate)) {
            ++g_quality_stats.state_error_events;
            app->context.grouplist[app->current_group].docheckstate = FALSE;
            ecx_readstate(&app->context);

            for (int slave_index = 1; slave_index <= app->context.slavecount;
                 ++slave_index) {
                ec_slavet *slave = &app->context.slavelist[slave_index];
                if (slave->group == app->current_group &&
                    slave->state != EC_STATE_OPERATIONAL) {
                    app->context.grouplist[app->current_group].docheckstate =
                        TRUE;
                    if (slave->state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        slave->state = EC_STATE_SAFE_OP + EC_STATE_ACK;
                        ecx_writestate(&app->context, slave_index);
                    } else if (slave->state == EC_STATE_SAFE_OP) {
                        slave->state = EC_STATE_OPERATIONAL;
                        ecx_writestate(&app->context, slave_index);
                    } else if (slave->state > EC_STATE_NONE) {
                        if (ecx_reconfig_slave(&app->context,
                                               slave_index,
                                               kEthercatMonitorTimeoutUs) >=
                            EC_STATE_PRE_OP) {
                            slave->islost = FALSE;
                            ++g_quality_stats.reconfiguration_events;
                        }
                    } else if (!slave->islost) {
                        ecx_statecheck(&app->context,
                                       slave_index,
                                       EC_STATE_OPERATIONAL,
                                       EC_TIMEOUTRET);
                        if (slave->state == EC_STATE_NONE) {
                            slave->islost = TRUE;
                        }
                    }
                }

                if (slave->islost) {
                    if (slave->state <= EC_STATE_INIT) {
                        if (ecx_recover_slave(&app->context,
                                              slave_index,
                                              kEthercatMonitorTimeoutUs)) {
                            slave->islost = FALSE;
                            ++g_quality_stats.recovery_events;
                        }
                    } else {
                        slave->islost = FALSE;
                    }
                }
            }
            app->wkc_check_misses = 0;
        }
        osal_usleep(10000);
    }
}

}  // namespace

void setup_realtime_process() {
    sched_param param {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        std::fprintf(stderr, "warning: sched_setscheduler failed: %s\n",
                     std::strerror(errno));
    }
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::fprintf(stderr, "warning: mlockall failed: %s\n",
                     std::strerror(errno));
    }
    prefault_stack();
}

int configure(App &app,
              const char *ifname,
              const std::string &axis_config_directory) {
    app.run = 1;
    app.mapping_done = 0;
    app.do_run = 0;
    app.in_op = false;
    app.wkc_check_misses = 0;
    g_motion_control = ServoMotionControl {};
    g_quality_stats = QualityStats {};
    g_sync_offset_ns = 0;

    std::printf("SOEM startup on %s\n", ifname);
    if (!ecx_init(&app.context, ifname)) {
        std::fprintf(stderr, "ecx_init failed on %s\n", ifname);
        return -1;
    }
    if (ecx_config_init(&app.context) <= 0) {
        std::fprintf(stderr, "no EtherCAT slave found\n");
        return -1;
    }
    std::printf("found %d slave(s)\n", app.context.slavecount);
    if (app.context.slavecount < static_cast<int>(kExpectedSlaveCount)) {
        std::fprintf(stderr, "expected at least %zu slaves, found %d\n",
                     kExpectedSlaveCount,
                     app.context.slavecount);
        return -1;
    }

    validate_slave_identity(app);

    AxisParameterSet axis_parameters;
    if (load_axis_parameter_set(axis_config_directory, axis_parameters) ||
        write_axis_parameters(&app.context, axis_parameters)) {
        return -1;
    }

    ecx_configdc(&app.context);
    for (int slave = 1; slave <= app.context.slavecount; ++slave) {
        if (app.context.slavelist[slave].hasdc) {
            ecx_dcsync0(&app.context, slave, TRUE, kCycleTimeNs, 0);
        }
    }

    for (std::size_t i = 0; i < kServoCount; ++i) {
        if (configure_servo_pdos(&app.context, static_cast<uint16_t>(i + 1))) {
            return -1;
        }
    }
    if (configure_endio_pdos(&app.context, kEndIoLogicalId)) {
        return -1;
    }

    std::fill(app.io_map.begin(), app.io_map.end(), 0);
    ecx_config_map_group(&app.context, app.io_map.data(), 0);
    app.expected_wkc =
        (app.context.grouplist[0].outputsWKC * 2) +
        app.context.grouplist[0].inputsWKC;
    std::printf("[PDO] expectedWKC=%d outputsWKC=%d inputsWKC=%d\n",
                app.expected_wkc,
                app.context.grouplist[0].outputsWKC,
                app.context.grouplist[0].inputsWKC);

    if (assign_pdo_pointers(app)) {
        return -1;
    }

    if (!osal_thread_create_rt(&app.rt_thread,
                               128000,
                               reinterpret_cast<void *>(ecatthread),
                               &app)) {
        std::fprintf(stderr, "failed to create SOEM RT thread\n");
        return -1;
    }
    if (!osal_thread_create(&app.check_thread,
                            128000,
                            reinterpret_cast<void *>(ecatcheck),
                            &app)) {
        std::fprintf(stderr, "failed to create SOEM check thread\n");
        return -1;
    }

    write_default_outputs(app);
    if (!request_state(&app.context, EC_STATE_SAFE_OP, "SAFE_OP")) {
        return -1;
    }

    std::printf("SAFE_OP PDO warmup\n");
    for (int i = 0; i < kSafeOpWarmupCycles; ++i) {
        ecx_send_processdata(&app.context);
        app.last_wkc = ecx_receive_processdata(&app.context, EC_TIMEOUTRET);
        osal_usleep(kCycleTimeNs / 1000);
    }
    write_default_outputs(app);
    for (int i = 0; i < 50; ++i) {
        ecx_send_processdata(&app.context);
        app.last_wkc = ecx_receive_processdata(&app.context, EC_TIMEOUTRET);
        osal_usleep(kCycleTimeNs / 1000);
    }

    for (int slave = 1; slave <= app.context.slavecount; ++slave) {
        if (app.context.slavelist[slave].CoEdetails > 0) {
            ecx_slavembxcyclic(&app.context, slave);
        }
    }

    app.mapping_done = 1;
    app.do_run = 1;
    osal_usleep(300000);

    if (!request_state(&app.context, EC_STATE_OPERATIONAL, "OPERATIONAL")) {
        return -1;
    }

    app.in_op = true;
    std::printf("OP OK, start 1 ms cyclic communication\n");
    return 0;
}

void run(App &app, volatile std::sig_atomic_t &keep_running) {
    std::printf("topology: servo id 1-6, EndIO id 7, DC assign=0x%04X\n",
                kDcAssignActivate);
    std::printf("motion: only Servo 6 enabled, amplitude=%d pulse, "
                "peak_to_peak=%d pulse, period=%llu ms ramp=%llu ms\n",
                kMotionAmplitudeCounts,
                kMotionRangeCounts,
                static_cast<unsigned long long>(kMotionPeriodCycles),
                static_cast<unsigned long long>(kMotionRampCycles));

    while (keep_running) {
        osal_usleep(100000);
    }

    app.do_run = 0;
    app.in_op = false;
    app.run = 0;
    osal_usleep(200000);
    print_quality_report(app, g_quality_stats, true);
}

void release(App &app) {
    app.run = 0;
    app.do_run = 0;
    app.in_op = false;
    write_default_outputs(app);
    ecx_send_processdata(&app.context);
    osal_usleep(100000);

    if (app.context.slavecount > 0) {
        std::printf("EtherCAT to SAFE_OP\n");
        app.context.slavelist[0].state = EC_STATE_SAFE_OP;
        ecx_writestate(&app.context, 0);
        ecx_statecheck(&app.context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

        std::printf("EtherCAT to INIT\n");
        app.context.slavelist[0].state = EC_STATE_INIT;
        ecx_writestate(&app.context, 0);
        ecx_statecheck(&app.context, 0, EC_STATE_INIT, EC_TIMEOUTSTATE);
    }

    ecx_close(&app.context);
}

}  // namespace siasun
