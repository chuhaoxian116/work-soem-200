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

/* SAFE_OP 阶段预热周期数；进入 OP 前先交换一段有效 PDO。*/
constexpr int kSafeOpWarmupCycles = 300;
/* 每次 SDO 写入后的等待时间，给从站 mailbox 处理留出余量。*/
constexpr int kPostSdoDelayUs = 10000;
/* SOEM check 线程中重配置/恢复从站使用的超时时间，单位 us。*/
constexpr int kEthercatMonitorTimeoutUs = 500;
/* 严重超周期阈值；超过 150% 标称周期计入 severe_overruns。*/
constexpr uint64_t kCycleOverrunLimitNs =
    static_cast<uint64_t>(kCycleTimeNs * 3 / 2);
constexpr double kPi = 3.14159265358979323846;
/* SIASUN ESI 中的 mailbox SM0/SM1 配置；IgH 能正常下参时使用的也是这组 mailbox。*/
constexpr uint16_t kMailboxOutStart = 0x1000;
constexpr uint16_t kMailboxInStart = 0x1080;
constexpr uint16_t kMailboxSize = 0x0080;
constexpr uint32_t kMailboxOutFlags = 0x00010026;
constexpr uint32_t kMailboxInFlags = 0x00010022;

/* 第 6 轴 CiA402 使能和运动状态机。*/
enum class ServoControlState {
    WaitStatus,
    Enable06,
    Enable07,
    Enable0F,
    SineMotion,
};

/* 第 6 轴运动控制状态，跨 1 ms 周期保存。*/
struct ServoMotionControl {
    /* 当前 CiA402/运动状态。*/
    ServoControlState state = ServoControlState::WaitStatus;
    /* 当前状态已经保持的通信周期数。*/
    uint64_t state_cycles = 0;
    /* 进入正弦运动后的累计周期数。*/
    uint64_t motion_cycles = 0;
    /* 进入正弦运动时锁存的实际位置，作为轨迹中心。*/
    int32_t base_position = 0;
};

/* 通信质量统计，和 IgH 版报告口径保持相近。*/
struct QualityStats {
    /* 所有从站进入 OP 后才开始统计，避免启动阶段污染运行数据。*/
    bool active = false;
    /* 已统计的运行周期总数。*/
    uint64_t cycles = 0;
    /* 周期时间样本数；第一个周期只用于初始化 last_cycle_time_ns。*/
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

/* g_motion_control：RT 线程拥有的第 6 轴控制状态。*/
ServoMotionControl g_motion_control {};
/* g_quality_stats：RT/check 线程共同更新的通信质量统计。*/
QualityStats g_quality_stats {};
/* g_sync_offset_ns：SOEM DC PI 同步算法使用的相位基准。*/
int64_t g_sync_offset_ns = 0;

/*
 * 将 SOEM ec_timet 转换为纳秒。
 *
 * ts：待转换的单调时间。
 */
int64_t timespec_to_ns(const ec_timet &ts) {
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

/* 读取 SOEM OSAL 单调时间并转换为纳秒，用于周期质量统计。*/
int64_t monotonic_raw_ns() {
    ec_timet now {};
    osal_get_monotonic_time(&now);
    return timespec_to_ns(now);
}

/* 纳秒转换为微秒，供质量报告打印使用。*/
double ns_to_us(int64_t ns) {
    return static_cast<double>(ns) / 1000.0;
}

/*
 * 在 ec_timet 上累加纳秒，并修正 tv_nsec 溢出。
 *
 * ts：周期线程下一次唤醒的绝对时间。
 * addtime：需要累加的纳秒数，包含 DC PI 修正量。
 */
void add_time_ns(ec_timet *ts, int64_t addtime) {
    ec_timet addts {};
    addts.tv_nsec = addtime % 1000000000LL;
    addts.tv_sec = (addtime - addts.tv_nsec) / 1000000000LL;
    osal_timespecadd(ts, &addts, ts);
}

/* 预触碰栈内存，降低进入实时循环后首次访问栈页导致的抖动。*/
void prefault_stack() {
    /* dummy：主动访问的一段栈空间。*/
    volatile unsigned char dummy[kMaxSafeStack];
    for (std::size_t i = 0; i < sizeof(dummy); ++i) {
        dummy[i] = 0;
    }
}

/*
 * SOEM 示例中的 DC PI 同步算法。
 *
 * reftime：SOEM 从 DC 报文得到的参考时间 ctx.DCtime。
 * cycletime_ns：通信周期，当前为 1 ms。
 * offsettime：输出给周期线程的下一周期唤醒修正量。
 * stats：同步误差统计对象。
 */
void ec_sync(int64 reftime,
             int64 cycletime_ns,
             int64 *offsettime,
             QualityStats &stats) {
    /* integral：PI 控制器积分项，跨周期累计 DC 时间误差。*/
    static int64 integral = 0;
    constexpr double kProportionalGain = 0.01;
    constexpr double kIntegralGain = 0.00002;

    /* delta：当前 DC 参考时间相对周期边界的相位误差。*/
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

/*
 * SOEM SDO 写辅助函数。
 *
 * context：SOEM 主站上下文。
 * slave：SOEM 1-based 从站编号。
 * index/subindex：对象字典地址。
 * data/size：待写入 payload。
 */
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

/*
 * 配置单个 PDO 映射对象 0x1600 或 0x1A00。
 *
 * SOEM 没有 IgH 的 ecrt_slave_config_pdos() 抽象，这里直接按 CoE 规则：
 * 先清 sub0，再写每个映射项，最后恢复 sub0 数量。
 */
template <std::size_t N>
int configure_pdo_mapping(ecx_contextt *context,
                          uint16_t slave,
                          uint16_t pdo,
                          const std::array<PdoEntry, N> &entries) {
    /* zero：写入 sub0 清空映射表，符合 CoE PDO remap 标准流程。*/
    uint8_t zero = 0;
    if (sdo_write(context, slave, pdo, 0x00, &zero, sizeof(zero))) {
        return -1;
    }

    for (std::size_t i = 0; i < entries.size(); ++i) {
        /* entry：当前要写入 0x1600/0x1A00 的 PDO entry 描述。*/
        const PdoEntry &entry = entries[i];
        /* mapping：index/subindex/bitlen 打包后的 32-bit 映射字。*/
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

/*
 * 将 PDO 对象挂到 SyncManager assignment。
 *
 * assignment：0x1C12 表示 RxPDO(SM2)，0x1C13 表示 TxPDO(SM3)。
 */
int assign_pdo(ecx_contextt *context,
               uint16_t slave,
               uint16_t assignment,
               uint16_t pdo_index) {
    /* zero/one：分别用于关闭和重新打开 SyncManager assignment。*/
    uint8_t zero = 0;
    uint8_t one = 1;
    return sdo_write(context, slave, assignment, 0x00, &zero, sizeof(zero)) ||
           sdo_write(context, slave, assignment, 0x01, &pdo_index,
                     sizeof(pdo_index)) ||
           sdo_write(context, slave, assignment, 0x00, &one, sizeof(one));
}

/*
 * 配置一个伺服从站的 RxPDO/TxPDO 映射。
 *
 * slave：SOEM 1-based 伺服从站编号，1-6 对应用户侧 Servo 1-6。
 */
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

/*
 * 配置末端 IO 从站的 RxPDO/TxPDO 映射。
 *
 * slave：SOEM 1-based 从站编号，当前为 7。
 */
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

/*
 * 请求整条 EtherCAT 总线切换状态。
 *
 * state：目标 AL 状态，例如 SAFE_OP 或 OP。
 * name：目标状态名称，仅用于日志。
 */
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

/* 清零单个伺服 RxPDO 输出区。*/
void zero_servo_output(ServoRxPdo *rx) {
    if (rx) {
        std::memset(rx, 0, sizeof(*rx));
    }
}

/* 清零末端 IO RxPDO 输出区。*/
void zero_endio_output(EndIoRxPdo *rx) {
    if (rx) {
        std::memset(rx, 0, sizeof(*rx));
    }
}

/* 清零全部业务输出；启动、等待 OP 和退出时都使用该默认输出。*/
void write_default_outputs(App &app) {
    for (ServoRxPdo *rx : app.servo_rx) {
        zero_servo_output(rx);
    }
    zero_endio_output(app.endio_rx);
}

/*
 * 第 6 轴保持当前位置。
 *
 * control_word：本周期要写入 0x6040 的控制字。
 */
void hold_motion_servo_position(App &app, uint16_t control_word) {
    /* rx：第 6 轴主站输出 PDO。*/
    ServoRxPdo *rx = app.servo_rx[kMotionServoIndex];
    /* tx：第 6 轴伺服反馈 PDO。*/
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

/* 切换第 6 轴控制状态，并清零该状态的保持计数。*/
void set_servo_control_state(ServoMotionControl &control,
                             ServoControlState state) {
    control.state = state;
    control.state_cycles = 0;
}

/*
 * 更新第 6 轴控制 PDO。
 *
 * 只在所有从站 OP 后使能第 6 轴；其它轴在 write_cycle_outputs() 中保持 0。
 */
void update_motion_servo_control(App &app, ServoMotionControl &control) {
    /* rx/tx：第 6 轴 PDO 指针；只有两者均有效才允许写控制量。*/
    ServoRxPdo *rx = app.servo_rx[kMotionServoIndex];
    const ServoTxPdo *tx = app.servo_tx[kMotionServoIndex];
    if (!rx || !tx) {
        return;
    }

    /* status_word：第 6 轴 CiA402 状态字。*/
    const uint16_t status_word = tx->status_word;
    /* actual_position：第 6 轴当前实际位置，用于保持和轨迹基准。*/
    const int32_t actual_position = tx->actual_position;
    /* op_ready：应用层和 SOEM AL 状态均确认 OP 后才开始使能。*/
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
        /* phase：当前正弦轨迹相位。*/
        const double phase =
            2.0 * kPi *
            static_cast<double>(control.motion_cycles % kMotionPeriodCycles) /
            static_cast<double>(kMotionPeriodCycles);
        /* ramp：启动包络，避免刚进入运动时目标位置突变。*/
        const double ramp =
            std::min(1.0,
                     static_cast<double>(control.motion_cycles) /
                         static_cast<double>(kMotionRampCycles));
        /* offset：相对 base_position 的本周期目标位置偏移。*/
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

/* 写入本周期全部输出 PDO：前 5 轴清零，第 6 轴执行控制状态机，EndIO 清零。*/
void write_cycle_outputs(App &app, ServoMotionControl &control) {
    for (std::size_t i = 0; i < kServoCount; ++i) {
        if (i != kMotionServoIndex) {
            zero_servo_output(app.servo_rx[i]);
        }
    }
    update_motion_servo_control(app, control);
    zero_endio_output(app.endio_rx);
}

/*
 * 更新周期时间质量统计。
 *
 * now_ns：当前单调时间，单位 ns。
 */
void quality_update_cycle(QualityStats &stats, int64_t now_ns) {
    ++stats.cycles;
    if (stats.start_time_ns == 0) {
        stats.start_time_ns = now_ns;
        stats.last_cycle_time_ns = now_ns;
        return;
    }

    /* interval_ns：本周期和上一周期之间的真实调度间隔。*/
    const int64_t interval_ns = now_ns - stats.last_cycle_time_ns;
    /* jitter_ns：相对标称 1 ms 的绝对抖动。*/
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

/*
 * 更新 WKC 质量统计。
 *
 * wkc：本周期收到的 Working Counter。
 * expected_wkc：PDO 映射后计算出的期望 WKC。
 */
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

/*
 * OP 后更新通信质量统计。
 *
 * 启动阶段 INIT/PREOP/SAFEOP 的 PDO 交换不计入最终运行质量。
 */
bool quality_update_after_op(App &app,
                             QualityStats &stats,
                             int64_t now_ns,
                             int wkc) {
    if (!stats.active) {
        if (!app.in_op) {
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

/*
 * 打印通信质量报告。
 *
 * final_report：true 表示 Ctrl+C 退出时的最终报告。
 */
void print_quality_report(App &app,
                          const QualityStats &stats,
                          bool final_report) {
    /* title：报告标题，区分周期报告和退出最终报告。*/
    const char *title = final_report ? "Ctrl+C final" : "periodic";
    /* avg_period_us/avg_jitter_us：运行期平均周期和平均绝对抖动。*/
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
    /* success_rate：WKC 完整周期占比。*/
    const double success_rate =
        stats.cycles > 0
            ? static_cast<double>(stats.good_wkc_cycles) * 100.0 /
                  static_cast<double>(stats.cycles)
            : 0.0;
    /* dc_avg_abs_us：DC PI 同步误差的平均绝对值。*/
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

/*
 * 校验从站 Vendor/ProductCode。
 *
 * 只打印 warning，不中断运行，方便现场在设备信息不一致时继续排查。
 */
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

void print_mailbox_config(const App &app, const char *stage) {
    std::printf("[MBX] %s\n", stage);
    for (int slave = 1; slave <= app.context.slavecount; ++slave) {
        const ec_slavet &item = app.context.slavelist[slave];
        std::printf("[MBX] slave=%d product=0x%08X state=0x%02X "
                    "wo=0x%04X wl=%u ro=0x%04X rl=%u proto=0x%04X "
                    "coe=0x%02X sm0=0x%04X/%u sm1=0x%04X/%u\n",
                    slave,
                    item.eep_id,
                    item.state,
                    item.mbx_wo,
                    item.mbx_l,
                    item.mbx_ro,
                    item.mbx_rl,
                    item.mbx_proto,
                    item.CoEdetails,
                    etohs(item.SM[0].StartAddr),
                    etohs(item.SM[0].SMlength),
                    etohs(item.SM[1].StartAddr),
                    etohs(item.SM[1].SMlength));
    }
}

bool is_siasun_mailbox_slave(const ec_slavet &slave) {
    return slave.eep_man == kVendorId &&
           (slave.eep_id == kServoProductCode ||
            slave.eep_id == kEndIoProductCode);
}

int configure_siasun_mailboxes(App &app) {
    print_mailbox_config(app, "after ecx_config_init");
    if (!request_state(&app.context, EC_STATE_INIT, "INIT for mailbox setup")) {
        return -1;
    }

    for (int slave = 1; slave <= app.context.slavecount; ++slave) {
        ec_slavet &item = app.context.slavelist[slave];
        if (!is_siasun_mailbox_slave(item)) {
            continue;
        }

        item.mbx_wo = kMailboxOutStart;
        item.mbx_l = kMailboxSize;
        item.mbx_ro = kMailboxInStart;
        item.mbx_rl = kMailboxSize;
        item.mbx_proto |= ECT_MBXPROT_COE;
        item.CoEdetails |= ECT_COEDET_SDO | ECT_COEDET_PDOASSIGN |
                            ECT_COEDET_PDOCONFIG;
        item.SMtype[0] = 1;
        item.SMtype[1] = 2;
        item.SM[0].StartAddr = htoes(kMailboxOutStart);
        item.SM[0].SMlength = htoes(kMailboxSize);
        item.SM[0].SMflags = htoel(kMailboxOutFlags);
        item.SM[1].StartAddr = htoes(kMailboxInStart);
        item.SM[1].SMlength = htoes(kMailboxSize);
        item.SM[1].SMflags = htoel(kMailboxInFlags);

        const int wkc = ecx_FPWR(&app.context.port,
                                 item.configadr,
                                 ECT_REG_SM0,
                                 sizeof(ec_smt) * 2,
                                 &item.SM[0],
                                 EC_TIMEOUTRET3);
        std::printf("[MBX] slave=%d force mailbox SM0/SM1 wkc=%d\n",
                    slave,
                    wkc);
        if (wkc <= 0) {
            std::fprintf(stderr,
                         "failed to configure mailbox SM for slave %d\n",
                         slave);
            return -1;
        }
    }

    print_mailbox_config(app, "after SIASUN mailbox setup");
    return request_state(&app.context, EC_STATE_PRE_OP, "PRE_OP") ? 0 : -1;
}

/*
 * 从 SOEM slavelist 中取出每个从站的 PDO 输入输出指针。
 *
 * 必须在 ecx_config_map_group() 之后调用，此时 outputs/inputs 才指向 IOmap。
 */
int assign_pdo_pointers(App &app) {
    for (std::size_t i = 0; i < kServoCount; ++i) {
        /* slave：SOEM 1-based 从站编号。*/
        const int slave = static_cast<int>(i + 1);
        const int obytes = app.context.slavelist[slave].Obytes;
        const int ibytes = app.context.slavelist[slave].Ibytes;
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
                     obytes,
                     ibytes);
        if (obytes != static_cast<int>(sizeof(ServoRxPdo)) ||
            ibytes != static_cast<int>(sizeof(ServoTxPdo))) {
            std::fprintf(stderr,
                         "warning: Servo %zu PDO size mismatch, expected "
                         "out=%zu in=%zu bytes\n",
                         i + 1,
                         sizeof(ServoRxPdo),
                         sizeof(ServoTxPdo));
        }
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
    if (app.context.slavelist[kEndIoLogicalId].Obytes !=
            static_cast<int>(sizeof(EndIoRxPdo)) ||
        app.context.slavelist[kEndIoLogicalId].Ibytes !=
            static_cast<int>(sizeof(EndIoTxPdo))) {
        std::fprintf(stderr,
                     "warning: EndIO PDO size mismatch, expected out=%zu "
                     "in=%zu bytes\n",
                     sizeof(EndIoRxPdo),
                     sizeof(EndIoTxPdo));
    }
    return 0;
}

/*
 * SOEM 实时通信线程。
 *
 * 线程职责：
 * - 按 1 ms 周期收发 process data。
 * - 使用 DC PI 修正本机唤醒时间。
 * - 周期性写入第 6 轴运动控制 PDO。
 * - 调用 ecx_mbxhandler() 处理周期 mailbox。
 */
OSAL_THREAD_FUNC_RT ecatthread(void *arg) {
    /* app：由 osal_thread_create_rt() 传入的主站运行期上下文。*/
    App *app = static_cast<App *>(arg);
    /* wakeup：下一次周期唤醒的绝对单调时间。*/
    ec_timet wakeup {};
    /* time_offset_ns：DC PI 算法输出的唤醒修正量。*/
    int64_t time_offset_ns = 0;
    /* cycle：RT 线程周期计数，用于低频报告节拍。*/
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

/*
 * SOEM 状态检查线程。
 *
 * 当 WKC 连续异常或 group 要求检查状态时，尝试将异常从站恢复到 OP。
 */
OSAL_THREAD_FUNC ecatcheck(void *arg) {
    /* app：由 osal_thread_create() 传入的主站运行期上下文。*/
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
                /* slave：当前检查的 SOEM 从站状态对象。*/
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
    if (configure_siasun_mailboxes(app)) {
        return -1;
    }

    /* axis_parameters：从 Axis1.xml-Axis6.xml 读取出的 6 轴参数。*/
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
