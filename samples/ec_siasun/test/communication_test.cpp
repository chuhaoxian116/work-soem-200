#include <cerrno>
#include <climits>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/resource.h>
#include <unistd.h>

#include "axis_config.h"
#include "ethercat_app.h"

namespace {

/* keep_running：收到退出信号或测试定时结束后置 0。 */
volatile std::sig_atomic_t keep_running = 1;

/*
 * 结束测试。
 *
 * signal_number：SIGINT、SIGTERM 或 SIGALRM 的信号编号。
 */
void signal_handler(int signal_number) {
    /* ignored_signal_number：显式消费未使用的信号编号。 */
    const int ignored_signal_number = signal_number;
    (void)ignored_signal_number;
    keep_running = 0;
}

/* 安装手动退出和定时结束信号。 */
void install_signal_handlers() {
    /* action：三个退出信号共用的处理配置。 */
    struct sigaction action {};
    action.sa_handler = signal_handler;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
    sigaction(SIGALRM, &action, nullptr);
}

/*
 * 解析非负整数参数。
 *
 * text：待解析文本。
 * name：参数名称，用于错误日志。
 * output：解析成功后的整数值。
 */
bool parse_unsigned(const char *text,
                    const char *name,
                    unsigned int &output) {
    /* end：解析结束位置。 */
    char *end = nullptr;
    errno = 0;

    /* value：strtoul() 返回的临时值。 */
    const unsigned long value = std::strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || text[0] == '-' ||
        value > UINT_MAX) {
        std::fprintf(stderr, "invalid %s: %s\n", name, text);
        return false;
    }

    output = static_cast<unsigned int>(value);
    return true;
}

/* 将 timeval 转换为秒。 */
double timeval_to_seconds(const timeval &value) {
    return static_cast<double>(value.tv_sec) +
           static_cast<double>(value.tv_usec) / 1000000.0;
}

/*
 * 打印测试通讯窗口的进程资源占用。
 *
 * before：进入周期通讯前的资源快照。
 * after：结束周期通讯后的资源快照。
 * duration_seconds：测试配置的墙钟运行时间。
 */
void print_resource_usage(const rusage &before,
                          const rusage &after,
                          unsigned int duration_seconds) {
    /* user_seconds：测试窗口使用的用户态 CPU 时间。 */
    const double user_seconds =
        timeval_to_seconds(after.ru_utime) -
        timeval_to_seconds(before.ru_utime);

    /* system_seconds：测试窗口使用的内核态 CPU 时间。 */
    const double system_seconds =
        timeval_to_seconds(after.ru_stime) -
        timeval_to_seconds(before.ru_stime);

    /* cpu_percent：单个逻辑 CPU 口径下的平均占用。 */
    const double cpu_percent =
        duration_seconds > 0
            ? (user_seconds + system_seconds) * 100.0 /
                  static_cast<double>(duration_seconds)
            : 0.0;

    std::printf("[RESOURCE] cpu_percent=%.3f user_s=%.6f system_s=%.6f\n",
                cpu_percent,
                user_seconds,
                system_seconds);
    std::printf("[RESOURCE] max_rss_kb=%ld minor_faults=%ld "
                "major_faults=%ld\n",
                after.ru_maxrss,
                after.ru_minflt - before.ru_minflt,
                after.ru_majflt - before.ru_majflt);
    std::printf("[RESOURCE] voluntary_cs=%ld involuntary_cs=%ld\n",
                after.ru_nvcsw - before.ru_nvcsw,
                after.ru_nivcsw - before.ru_nivcsw);
}

/* 打印 SOEM 测试 demo 的参数格式。 */
void print_usage(const char *program_name) {
    std::fprintf(
        stderr,
        "usage: %s ifname [AxisXmlDirectory] [duration_s] "
        "[require_endio_op]\n"
        "defaults: duration_s=60 require_endio_op=0\n"
        "require_endio_op=0 judges communication using Servo 1-6 only\n",
        program_name);
}

}  // namespace

/* SOEM SIASUN 六关节通讯、DC、运动和资源统计测试入口。 */
int main(int argc, char *argv[]) {
    if (argc < 2 || argc > 5) {
        print_usage(argv[0]);
        return 1;
    }

    /* ifname：SOEM 使用的 EtherCAT 物理网卡名称。 */
    const char *ifname = argv[1];

    /* requested_directory：Axis1.xml-Axis6.xml 参数目录。 */
    const std::string requested_directory = argc > 2 ? argv[2] : "";

    /* duration_seconds：周期通讯测试时长。 */
    unsigned int duration_seconds = 60;

    /* require_endio_op：1 要求7站 OP，0 只要求6个伺服 OP。 */
    unsigned int require_endio_op = 0;

    if ((argc > 3 &&
         !parse_unsigned(argv[3], "duration_s", duration_seconds)) ||
        (argc > 4 &&
         !parse_unsigned(argv[4],
                         "require_endio_op",
                         require_endio_op))) {
        print_usage(argv[0]);
        return 1;
    }
    if (duration_seconds == 0) {
        std::fprintf(stderr, "duration_s must be greater than zero\n");
        return 1;
    }
    if (require_endio_op > 1) {
        std::fprintf(stderr, "require_endio_op must be 0 or 1\n");
        return 1;
    }

    /* axis_config_directory：校验后的实际 Axis XML 目录。 */
    std::string axis_config_directory;
    if (siasun::resolve_axis_config_directory(requested_directory,
                                              axis_config_directory)) {
        return 1;
    }

    std::printf("[TEST] duration=%u s interface=%s axis_dir=%s\n",
                duration_seconds,
                ifname,
                axis_config_directory.c_str());
    std::printf("[TEST] communication gate=%s\n",
                require_endio_op == 1
                    ? "Servo 1-6 + EndIO 7"
                    : "Servo 1-6 only; EndIO 7 is optional");

    install_signal_handlers();
    siasun::setup_realtime_process();

    /* app：SOEM SIASUN 主站运行上下文。 */
    siasun::App app;
    if (siasun::configure(app,
                          ifname,
                          axis_config_directory,
                          require_endio_op == 1)) {
        siasun::release(app);
        return 1;
    }

    /* usage_before：进入周期通讯前的资源统计快照。 */
    rusage usage_before {};
    getrusage(RUSAGE_SELF, &usage_before);

    alarm(duration_seconds);
    siasun::run(app, keep_running);

    /* usage_after：退出周期通讯后的资源统计快照。 */
    rusage usage_after {};
    const int usage_status = getrusage(RUSAGE_SELF, &usage_after);

    siasun::release(app);

    if (usage_status == 0) {
        print_resource_usage(usage_before, usage_after, duration_seconds);
    } else {
        std::fprintf(stderr,
                     "warning: getrusage failed: %s\n",
                     std::strerror(errno));
    }

    return 0;
}
