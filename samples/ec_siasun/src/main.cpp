#include <csignal>
#include <cstdio>
#include <string>

#include "axis_config.h"
#include "ethercat_app.h"

namespace {

/* 运行标志；信号处理函数置 0 后，主线程等待循环会自然退出。*/
volatile std::sig_atomic_t keep_running = 1;

/* 进程信号处理函数，只修改 sig_atomic_t 标志，避免在信号上下文做复杂操作。*/
void signal_handler(int) {
    keep_running = 0;
}

/* 安装 Ctrl+C 和进程终止信号处理，保证 SOEM 主站可以干净退回 INIT。*/
void install_signal_handlers() {
    /* sa：POSIX sigaction 配置结构体，指定 signal_handler 作为回调。*/
    struct sigaction sa {};
    sa.sa_handler = signal_handler;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

}  // namespace

/* SINSUN SOEM 主站程序入口。*/
int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: %s <ifname> [AxisXmlDirectory]\n"
                     "example: %s enp1s0 "
                     "samples/ec_siasun/doc/gcr10_1300\n",
                     argv[0],
                     argv[0]);
        return 1;
    }

    /* ifname：SOEM 使用的网卡名，例如 enp1s0。*/
    const char *ifname = argv[1];
    /* requested_directory：命令行传入的 Axis*.xml 目录，可为空。*/
    const std::string requested_directory = argc > 2 ? argv[2] : "";
    /* axis_config_directory：最终解析得到的 Axis*.xml 目录。*/
    std::string axis_config_directory;

    if (siasun::resolve_axis_config_directory(requested_directory,
                                              axis_config_directory)) {
        return 1;
    }

    std::printf("using Axis*.xml directory: %s\n",
                axis_config_directory.c_str());

    /* app：SINSUN SOEM 主站运行期上下文。*/
    siasun::App app;
    install_signal_handlers();
    siasun::setup_realtime_process();

    if (siasun::configure(app, ifname, axis_config_directory)) {
        siasun::release(app);
        return 1;
    }

    siasun::run(app, keep_running);
    siasun::release(app);

    std::printf("stopped\n");
    return 0;
}
