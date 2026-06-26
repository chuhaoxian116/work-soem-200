#include <csignal>
#include <cstdio>
#include <string>

#include "axis_config.h"
#include "ethercat_app.h"

namespace {

volatile std::sig_atomic_t keep_running = 1;

void signal_handler(int) {
    keep_running = 0;
}

void install_signal_handlers() {
    struct sigaction sa {};
    sa.sa_handler = signal_handler;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

}  // namespace

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

    const char *ifname = argv[1];
    const std::string requested_directory = argc > 2 ? argv[2] : "";
    std::string axis_config_directory;

    if (siasun::resolve_axis_config_directory(requested_directory,
                                              axis_config_directory)) {
        return 1;
    }

    std::printf("using Axis*.xml directory: %s\n",
                axis_config_directory.c_str());

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
