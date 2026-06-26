#ifndef SIASUN_SOEM_ETHERCAT_APP_H
#define SIASUN_SOEM_ETHERCAT_APP_H

#include <array>
#include <csignal>
#include <cstdint>
#include <string>

#include "app_config.h"
#include "pdo_config.h"
#include "soem/soem.h"

namespace siasun {

struct App {
    ecx_contextt context {};
    std::array<uint8_t, kIoMapSize> io_map {};

    std::array<ServoRxPdo *, kServoCount> servo_rx {};
    std::array<ServoTxPdo *, kServoCount> servo_tx {};
    EndIoRxPdo *endio_rx = nullptr;
    EndIoTxPdo *endio_tx = nullptr;

    OSAL_THREAD_HANDLE rt_thread {};
    OSAL_THREAD_HANDLE check_thread {};

    int expected_wkc = 0;
    int last_wkc = 0;
    int wkc_check_misses = 0;
    int current_group = 0;

    volatile int run = 0;
    volatile int mapping_done = 0;
    volatile int do_run = 0;
    bool in_op = false;
};

int configure(App &app,
              const char *ifname,
              const std::string &axis_config_directory);
void setup_realtime_process();
void run(App &app, volatile std::sig_atomic_t &keep_running);
void release(App &app);

}  // namespace siasun

#endif
