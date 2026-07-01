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

/* SOEM 主站运行期上下文，集中保存 context、IOmap、PDO 指针和线程状态。*/
struct App {
    /* SOEM 主站上下文；所有 ecx_* API 都通过该对象访问网卡和从站列表。*/
    ecx_contextt context {};
    /* SOEM process data 映射缓冲区，由 ecx_config_map_group() 填充。*/
    std::array<uint8_t, kIoMapSize> io_map {};

    /* 6 个伺服 RxPDO 输出指针，数组下标 0-5 对应 SOEM 从站 1-6。*/
    std::array<ServoRxPdo *, kServoCount> servo_rx {};
    /* 6 个伺服 TxPDO 输入指针，用于读取状态字、实际位置和错误码。*/
    std::array<ServoTxPdo *, kServoCount> servo_tx {};
    /* 末端 IO RxPDO 输出指针，对应 SOEM 从站 7 的 outputs 区。*/
    EndIoRxPdo *endio_rx = nullptr;
    /* 末端 IO TxPDO 输入指针，对应 SOEM 从站 7 的 inputs 区。*/
    EndIoTxPdo *endio_tx = nullptr;

    /* SOEM OSAL 实时通信线程句柄，周期收发 process data。*/
    OSAL_THREAD_HANDLE rt_thread {};
    /* SOEM OSAL 状态检查线程句柄，负责 OP 异常后的恢复。*/
    OSAL_THREAD_HANDLE check_thread {};

    /* 当前 PDO 映射对应的期望 Working Counter。*/
    int expected_wkc = 0;
    /* 最近一个周期 ecx_receive_processdata() 返回的 WKC。*/
    int last_wkc = 0;
    /* 连续 WKC 异常计数，超过阈值后触发 ecatcheck 检查。*/
    int wkc_check_misses = 0;
    /* 当前使用的 SOEM group；本例所有从站均放在 group 0。*/
    int current_group = 0;
    /* 质量统计和状态恢复必须覆盖的前置从站数量。*/
    std::size_t required_slave_count = kExpectedSlaveCount;

    /* 线程运行标志；置 0 后 RT/check 线程自然退出循环。*/
    volatile int run = 0;
    /* PDO 映射完成标志；RT 线程等待它置 1 后开始对齐周期。*/
    volatile int mapping_done = 0;
    /* 周期通信使能标志；进入 SAFE_OP warmup 后才开始正式收发。*/
    volatile int do_run = 0;
    /* 应用层 OP 标志；置 1 后才允许写入第 6 轴运动控制 PDO。*/
    bool in_op = false;
    /* true 要求 EndIO；false 只要求 6 个伺服。*/
    bool require_endio = true;
    /* EndIO PDO 已映射并可安全访问。*/
    bool endio_configured = false;
};

/* 初始化主站、下发 XML 参数、映射从站默认 PDO、配置 DC 并进入 OP。*/
int configure(App &app,
              const char *ifname,
              const std::string &axis_config_directory,
              bool require_endio = true);
/* 尽力设置实时调度、锁定内存并预触碰栈；失败只打印 warning。*/
void setup_realtime_process();
/* 主线程等待退出信号；周期通信已经由 SOEM RT 线程执行。*/
void run(App &app, volatile std::sig_atomic_t &keep_running);
/* 停止线程、清零输出、切回 SAFE_OP/INIT 并关闭 SOEM context。*/
void release(App &app);

}  // namespace siasun

#endif
