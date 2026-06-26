#ifndef SIASUN_SOEM_APP_CONFIG_H
#define SIASUN_SOEM_APP_CONFIG_H

#include <cstddef>
#include <cstdint>

namespace siasun {

/* 新松 SINSUN 设备 Vendor ID，来自 ESI XML 的 Vendor/Id。*/
constexpr uint32_t kVendorId = 0x000008CF;
/* 伺服从站 ProductCode，拓扑中 1-6 号从站均使用该产品码。*/
constexpr uint32_t kServoProductCode = 0x00009252;
/* 末端 IO 从站 ProductCode，拓扑中第 7 个从站使用该产品码。*/
constexpr uint32_t kEndIoProductCode = 0x00009250;

/* SOEM 周期任务的目标通信周期，单位 ns；当前按 1 ms 运行。*/
constexpr int64_t kCycleTimeNs = 1000000;
/* DC AssignActivate，和 IgH 版使用的 SINSUN ESI DC 模式保持一致。*/
constexpr uint16_t kDcAssignActivate = 0x0300;
/* 通信质量累计报告周期；60000 个 1 ms 周期约等于 1 分钟。*/
constexpr uint64_t kQualityReportPeriodCycles = 60000;

/* 是否打印 Axis*.xml 中每条有效参数的解析结果。*/
constexpr bool kLogAxisXmlParameterDetails = true;
/* 是否打印 SDO mailbox 参数下发的每一步写入。*/
constexpr bool kLogSdoDownloadDetails = true;

/* 实时循环前预触碰的栈大小，用于降低运行期缺页概率。*/
constexpr int kMaxSafeStack = 8 * 1024;
/* 拓扑中的伺服从站数量；SOEM 从站编号为 1-6。*/
constexpr std::size_t kServoCount = 6;
/* 仅使能和运动最后一轴；数组下标 5 对应用户侧 Servo 6。*/
constexpr std::size_t kMotionServoIndex = kServoCount - 1;
/* 第 6 轴目标运行模式：8 表示 CSP(Cyclic Synchronous Position)。*/
constexpr int8_t kDriveOperationMode = 8;

/* CiA402 使能过程中 0x06/0x07/0x0F 各保持 1000 个通信周期。*/
constexpr uint64_t kEnableStepCycles = 1000;
/* 第 6 轴正弦运动幅值，单位 pulse。*/
constexpr int32_t kMotionAmplitudeCounts = 100000;
/* 正弦运动峰峰值，便于日志打印和现场确认。*/
constexpr int32_t kMotionRangeCounts = kMotionAmplitudeCounts * 2;
/* 正弦运动周期，单位为 1 ms 通信周期数。*/
constexpr uint64_t kMotionPeriodCycles = 20000;
/* 正弦启动包络时间；前 2 秒从 0 平滑拉到设定幅值。*/
constexpr uint64_t kMotionRampCycles = 2000;

/* 末端 IO 的用户侧逻辑 id；SOEM 从站编号同样为 7。*/
constexpr uint16_t kEndIoLogicalId = 7;
/* 期望拓扑从站数：6 个伺服 + 1 个末端 IO。*/
constexpr std::size_t kExpectedSlaveCount = 7;
/* SOEM process data IOmap 缓冲区大小，覆盖当前完整 PDO 映射。*/
constexpr std::size_t kIoMapSize = 8192;

}  // namespace siasun

#endif
