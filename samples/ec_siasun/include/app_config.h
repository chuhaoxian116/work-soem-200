#ifndef SIASUN_SOEM_APP_CONFIG_H
#define SIASUN_SOEM_APP_CONFIG_H

#include <cstddef>
#include <cstdint>

namespace siasun {

constexpr uint32_t kVendorId = 0x000008CF;
constexpr uint32_t kServoProductCode = 0x00009252;
constexpr uint32_t kEndIoProductCode = 0x00009250;

constexpr int64_t kCycleTimeNs = 1000000;
constexpr uint16_t kDcAssignActivate = 0x0300;
constexpr uint64_t kQualityReportPeriodCycles = 60000;

constexpr bool kLogAxisXmlParameterDetails = true;
constexpr bool kLogSdoDownloadDetails = true;

constexpr int kMaxSafeStack = 8 * 1024;
constexpr std::size_t kServoCount = 6;
constexpr std::size_t kMotionServoIndex = kServoCount - 1;
constexpr int8_t kDriveOperationMode = 8;

constexpr uint64_t kEnableStepCycles = 1000;
constexpr int32_t kMotionAmplitudeCounts = 100000;
constexpr int32_t kMotionRangeCounts = kMotionAmplitudeCounts * 2;
constexpr uint64_t kMotionPeriodCycles = 20000;
constexpr uint64_t kMotionRampCycles = 2000;

constexpr uint16_t kEndIoLogicalId = 7;
constexpr std::size_t kExpectedSlaveCount = 7;
constexpr std::size_t kIoMapSize = 8192;

}  // namespace siasun

#endif
