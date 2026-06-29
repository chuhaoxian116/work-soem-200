#ifndef SIASUN_SOEM_PDO_CONFIG_H
#define SIASUN_SOEM_PDO_CONFIG_H

#include <array>
#include <cstddef>
#include <cstdint>

namespace siasun {

/*
 * 单个 PDO entry 描述。
 *
 * 这些表只描述应用期望的从站默认布局并用于编译期尺寸校验，
 * 程序不会把它们通过 SDO 写入 0x1600/0x1A00。
 */
struct PdoEntry {
    /* 对象字典 index，例如 0x607A 表示目标位置。*/
    uint16_t index;
    /* 对象字典 subindex。*/
    uint8_t subindex;
    /* 该 PDO entry 的位宽。*/
    uint8_t bits;
};

/*
 * 伺服 RxPDO entry 列表。
 *
 * RxPDO 是主站写给伺服的过程数据。当前程序周期性刷新全部输出区，
 * 但只对第 6 轴写入使能控制字、CSP 模式和目标位置，其余轴保持 0 输出。
 */
inline constexpr std::array<PdoEntry, 9> kServoRxPdoEntries {{
    {0x607A, 0x00, 32}, {0x60FE, 0x00, 32}, {0x60FF, 0x00, 32},
    {0x6040, 0x00, 16}, {0x6071, 0x00, 16}, {0x6060, 0x00, 8},
    {0x7006, 0x00, 8},  {0x7007, 0x00, 32}, {0x7008, 0x00, 32},
}};

/*
 * 伺服 TxPDO entry 列表。
 *
 * TxPDO 是主站从伺服读取的过程数据，包含位置、速度、状态字、错误码、
 * Sync0 时间差以及安全板相关状态。下面 packed 结构体必须与该顺序一致。
 */
inline constexpr std::array<PdoEntry, 26> kServoTxPdoEntries {{
    {0x6064, 0x00, 32}, {0x60FD, 0x00, 32}, {0x6063, 0x00, 32},
    {0x6069, 0x00, 32}, {0x603F, 0x00, 32}, {0x606C, 0x00, 32},
    {0x6041, 0x00, 16}, {0x6077, 0x00, 16}, {0x6078, 0x00, 16},
    {0x6061, 0x00, 8},  {0x0000, 0x00, 8},  {0x600B, 0x00, 16},
    {0x600C, 0x00, 32}, {0x600D, 0x01, 32}, {0x600D, 0x02, 16},
    {0x600D, 0x03, 32}, {0x600D, 0x04, 32}, {0x600D, 0x05, 16},
    {0x600D, 0x06, 16}, {0x600D, 0x07, 32}, {0x600D, 0x08, 32},
    {0x600D, 0x09, 16}, {0x600D, 0x0A, 16}, {0x600D, 0x0B, 32},
    {0x600D, 0x0C, 32}, {0x600D, 0x0D, 32},
}};

/*
 * 末端 IO RxPDO entry 列表。
 *
 * 这些 entry 是主站写给末端 IO 的 LED、数字输出和 RS485 输出缓冲。
 * 当前业务不主动输出 IO 数据，但仍保持完整映射并在周期中写 0。
 */
inline constexpr std::array<PdoEntry, 36> kEndIoRxPdoEntries {{
    {0x7000, 0x01, 8},  {0x7000, 0x02, 8},  {0x7000, 0x03, 16},
    {0x7000, 0x04, 16}, {0x7000, 0x05, 8},  {0x7000, 0x06, 8},
    {0x7000, 0x07, 8},  {0x7000, 0x08, 8},  {0x7000, 0x09, 8},
    {0x7000, 0x0A, 8},  {0x7000, 0x0B, 8},  {0x7000, 0x0C, 8},
    {0x7000, 0x0D, 8},  {0x7000, 0x0E, 8},  {0x7000, 0x0F, 8},
    {0x7000, 0x10, 8},  {0x7000, 0x11, 8},  {0x7000, 0x12, 8},
    {0x7000, 0x13, 8},  {0x7000, 0x14, 8},  {0x7000, 0x15, 8},
    {0x7000, 0x16, 8},  {0x7000, 0x17, 8},  {0x7000, 0x18, 8},
    {0x7000, 0x19, 8},  {0x7000, 0x1A, 8},  {0x7000, 0x1B, 8},
    {0x7000, 0x1C, 8},  {0x7000, 0x1D, 8},  {0x7000, 0x1E, 8},
    {0x7000, 0x1F, 8},  {0x7000, 0x20, 8},  {0x7000, 0x21, 8},
    {0x7000, 0x22, 8},  {0x7000, 0x23, 8},  {0x7000, 0x24, 8},
}};

/*
 * 末端 IO TxPDO entry 列表。
 *
 * 这些 entry 是主站读取的末端 IO 状态，包括错误码、数字输入、模拟量、
 * 温度、三轴加速度和 RS485 输入缓冲。
 */
inline constexpr std::array<PdoEntry, 42> kEndIoTxPdoEntries {{
    {0x6000, 0x01, 8},  {0x6000, 0x02, 8},  {0x6000, 0x03, 16},
    {0x6000, 0x04, 16}, {0x6000, 0x05, 16}, {0x6000, 0x06, 16},
    {0x6000, 0x07, 16}, {0x6000, 0x08, 16}, {0x6000, 0x09, 16},
    {0x6000, 0x0A, 16}, {0x6000, 0x0B, 8},  {0x6000, 0x0C, 8},
    {0x6000, 0x0D, 8},  {0x6000, 0x0E, 8},  {0x6000, 0x0F, 8},
    {0x6000, 0x10, 8},  {0x6000, 0x11, 8},  {0x6000, 0x12, 8},
    {0x6000, 0x13, 8},  {0x6000, 0x14, 8},  {0x6000, 0x15, 8},
    {0x6000, 0x16, 8},  {0x6000, 0x17, 8},  {0x6000, 0x18, 8},
    {0x6000, 0x19, 8},  {0x6000, 0x1A, 8},  {0x6000, 0x1B, 8},
    {0x6000, 0x1C, 8},  {0x6000, 0x1D, 8},  {0x6000, 0x1E, 8},
    {0x6000, 0x1F, 8},  {0x6000, 0x20, 8},  {0x6000, 0x21, 8},
    {0x6000, 0x22, 8},  {0x6000, 0x23, 8},  {0x6000, 0x24, 8},
    {0x6000, 0x25, 8},  {0x6000, 0x26, 8},  {0x6000, 0x27, 8},
    {0x6000, 0x28, 8},  {0x6000, 0x29, 8},  {0x6000, 0x2A, 8},
}};

/*
 * SOEM 直接把 slavelist[n].outputs/inputs 指向 IOmap 中的字节区域。
 * 因此 C++ 结构体必须 1 字节对齐，字段顺序也必须严格匹配 PDO 映射顺序。
 */
#pragma pack(push, 1)
/* 伺服 RxPDO 输出区：主站 -> 伺服。*/
struct ServoRxPdo {
    /* 0x607A:00 目标位置。*/
    int32_t target_position;
    /* 0x60FE:00 数字输出。*/
    uint32_t digital_outputs;
    /* 0x60FF:00 目标速度。*/
    int32_t target_velocity;
    /* 0x6040:00 CiA402 控制字。*/
    uint16_t control_word;
    /* 0x6071:00 目标转矩。*/
    int16_t target_torque;
    /* 0x6060:00 目标运行模式。*/
    int8_t operation_mode;
    /* 0x7006:00 安全控制字节。*/
    uint8_t safe_control;
    /* 0x7007:00 安全目标位置。*/
    int32_t target_safe_position;
    /* 0x7008:00 用户输出。*/
    uint32_t user_output;
};

/* 伺服 TxPDO 输入区：伺服 -> 主站。*/
struct ServoTxPdo {
    /* 0x6064:00 实际位置。*/
    int32_t actual_position;
    /* 0x60FD:00 数字输入。*/
    uint32_t digital_input;
    /* 0x6063:00 位置需求值。*/
    int32_t position_demand_value;
    /* 0x6069:00 速度传感器实际值。*/
    int32_t velocity_sensor_actual_value;
    /* 0x603F:00 错误码。*/
    uint32_t error_code;
    /* 0x606C:00 实际速度。*/
    int32_t actual_velocity;
    /* 0x6041:00 CiA402 状态字。*/
    uint16_t status_word;
    /* 0x6077:00 实际转矩。*/
    int16_t actual_torque;
    /* 0x6078:00 实际电流。*/
    int16_t current_actual_value;
    /* 0x6061:00 当前运行模式显示。*/
    int8_t operation_mode_display;
    /* 0x0000:00 8-bit gap，占位以保持后续字段偏移。*/
    uint8_t gap;
    /* 0x600B:00 伺服反馈的 Sync0 时间差。*/
    uint16_t sync0_time_difference;
    /* 0x600C:00 Sync0 计数。*/
    uint32_t sync0_count;
    /* 0x600D:01~0x600D:0D 安全板状态数据，当前仅映射并报告。*/
    uint32_t safety_status_1;
    uint16_t safety_status_2;
    uint32_t safety_status_3;
    uint32_t safety_status_4;
    uint16_t safety_status_5;
    uint16_t safety_status_6;
    uint32_t safety_status_7;
    uint32_t safety_status_8;
    uint16_t safety_status_9;
    uint16_t safety_status_10;
    uint32_t safety_status_11;
    uint32_t safety_status_12;
    uint32_t safety_status_13;
};

/* 末端 IO RxPDO 输出区：主站 -> 末端 IO。*/
struct EndIoRxPdo {
    /* 0x7000:01 LED 工作控制。*/
    uint8_t led_work_control;
    /* 0x7000:02 数字输出控制。*/
    uint8_t digital_outputs_control;
    /* 0x7000:03 RS485 输出帧计数。*/
    uint16_t rs485_outputs_count;
    /* 0x7000:04 RS485 输出数据长度。*/
    uint16_t rs485_outputs_len;
    /* 0x7000:05~0x7000:24 RS485 输出数据 1~32。*/
    uint8_t rs485_outputs_data[32];
};

/* 末端 IO TxPDO 输入区：末端 IO -> 主站。*/
struct EndIoTxPdo {
    /* 0x6000:01 错误码。*/
    uint8_t error_code;
    /* 0x6000:02 数字输入。*/
    uint8_t digital_inputs;
    /* 0x6000:03/04 两路模拟量输入。*/
    uint16_t analog_voltage_1;
    uint16_t analog_voltage_2;
    /* 0x6000:05 温度值。*/
    int16_t temperature;
    /* 0x6000:06~08 三轴加速度。*/
    int16_t acceleration_1;
    int16_t acceleration_2;
    int16_t acceleration_3;
    /* 0x6000:09/0A RS485 输入计数和长度。*/
    uint16_t rs485_inputs_count;
    uint16_t rs485_inputs_len;
    /* 0x6000:0B~0x6000:2A RS485 输入数据 1~32。*/
    uint8_t rs485_inputs_data[32];
};
#pragma pack(pop)

/* 编译期确认 packed 结构体和 PDO entry 位宽总和完全一致。*/
template <std::size_t N>
constexpr std::size_t pdo_mapped_bytes(
    const std::array<PdoEntry, N> &entries) {
    std::size_t bits = 0;
    for (const PdoEntry &entry : entries) {
        bits += entry.bits;
    }
    return (bits + 7U) / 8U;
}

static_assert(sizeof(ServoRxPdo) == pdo_mapped_bytes(kServoRxPdoEntries),
              "ServoRxPdo does not match 0x1600 mapping");
static_assert(sizeof(ServoTxPdo) == pdo_mapped_bytes(kServoTxPdoEntries),
              "ServoTxPdo does not match 0x1A00 mapping");
static_assert(sizeof(EndIoRxPdo) == pdo_mapped_bytes(kEndIoRxPdoEntries),
              "EndIoRxPdo does not match 0x1600 mapping");
static_assert(sizeof(EndIoTxPdo) == pdo_mapped_bytes(kEndIoTxPdoEntries),
              "EndIoTxPdo does not match 0x1A00 mapping");

}  // namespace siasun

#endif
