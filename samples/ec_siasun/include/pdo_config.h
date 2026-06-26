#ifndef SIASUN_SOEM_PDO_CONFIG_H
#define SIASUN_SOEM_PDO_CONFIG_H

#include <array>
#include <cstdint>

namespace siasun {

struct PdoEntry {
    uint16_t index;
    uint8_t subindex;
    uint8_t bits;
};

inline constexpr std::array<PdoEntry, 9> kServoRxPdoEntries {{
    {0x607A, 0x00, 32}, {0x60FE, 0x00, 32}, {0x60FF, 0x00, 32},
    {0x6040, 0x00, 16}, {0x6071, 0x00, 16}, {0x6060, 0x00, 8},
    {0x7006, 0x00, 8},  {0x7007, 0x00, 32}, {0x7008, 0x00, 32},
}};

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

#pragma pack(push, 1)
struct ServoRxPdo {
    int32_t target_position;
    uint32_t digital_outputs;
    int32_t target_velocity;
    uint16_t control_word;
    int16_t target_torque;
    int8_t operation_mode;
    uint8_t safe_control;
    int32_t target_safe_position;
    uint32_t user_output;
};

struct ServoTxPdo {
    int32_t actual_position;
    uint32_t digital_input;
    int32_t position_demand_value;
    int32_t velocity_sensor_actual_value;
    uint32_t error_code;
    int32_t actual_velocity;
    uint16_t status_word;
    int16_t actual_torque;
    int16_t current_actual_value;
    int8_t operation_mode_display;
    uint8_t gap;
    uint16_t sync0_time_difference;
    uint32_t sync0_count;
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

struct EndIoRxPdo {
    uint8_t led_work_control;
    uint8_t digital_outputs_control;
    uint16_t rs485_outputs_count;
    uint16_t rs485_outputs_len;
    uint8_t rs485_outputs_data[32];
};

struct EndIoTxPdo {
    uint8_t error_code;
    uint8_t digital_inputs;
    uint16_t analog_voltage_1;
    uint16_t analog_voltage_2;
    int16_t temperature;
    int16_t acceleration_1;
    int16_t acceleration_2;
    int16_t acceleration_3;
    uint16_t rs485_inputs_count;
    uint16_t rs485_inputs_len;
    uint8_t rs485_inputs_data[32];
};
#pragma pack(pop)

}  // namespace siasun

#endif
