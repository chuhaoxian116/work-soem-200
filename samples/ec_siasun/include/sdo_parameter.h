#ifndef SIASUN_SOEM_SDO_PARAMETER_H
#define SIASUN_SOEM_SDO_PARAMETER_H

#include <cstdint>

#include "axis_config.h"
#include "soem/soem.h"

namespace siasun {

/*
 * 将 Axis*.xml 中的伺服参数写入 1-6 号伺服。
 *
 * context：SOEM 主站上下文。
 * parameters：6 个轴的参数集合。
 */
int write_axis_parameters(ecx_contextt *context,
                          const AxisParameterSet &parameters);

/*
 * 将单个伺服参数按 SINSUN mailbox 协议写入指定从站。
 *
 * context：SOEM 主站上下文。
 * slave：SOEM 1-based 从站编号。
 * parameter：需要写入的参数。
 */
int write_servo_parameter(ecx_contextt *context,
                          uint16_t slave,
                          const ServoParameter &parameter);

}  // namespace siasun

#endif
