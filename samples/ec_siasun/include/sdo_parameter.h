#ifndef SIASUN_SOEM_SDO_PARAMETER_H
#define SIASUN_SOEM_SDO_PARAMETER_H

#include <cstdint>

#include "axis_config.h"
#include "soem/soem.h"

namespace siasun {

int write_axis_parameters(ecx_contextt *context,
                          const AxisParameterSet &parameters);
int write_servo_parameter(ecx_contextt *context,
                          uint16_t slave,
                          const ServoParameter &parameter);

}  // namespace siasun

#endif
