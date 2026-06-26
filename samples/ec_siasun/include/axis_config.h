#ifndef SIASUN_SOEM_AXIS_CONFIG_H
#define SIASUN_SOEM_AXIS_CONFIG_H

#include <array>
#include <string>
#include <vector>

#include "app_config.h"

namespace siasun {

struct ServoParameter {
    int id = -1;
    std::string name;
    double value = 0.0;
    int qfmt = 0;
};

using AxisParameters = std::vector<ServoParameter>;
using AxisParameterSet = std::array<AxisParameters, kServoCount>;

int load_axis_parameter_set(const std::string &directory,
                            AxisParameterSet &parameters);
int resolve_axis_config_directory(const std::string &requested_directory,
                                  std::string &selected_directory);

}  // namespace siasun

#endif
