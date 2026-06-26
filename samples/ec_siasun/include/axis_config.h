#ifndef SIASUN_SOEM_AXIS_CONFIG_H
#define SIASUN_SOEM_AXIS_CONFIG_H

#include <array>
#include <string>
#include <vector>

#include "app_config.h"

namespace siasun {

/* 单个 ServoParameters XML 节点解析得到的伺服参数。*/
struct ServoParameter {
    /* 参数 ID，对应 Axis*.xml 中的 <Id>。*/
    int id = -1;
    /* 参数名称，对应 Axis*.xml 中的 <Name>，仅用于日志和排查。*/
    std::string name;
    /* 参数值，对应 Axis*.xml 中的 <Value>。*/
    double value = 0.0;
    /* 定点数缩放位数，对应 Axis*.xml 中的 <qFmt>。*/
    int qfmt = 0;
};

/* 单个轴的全部有效参数。*/
using AxisParameters = std::vector<ServoParameter>;
/* 6 个轴的参数集合；数组下标 0-5 对应 Axis1.xml-Axis6.xml。*/
using AxisParameterSet = std::array<AxisParameters, kServoCount>;

/*
 * 读取 Axis1.xml 到 Axis6.xml。
 *
 * directory：Axis*.xml 所在目录。
 * parameters：输出参数集合，按轴号分别保存。
 */
int load_axis_parameter_set(const std::string &directory,
                            AxisParameterSet &parameters);

/*
 * 选择可用的 Axis*.xml 目录。
 *
 * requested_directory：命令行显式传入的目录；为空时使用内置候选路径。
 * selected_directory：最终找到并用于读取的目录。
 */
int resolve_axis_config_directory(const std::string &requested_directory,
                                  std::string &selected_directory);

}  // namespace siasun

#endif
