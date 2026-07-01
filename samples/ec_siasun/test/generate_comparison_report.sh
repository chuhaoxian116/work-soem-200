#!/usr/bin/env bash

set -u
set -o pipefail

# 检查参数数量并打印用法。
if [[ "$#" -lt 2 || "$#" -gt 3 ]]; then
    printf 'usage: %s IGH_RESULT_DIR SOEM_RESULT_DIR [OUTPUT.md]\n' "$0" >&2
    exit 1
fi

# IGH_RESULT_DIR：IgH run_realtime_test.sh 生成的单组结果目录。
IGH_RESULT_DIR="$1"
# SOEM_RESULT_DIR：SOEM run_realtime_test.sh 生成的单组结果目录。
SOEM_RESULT_DIR="$2"
# OUTPUT_PATH：生成的 Markdown 对比报告路径。
OUTPUT_PATH="${3:-igh-soem-comparison-$(date +%Y%m%d-%H%M%S).md}"

# IGH_LOG：IgH 最终通讯质量日志。
IGH_LOG="${IGH_RESULT_DIR}/communication.log"
# IGH_ENV：IgH 测试环境和变量日志。
IGH_ENV="${IGH_RESULT_DIR}/environment.log"
# IGH_STATUS：IgH 脚本退出状态日志。
IGH_STATUS="${IGH_RESULT_DIR}/status.log"
# SOEM_LOG：SOEM 最终通讯质量日志。
SOEM_LOG="${SOEM_RESULT_DIR}/communication.log"
# SOEM_ENV：SOEM 测试环境和变量日志。
SOEM_ENV="${SOEM_RESULT_DIR}/environment.log"
# SOEM_STATUS：SOEM 脚本退出状态日志。
SOEM_STATUS="${SOEM_RESULT_DIR}/status.log"

# 检查报告生成所需的结果文件。
for required_file in \
    "${IGH_LOG}" "${IGH_ENV}" "${IGH_STATUS}" \
    "${SOEM_LOG}" "${SOEM_ENV}" "${SOEM_STATUS}"; do
    if [[ ! -f "${required_file}" ]]; then
        printf 'required result file not found: %s\n' "${required_file}" >&2
        exit 1
    fi
done

# 读取最后一个“标签: 值”格式的日志字段。
# $1：日志文件。
# $2：冒号前的完整标签。
extract_label() {
    local file="$1"
    local expected_label="$2"

    awk -v expected_label="${expected_label}" '
        {
            separator = index($0, ":")
            if (separator == 0) {
                next
            }

            label = substr($0, 1, separator - 1)
            value = substr($0, separator + 1)
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", label)
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
            if (label == expected_label) {
                result = value
            }
        }
        END {
            print result == "" ? "N/A" : result
        }
    ' "${file}"
}

# 读取 [RESOURCE] 行中的 key=value 字段。
# $1：communication.log。
# $2：字段名称。
extract_resource() {
    local file="$1"
    local expected_key="$2"

    awk -v expected_key="${expected_key}" '
        index($0, "[RESOURCE]") {
            for (field = 1; field <= NF; ++field) {
                split($field, pair, "=")
                if (pair[1] == expected_key) {
                    result = pair[2]
                }
            }
        }
        END {
            print result == "" ? "N/A" : result
        }
    ' "${file}"
}

# 从 environment.log 的 KEY=value 设置中读取最后一个值。
# $1：environment.log。
# $2：变量名称。
extract_setting() {
    local file="$1"
    local expected_key="$2"

    awk -v expected_key="${expected_key}" '
        {
            for (field = 1; field <= NF; ++field) {
                split($field, pair, "=")
                if (pair[1] == expected_key) {
                    result = pair[2]
                }
            }
        }
        END {
            print result == "" ? "N/A" : result
        }
    ' "${file}"
}

# 读取 IgH 测试配置和结果。
igh_tag="$(extract_setting "${IGH_ENV}" "TEST_TAG")"
igh_duration="$(extract_setting "${IGH_ENV}" "DURATION_S")"
igh_cpu="$(extract_setting "${IGH_ENV}" "CPU_ID")"
igh_policy="$(extract_setting "${IGH_ENV}" "POLICY")"
igh_priority="$(extract_setting "${IGH_ENV}" "PRIORITY")"
igh_mlock="$(extract_setting "${IGH_ENV}" "MLOCK")"
igh_dc_sync="$(extract_setting "${IGH_ENV}" "DC_SYNC_CYCLES")"
igh_motion="$(extract_setting "${IGH_ENV}" "ENABLE_MOTION")"
igh_load="$(extract_setting "${IGH_ENV}" "LOAD_PROFILE")"
igh_load_cpuset="$(extract_setting "${IGH_ENV}" "LOAD_CPUSET")"
igh_cpu_workers="$(extract_setting "${IGH_ENV}" "CPU_WORKERS")"
igh_cpu_load="$(extract_setting "${IGH_ENV}" "CPU_LOAD")"
igh_vm_workers="$(extract_setting "${IGH_ENV}" "VM_WORKERS")"
igh_vm_bytes="$(extract_setting "${IGH_ENV}" "VM_BYTES")"
igh_endio="$(extract_setting "${IGH_ENV}" "REQUIRE_ENDIO_OP")"
igh_gate="$(extract_label "${IGH_LOG}" "判定对象")"
igh_cycles="$(extract_label "${IGH_LOG}" "累计周期")"
igh_avg_period="$(extract_label "${IGH_LOG}" "平均实际周期")"
igh_period_range="$(extract_label "${IGH_LOG}" "最小 / 最大周期")"
igh_avg_jitter="$(extract_label "${IGH_LOG}" "平均绝对抖动")"
igh_max_jitter="$(extract_label "${IGH_LOG}" "最大绝对抖动")"
igh_overruns="$(extract_label "${IGH_LOG}" "严重超周期次数")"
igh_complete="$(extract_label "${IGH_LOG}" "完整周期")"
igh_incomplete="$(extract_label "${IGH_LOG}" "不完整周期")"
igh_no_data="$(extract_label "${IGH_LOG}" "无过程数据周期")"
igh_success="$(extract_label "${IGH_LOG}" "通信成功率")"
igh_wc="$(extract_label "${IGH_LOG}" "WC 最小 / 最大")"
igh_dc_samples="$(extract_label "${IGH_LOG}" "有效 / 无效采样")"
igh_dc_avg="$(extract_label "${IGH_LOG}" "平均绝对误差")"
igh_dc_max="$(extract_label "${IGH_LOG}" "最大绝对误差")"
igh_cpu_percent="$(extract_resource "${IGH_LOG}" "cpu_percent")"
igh_rss="$(extract_resource "${IGH_LOG}" "max_rss_kb")"
igh_voluntary="$(extract_resource "${IGH_LOG}" "voluntary_cs")"
igh_involuntary="$(extract_resource "${IGH_LOG}" "involuntary_cs")"
igh_exit="$(extract_label "${IGH_STATUS}" "test exit status")"

# 读取 SOEM 测试配置和结果。
soem_tag="$(extract_setting "${SOEM_ENV}" "TEST_TAG")"
soem_duration="$(extract_setting "${SOEM_ENV}" "DURATION_S")"
soem_cpu="$(extract_setting "${SOEM_ENV}" "CPU_ID")"
soem_policy="$(extract_setting "${SOEM_ENV}" "POLICY")"
soem_priority="$(extract_setting "${SOEM_ENV}" "PRIORITY")"
soem_mlock="$(extract_setting "${SOEM_ENV}" "MLOCK")"
soem_dc_sync="$(extract_setting "${SOEM_ENV}" "DC_SYNC_CYCLES")"
soem_motion="$(extract_setting "${SOEM_ENV}" "ENABLE_MOTION")"
soem_load="$(extract_setting "${SOEM_ENV}" "LOAD_PROFILE")"
soem_load_cpuset="$(extract_setting "${SOEM_ENV}" "LOAD_CPUSET")"
soem_cpu_workers="$(extract_setting "${SOEM_ENV}" "CPU_WORKERS")"
soem_cpu_load="$(extract_setting "${SOEM_ENV}" "CPU_LOAD")"
soem_vm_workers="$(extract_setting "${SOEM_ENV}" "VM_WORKERS")"
soem_vm_bytes="$(extract_setting "${SOEM_ENV}" "VM_BYTES")"
soem_endio="$(extract_setting "${SOEM_ENV}" "REQUIRE_ENDIO_OP")"
soem_gate="$(extract_label "${SOEM_LOG}" "required topology")"
soem_cycles="$(extract_label "${SOEM_LOG}" "cycles")"
soem_avg_period="$(extract_label "${SOEM_LOG}" "avg period")"
soem_period_range="$(extract_label "${SOEM_LOG}" "min / max period")"
soem_avg_jitter="$(extract_label "${SOEM_LOG}" "avg abs jitter")"
soem_max_jitter="$(extract_label "${SOEM_LOG}" "max abs jitter")"
soem_overruns="$(extract_label "${SOEM_LOG}" "severe overruns")"
soem_good_bad="$(extract_label "${SOEM_LOG}" "good / bad cycles")"
soem_no_frame="$(extract_label "${SOEM_LOG}" "no frame cycles")"
soem_success="$(extract_label "${SOEM_LOG}" "success rate")"
soem_wc="$(extract_label "${SOEM_LOG}" "expected WKC")"
soem_dc_samples="$(extract_label "${SOEM_LOG}" "DC valid samples")"
soem_dc_values="$(extract_label "${SOEM_LOG}" "DC current / avg / max")"
soem_recovery="$(extract_label "${SOEM_LOG}" "recovery events")"
soem_cpu_percent="$(extract_resource "${SOEM_LOG}" "cpu_percent")"
soem_rss="$(extract_resource "${SOEM_LOG}" "max_rss_kb")"
soem_voluntary="$(extract_resource "${SOEM_LOG}" "voluntary_cs")"
soem_involuntary="$(extract_resource "${SOEM_LOG}" "involuntary_cs")"
soem_exit="$(extract_label "${SOEM_STATUS}" "test exit status")"

# report_date：报告生成时间。
report_date="$(date --iso-8601=seconds)"

{
    printf '# IgH 与 SOEM SIASUN 实时通讯对比报告\n\n'
    printf '生成时间：%s\n\n' "${report_date}"
    printf '## 数据来源\n\n'
    printf -- '- IgH：`%s`\n' "$(realpath "${IGH_RESULT_DIR}")"
    printf -- '- SOEM：`%s`\n\n' "$(realpath "${SOEM_RESULT_DIR}")"

    printf '## 测试条件\n\n'
    printf '| 项目 | IgH | SOEM |\n'
    printf '| --- | --- | --- |\n'
    printf '| TEST_TAG | %s | %s |\n' "${igh_tag}" "${soem_tag}"
    printf '| 配置时长 | %s s | %s s |\n' "${igh_duration}" "${soem_duration}"
    printf '| 应用 CPU | %s | %s |\n' "${igh_cpu}" "${soem_cpu}"
    printf '| 调度策略 | %s | %s |\n' "${igh_policy}" "${soem_policy}"
    printf '| 优先级参数 | %s | %s |\n' "${igh_priority}" "${soem_priority}"
    printf '| mlock | %s | %s |\n' "${igh_mlock}" "${soem_mlock}"
    printf '| DC 同步周期 | %s | %s |\n' "${igh_dc_sync}" "${soem_dc_sync}"
    printf '| Servo 6 运动 | %s | %s |\n' "${igh_motion}" "${soem_motion}"
    printf '| 压力类型 | %s | %s |\n' "${igh_load}" "${soem_load}"
    printf '| 压力 CPU | %s | %s |\n' \
        "${igh_load_cpuset:-空}" "${soem_load_cpuset:-空}"
    printf '| CPU workers / load | %s / %s%% | %s / %s%% |\n' \
        "${igh_cpu_workers}" "${igh_cpu_load}" \
        "${soem_cpu_workers}" "${soem_cpu_load}"
    printf '| VM workers / bytes | %s / %s | %s / %s |\n' \
        "${igh_vm_workers}" "${igh_vm_bytes}" \
        "${soem_vm_workers}" "${soem_vm_bytes}"
    printf '| REQUIRE_ENDIO_OP | %s | %s |\n' "${igh_endio}" "${soem_endio}"
    printf '| 通讯判定对象 | %s | %s |\n\n' "${igh_gate}" "${soem_gate}"

    printf '## 周期实时性\n\n'
    printf '| 指标 | IgH | SOEM |\n'
    printf '| --- | ---: | ---: |\n'
    printf '| 有效周期 | %s | %s |\n' "${igh_cycles}" "${soem_cycles}"
    printf '| 平均周期 | %s | %s |\n' "${igh_avg_period}" "${soem_avg_period}"
    printf '| 最小 / 最大周期 | %s | %s |\n' \
        "${igh_period_range}" "${soem_period_range}"
    printf '| 平均绝对抖动 | %s | %s |\n' \
        "${igh_avg_jitter}" "${soem_avg_jitter}"
    printf '| 最大绝对抖动 | %s | %s |\n' \
        "${igh_max_jitter}" "${soem_max_jitter}"
    printf '| 严重超周期 | %s | %s |\n\n' \
        "${igh_overruns}" "${soem_overruns}"

    printf '## 通讯完整性\n\n'
    printf '| 指标 | IgH | SOEM |\n'
    printf '| --- | ---: | ---: |\n'
    printf '| 完整 / Good 周期 | %s | %s |\n' \
        "${igh_complete}" "${soem_good_bad}"
    printf '| 不完整周期 | %s | 见 Good / Bad |\n' "${igh_incomplete}"
    printf '| 无数据 / 无帧周期 | %s | %s |\n' \
        "${igh_no_data}" "${soem_no_frame}"
    printf '| 通信成功率 | %s | %s |\n' "${igh_success}" "${soem_success}"
    printf '| WC | %s | %s |\n' "${igh_wc}" "${soem_wc}"
    printf '| SOEM 状态/重配/恢复事件 | - | %s |\n\n' "${soem_recovery}"

    printf '## DC 数据\n\n'
    printf '| 指标 | IgH | SOEM |\n'
    printf '| --- | ---: | ---: |\n'
    printf '| 样本 | 有效/无效：%s | 有效：%s |\n' \
        "${igh_dc_samples}" "${soem_dc_samples}"
    printf '| 平均绝对误差 | %s | 见下一行 avg |\n' "${igh_dc_avg}"
    printf '| 最大绝对误差 | %s | 见下一行 max |\n' "${igh_dc_max}"
    printf '| current / avg / max | - | %s |\n\n' "${soem_dc_values}"

    printf '> 注意：IgH 数据来自 `sync monitor`，用于观察 DC 从站相对参考时钟'
    printf '的差值；SOEM 数据来自本地周期与 `DCtime` 的 PI 相位误差。'
    printf '二者单位相同，但物理含义和采样频率不同，不应仅按数值大小直接排名。\n\n'

    printf '## 进程资源\n\n'
    printf '| 指标 | IgH | SOEM |\n'
    printf '| --- | ---: | ---: |\n'
    printf '| 平均 CPU（单核口径） | %s %% | %s %% |\n' \
        "${igh_cpu_percent}" "${soem_cpu_percent}"
    printf '| 最大 RSS | %s KiB | %s KiB |\n' "${igh_rss}" "${soem_rss}"
    printf '| 主动上下文切换 | %s | %s |\n' \
        "${igh_voluntary}" "${soem_voluntary}"
    printf '| 非主动上下文切换 | %s | %s |\n' \
        "${igh_involuntary}" "${soem_involuntary}"
    printf '| 脚本退出状态 | %s | %s |\n\n' "${igh_exit}" "${soem_exit}"

    printf '## 结论边界\n\n'
    printf -- '- 周期抖动和通信成功率可以在相同硬件、周期、运动与压力条件下'
    printf '横向比较。\n'
    printf -- '- IgH 资源数据主要对应单个应用通讯线程；SOEM 进程包含实时通讯'
    printf '线程和状态检查线程，CPU 占用不是完全相同的线程模型。\n'
    printf -- '- IgH 的 `CPU_ID` 绑定用户态通讯线程，`EtherCAT-OP` 仍是独立'
    printf '内核线程；SOEM 的 `CPU_ID` 由整个进程及其两个线程继承。\n'
    printf -- '- 两组测试应使用相同的 CPU 亲和性、压力参数、运动轨迹、时长和'
    printf ' `REQUIRE_ENDIO_OP`，否则只能作为观察数据。\n'
    printf -- '- 本报告只汇总单次结果。确定方案前应交错执行并至少重复3次。\n'
} >"${OUTPUT_PATH}"

printf 'comparison report: %s\n' "$(realpath "${OUTPUT_PATH}")"
