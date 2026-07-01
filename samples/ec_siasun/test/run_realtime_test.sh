#!/usr/bin/env bash

set -u
set -o pipefail

# TEST_BIN：被测 SOEM SIASUN 通讯测试程序路径。
TEST_BIN="${TEST_BIN:-../../../build/samples/ec_siasun/test/ec_siasun_communication_test}"
# IFNAME：SOEM 直接访问的 EtherCAT 网卡名称。
IFNAME="${IFNAME:-enp1s0}"
# AXIS_DIR：Axis1.xml-Axis6.xml 所在目录。
AXIS_DIR="${AXIS_DIR:-../doc/gcr10_1300}"
# DURATION_S：单次测试持续时间，单位秒。
DURATION_S="${DURATION_S:-60}"
# CPU_ID：SOEM 进程及其线程继承的 CPU 亲和性；-1 表示不绑定。
CPU_ID="${CPU_ID:-2}"
# POLICY：当前 SOEM 公共接口固定使用 fifo。
POLICY="${POLICY:-fifo}"
# PRIORITY：-1 表示 SOEM 公共接口使用 FIFO 最高优先级。
PRIORITY="${PRIORITY:--1}"
# MLOCK：当前 SOEM 公共接口固定开启内存锁定。
MLOCK="${MLOCK:-1}"
# DC_SYNC_CYCLES：当前 SOEM DC PI 同步固定每周期执行。
DC_SYNC_CYCLES="${DC_SYNC_CYCLES:-1}"
# ENABLE_MOTION：当前 SOEM demo 固定执行 Servo 6 老化运动。
ENABLE_MOTION="${ENABLE_MOTION:-1}"
# REQUIRE_ENDIO_OP：1 要求7站 OP；0 只判断前6个伺服。
REQUIRE_ENDIO_OP="${REQUIRE_ENDIO_OP:-0}"
# LOAD_PROFILE：压力类型，可选 idle、cpu、memory 或 mixed。
LOAD_PROFILE="${LOAD_PROFILE:-idle}"
# TEST_TAG：本轮测试编号。
TEST_TAG="${TEST_TAG:-manual}"
# LOAD_CPUSET：stress-ng 使用的 CPU 列表；空值表示由系统调度。
LOAD_CPUSET="${LOAD_CPUSET:-}"
# CPU_WORKERS：CPU 压力 worker 数量。
CPU_WORKERS="${CPU_WORKERS:-1}"
# CPU_LOAD：每个 CPU worker 的目标忙碌比例。
CPU_LOAD="${CPU_LOAD:-100}"
# VM_WORKERS：内存压力 worker 数量。
VM_WORKERS="${VM_WORKERS:-1}"
# VM_BYTES：每个内存压力 worker 使用的内存大小。
VM_BYTES="${VM_BYTES:-512M}"
# RESULT_DIR：本轮日志和环境快照目录。
RESULT_DIR="${RESULT_DIR:-results/$(date +%Y%m%d-%H%M%S)-${TEST_TAG}-${LOAD_PROFILE}}"
# STRESS_PID：后台 stress-ng 进程号。
STRESS_PID=""

# 结束测试时终止并等待后台压力进程。
cleanup() {
    if [[ -n "${STRESS_PID}" ]] &&
       kill -0 "${STRESS_PID}" 2>/dev/null; then
        kill "${STRESS_PID}" 2>/dev/null || true
        wait "${STRESS_PID}" 2>/dev/null || true
    fi
}

# 检查当前 SOEM demo 尚未开放为变量的固定测试条件。
validate_fixed_options() {
    if [[ "${POLICY}" != "fifo" ]] ||
       [[ "${PRIORITY}" != "-1" ]] ||
       [[ "${MLOCK}" != "1" ]] ||
       [[ "${DC_SYNC_CYCLES}" != "1" ]] ||
       [[ "${ENABLE_MOTION}" != "1" ]]; then
        printf '%s\n' \
            'SOEM demo currently requires POLICY=fifo PRIORITY=-1 MLOCK=1' \
            'DC_SYNC_CYCLES=1 ENABLE_MOTION=1' >&2
        exit 2
    fi

    if [[ "${REQUIRE_ENDIO_OP}" != "0" ]] &&
       [[ "${REQUIRE_ENDIO_OP}" != "1" ]]; then
        printf 'REQUIRE_ENDIO_OP must be 0 or 1\n' >&2
        exit 2
    fi
}

# 启动当前 LOAD_PROFILE 对应的 CPU 或内存压力。
start_stress() {
    # stress_command：按压力类型组成的 stress-ng 命令。
    local -a stress_command=(stress-ng --metrics-brief)

    case "${LOAD_PROFILE}" in
    idle)
        return
        ;;
    cpu)
        stress_command+=(--cpu "${CPU_WORKERS}" --cpu-load "${CPU_LOAD}")
        ;;
    memory)
        stress_command+=(--vm "${VM_WORKERS}" --vm-bytes "${VM_BYTES}"
            --vm-keep)
        ;;
    mixed)
        stress_command+=(--cpu "${CPU_WORKERS}" --cpu-load "${CPU_LOAD}"
            --vm "${VM_WORKERS}" --vm-bytes "${VM_BYTES}" --vm-keep)
        ;;
    *)
        printf 'invalid LOAD_PROFILE: %s\n' "${LOAD_PROFILE}" >&2
        exit 2
        ;;
    esac

    if ! command -v stress-ng >/dev/null 2>&1; then
        printf 'stress-ng is required for LOAD_PROFILE=%s\n' \
            "${LOAD_PROFILE}" >&2
        exit 2
    fi

    if [[ -n "${LOAD_CPUSET}" ]]; then
        stress_command=(taskset -c "${LOAD_CPUSET}" "${stress_command[@]}")
    fi

    "${stress_command[@]}" >"${RESULT_DIR}/stress.log" 2>&1 &
    STRESS_PID=$!
}

validate_fixed_options
trap cleanup EXIT INT TERM

mkdir -p "${RESULT_DIR}"

# environment_log：内核、CPU、网卡、压力配置和系统状态快照。
environment_log="${RESULT_DIR}/environment.log"
{
    printf 'date: '
    date --iso-8601=seconds
    uname -a
    printf '\ncmdline: '
    tr '\0' ' ' </proc/cmdline
    printf '\n\ncpu online: '
    cat /sys/devices/system/cpu/online
    printf '\nrealtime kernel flag: '
    cat /sys/kernel/realtime 2>/dev/null || printf 'not exposed'
    printf '\n\nSOEM interface:\n'
    ip -details link show "${IFNAME}" 2>&1 || true
    printf '\nSOEM interface driver:\n'
    ethtool -i "${IFNAME}" 2>&1 || true
    printf '\nEtherCAT related IRQs:\n'
    grep -Ei 'ethercat|ec_|eno|enp|eth' /proc/interrupts || true
    printf '\nsettings:\n'
    printf 'STACK=SOEM TEST_TAG=%s IFNAME=%s\n' "${TEST_TAG}" "${IFNAME}"
    printf 'DURATION_S=%s CPU_ID=%s POLICY=%s PRIORITY=%s MLOCK=%s\n' \
        "${DURATION_S}" "${CPU_ID}" "${POLICY}" "${PRIORITY}" "${MLOCK}"
    printf 'DC_SYNC_CYCLES=%s ENABLE_MOTION=%s REQUIRE_ENDIO_OP=%s\n' \
        "${DC_SYNC_CYCLES}" "${ENABLE_MOTION}" "${REQUIRE_ENDIO_OP}"
    printf 'LOAD_PROFILE=%s LOAD_CPUSET=%s\n' \
        "${LOAD_PROFILE}" "${LOAD_CPUSET}"
    printf 'CPU_WORKERS=%s CPU_LOAD=%s VM_WORKERS=%s VM_BYTES=%s\n' \
        "${CPU_WORKERS}" "${CPU_LOAD}" "${VM_WORKERS}" "${VM_BYTES}"
    printf '\nloadavg before: '
    cat /proc/loadavg
    printf '\nmemory before:\n'
    grep -E '^(MemTotal|MemAvailable|SwapTotal|SwapFree):' /proc/meminfo
    printf '\npressure before:\n'
    grep -H . /proc/pressure/cpu /proc/pressure/memory 2>/dev/null || true
} >"${environment_log}"

start_stress

# test_command：SOEM demo 的完整命令和位置参数。
test_command=("${TEST_BIN}" "${IFNAME}" "${AXIS_DIR}" "${DURATION_S}"
    "${REQUIRE_ENDIO_OP}")

if [[ "${CPU_ID}" -ge 0 ]]; then
    test_command=(taskset -c "${CPU_ID}" "${test_command[@]}")
fi

printf 'running:'
printf ' %q' "${test_command[@]}"
printf '\nresults: %s\n' "${RESULT_DIR}"

# time_command：优先使用 GNU time 记录进程整体资源统计。
time_command=()
if [[ -x /usr/bin/time ]]; then
    time_command=(/usr/bin/time -v)
fi

"${time_command[@]}" "${test_command[@]}" \
    > >(tee "${RESULT_DIR}/communication.log") \
    2> >(tee "${RESULT_DIR}/communication.stderr.log" >&2)

# test_status：SOEM 测试 demo 的退出状态。
test_status=$?
cleanup
trap - EXIT INT TERM

{
    printf '\nloadavg after: '
    cat /proc/loadavg
    printf '\nmemory after:\n'
    grep -E '^(MemTotal|MemAvailable|SwapTotal|SwapFree):' /proc/meminfo
    printf '\npressure after:\n'
    grep -H . /proc/pressure/cpu /proc/pressure/memory 2>/dev/null || true
} >>"${environment_log}"

printf 'test exit status: %d\n' "${test_status}" \
    | tee "${RESULT_DIR}/status.log"
exit "${test_status}"
