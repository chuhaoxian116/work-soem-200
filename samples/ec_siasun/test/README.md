# SOEM SIASUN 通讯测试

`ec_siasun_communication_test` 复用上层 `src` 和 `include` 中的 Axis XML、
SDO、PDO、DC、Servo 6 老化运动和通信质量统计实现。

测试 demo 默认允许现场只有 Servo 1-6。除“最低从站数量”外，状态切换、
PDO 映射、DC 配置、WKC 和周期通讯全部沿用正式程序原版流程；发现 EndIO
时也仍按原版处理。正式 `ec_siasun` 程序默认要求6个伺服和 EndIO 共7站。

## 编译

SOEM 2.0.0 当前要求 CMake 3.28 或更高版本。

```sh
cd /home/js/ETCAT/soem/SOEM-2.0.0
cmake -S . -B build -DSOEM_BUILD_SAMPLES=ON
cmake --build build --target ec_siasun_communication_test
```

## 运行

```sh
sudo ./build/samples/ec_siasun/test/ec_siasun_communication_test \
    enp2s0 \
    ./samples/ec_siasun/doc/gcr10_1300 \
    1800 \
    0
```

位置参数：

```text
ifname AxisXmlDirectory duration_s require_endio_op
```

- `require_endio_op=0`：最低允许只发现 Servo 1-6。
- `require_endio_op=1`：最低要求 Servo 1-6 和 EndIO 7 共7站。

开始测试前应停止 IgH Master，确保网卡没有被 `ec_igc` 占用，并确认 Servo
6 老化运动的方向、范围、机械空间和急停状态安全。

## IgH/SOEM 对照原则

IgH 与 SOEM 不能同时占用同一块 EtherCAT 网卡，测试采用相同硬件上的
顺序 A/B 对照。每一对测试必须保持以下条件一致：

```text
内核和 BIOS 设置、网卡和从站拓扑、1 ms 周期、测试时长
APP CPU、压力 CPU 和压力参数、Servo 6 轨迹、REQUIRE_ENDIO_OP
```

SOEM 当前固定使用 FIFO 最高优先级、mlock、每周期 DC PI 同步和 Servo 6
运动。`run_realtime_test.sh` 会拒绝改变这些固定条件。IgH 对照组也必须
设置 `POLICY=fifo PRIORITY=-1 MLOCK=1 DC_SYNC_CYCLES=1
ENABLE_MOTION=1`。

建议同一编号交替执行 IgH 和 SOEM，并在重复测试时交换先后顺序，降低温度、
后台任务和系统运行时间造成的顺序偏差。两个主站使用相同 `TEST_TAG`，结果
分别保存在各自的 `test/results` 目录。

## 运动和通讯判定

对照组固定复用 `app_config.h` 中的 Servo 6 老化轨迹：

```text
模式：CSP（模式 8）
幅值：100000 pulse
峰峰值：200000 pulse
周期：20 s
渐入时间：2 s
```

其它 5 个伺服和 EndIO 输出保持为 0。开始正式测试前先运行 T01 冒烟测试，
确认运动方向、机械空间和急停状态安全。

现场没有 EndIO 时，IgH 和 SOEM 都设置 `REQUIRE_ENDIO_OP=0`。如果 SOEM
发现了 EndIO，仍按原版把它纳入状态、PDO 和 WKC；该开关不改变已有从站的
通讯流程。EndIO 恢复后，两边同时改为 `REQUIRE_ENDIO_OP=1`。

## 测试前准备

SOEM 运行前停止 IgH 正式程序和 Master，确认网卡已由普通 Linux 网卡驱动
管理、接口名存在，并且没有其它程序占用：

```sh
sudo pkill -INT igh_master_sinsun
ip -details link show enp2s0
ethtool -i enp2s0
lscpu -e=CPU,CORE,SOCKET,NODE,ONLINE
cat /sys/devices/system/cpu/cpu*/topology/thread_siblings_list | sort -u
```

IgH 使用 `ec_igc`，SOEM 使用普通网卡接口；切换主站时需要按目标机现有
部署方式切换驱动或重启。网卡驱动本身属于两套主站栈的一部分，必须在报告
中保留 `environment.log`，不能把驱动差异误写成纯用户态程序差异。

以下命令按当前 4 核 8 线程目标机编写：

```text
CPU 0、4：系统和监控
CPU 1、5：CPU/内存压力
CPU 2、6：SOEM APP；APP 绑定 CPU 2，CPU 6 保持空闲
CPU 3、7：保留
```

如果目标机 CPU 拓扑不同，应先修改表格和命令中的 CPU 编号。

## 压力测试脚本

进入目录并定义公共参数：

```sh
cd /home/js/ETCAT/soem/SOEM-2.0.0/samples/ec_siasun/test

export TEST_BIN=../../../build/samples/ec_siasun/test/ec_siasun_communication_test
export IFNAME=enp1s0
export AXIS_DIR=../doc/gcr10_1300
export REQUIRE_ENDIO_OP=0
```

`LOAD_PROFILE` 支持 `idle`、`cpu`、`memory` 和 `mixed`，后三项依赖
`stress-ng`。先把 `DURATION_S` 设为 600 做冒烟测试；正式测试每组运行
1800 秒并至少重复 3 次。除了表中变量，其它参数保持不变。

## 与 IgH 共用的测试矩阵

| 编号 | APP CPU | 压力类型 | 压力 CPU | 压力参数 | DC 设置 | 时长 | 目的 |
|---|---:|---|---|---|---:|---:|---|
| T00 | 不绑定 | idle | - | - | 1 | 30 min | 保存未绑核基线 |
| T01 | CPU 2 | idle | - | CPU 6 保持空闲 | 1 | 30 min | 比较 APP 绑核收益 |
| T02 | CPU 2 | cpu | CPU 1、5 | 2 workers / 50% | 1 | 30 min | 其它物理核普通负载 |
| T03 | CPU 2 | cpu | CPU 1、5 | 2 workers / 100% | 1 | 30 min | 其它物理核满载 |
| T04A | CPU 2 | cpu | CPU 2 | 1 worker / 100% | 1 | 30 min | 同一逻辑 CPU 竞争 |
| T04B | CPU 2 | cpu | CPU 6 | 1 worker / 100% | 1 | 30 min | SMT 兄弟线程竞争 |
| T05 | CPU 2 | memory | CPU 1、5 | 1 worker / 8G | 1 | 30 min | 内存带宽和回收压力 |
| T06 | CPU 2 | memory | CPU 1、5 | 1 worker / 16G | 1 | 30 min | 约 50% 物理内存压力 |
| T07 | CPU 2 | mixed | CPU 1、5 | CPU 100% + 8G | 1 | 30 min | 混合高负载 |
| T10 | CPU 2 | mixed | CPU 2 | CPU 100% + 16G | 1 | 30 min | APP 同核混合竞争 |
| T11 | 最优配置 | mixed | 其它核心 | CPU 100% + 8G | 1 | 8 h | 长时间稳定性 |

T06 和 T10 运行前先确认可用内存，避免 OOM 终止测试或影响系统服务。

## 测试命令

T00 未绑核基线：

```sh
sudo env TEST_TAG=T00 TEST_BIN="$TEST_BIN" IFNAME="$IFNAME" \
    AXIS_DIR="$AXIS_DIR" DURATION_S=1800 CPU_ID=-1 \
    REQUIRE_ENDIO_OP="$REQUIRE_ENDIO_OP" LOAD_PROFILE=idle \
    ./run_realtime_test.sh
```

T01 绑定 APP CPU：

```sh
sudo env TEST_TAG=T01 TEST_BIN="$TEST_BIN" IFNAME="$IFNAME" \
    AXIS_DIR="$AXIS_DIR" DURATION_S=1800 CPU_ID=2 \
    REQUIRE_ENDIO_OP="$REQUIRE_ENDIO_OP" LOAD_PROFILE=idle \
    ./run_realtime_test.sh
```

T02/T03 只改变 `TEST_TAG` 和 `CPU_LOAD`：

```sh
sudo env TEST_TAG=T03 TEST_BIN="$TEST_BIN" IFNAME="$IFNAME" \
    AXIS_DIR="$AXIS_DIR" DURATION_S=1800 CPU_ID=2 \
    REQUIRE_ENDIO_OP="$REQUIRE_ENDIO_OP" \
    LOAD_PROFILE=cpu LOAD_CPUSET=1,5 \
    CPU_WORKERS=2 CPU_LOAD=100 \
    ./run_realtime_test.sh
```

T04A 同一逻辑 CPU 竞争：

```sh
sudo env TEST_TAG=T04A TEST_BIN="$TEST_BIN" IFNAME="$IFNAME" \
    AXIS_DIR="$AXIS_DIR" DURATION_S=1800 CPU_ID=2 \
    REQUIRE_ENDIO_OP="$REQUIRE_ENDIO_OP" \
    LOAD_PROFILE=cpu LOAD_CPUSET=2 CPU_WORKERS=1 CPU_LOAD=100 \
    ./run_realtime_test.sh
```

T04B SMT 兄弟线程竞争：

```sh
sudo env TEST_TAG=T04B TEST_BIN="$TEST_BIN" IFNAME="$IFNAME" \
    AXIS_DIR="$AXIS_DIR" DURATION_S=1800 CPU_ID=2 \
    REQUIRE_ENDIO_OP="$REQUIRE_ENDIO_OP" \
    LOAD_PROFILE=cpu LOAD_CPUSET=6 CPU_WORKERS=1 CPU_LOAD=100 \
    ./run_realtime_test.sh
```

T05/T06 只改变 `TEST_TAG` 和 `VM_BYTES`：

```sh
sudo env TEST_TAG=T05 TEST_BIN="$TEST_BIN" IFNAME="$IFNAME" \
    AXIS_DIR="$AXIS_DIR" DURATION_S=1800 CPU_ID=2 \
    REQUIRE_ENDIO_OP="$REQUIRE_ENDIO_OP" \
    LOAD_PROFILE=memory LOAD_CPUSET=1,5 VM_WORKERS=1 VM_BYTES=8G \
    ./run_realtime_test.sh
```

T07 混合压力：

```sh
sudo env TEST_TAG=T07 TEST_BIN="$TEST_BIN" IFNAME="$IFNAME" \
    AXIS_DIR="$AXIS_DIR" DURATION_S=1800 CPU_ID=2 \
    REQUIRE_ENDIO_OP="$REQUIRE_ENDIO_OP" \
    LOAD_PROFILE=mixed LOAD_CPUSET=1,5 \
    CPU_WORKERS=2 CPU_LOAD=100 VM_WORKERS=1 VM_BYTES=8G \
    ./run_realtime_test.sh
```

T10 APP 同核混合压力：

```sh
sudo env TEST_TAG=T10 TEST_BIN="$TEST_BIN" IFNAME="$IFNAME" \
    AXIS_DIR="$AXIS_DIR" DURATION_S=1800 CPU_ID=2 \
    REQUIRE_ENDIO_OP="$REQUIRE_ENDIO_OP" \
    LOAD_PROFILE=mixed LOAD_CPUSET=2 \
    CPU_WORKERS=1 CPU_LOAD=100 VM_WORKERS=1 VM_BYTES=16G \
    ./run_realtime_test.sh
```

T11 应在 T00-T10 完成后，将 `CPU_ID` 和压力 CPU 换成前面测试得到的最优
配置，并设置 `DURATION_S=28800`。

## 不直接纳入 A/B 的测试

IgH 的 T08/T09 改变 DC 同步报文发送间隔；SOEM 当前每周期读取 `DCtime`
并执行本地 PI 相位修正。这两个量的控制路径不同，不能把
`DC_SYNC_CYCLES=2/5` 机械迁移到 SOEM 后直接排名。

IgH 的 O00-O03 测试独立内核线程 `EtherCAT-OP`。SOEM 没有该线程，其 RT
通讯线程和状态检查线程都属于当前进程并继承 `taskset`，因此 O00-O03 也不
作为同名对照项。若要优化 SOEM 双线程，应另建 SOEM 专用矩阵，避免与 IgH
线程模型混淆。

## 每轮结果和失败条件

每轮结果保存在 `results/<时间>-<TEST_TAG>-<压力类型>/`：

```text
communication.log
communication.stderr.log
environment.log
stress.log              # 仅压力场景
status.log
```

每轮首先确认：

```text
[RT] actual policy=1 priority=99 cpu=<目标 CPU>
communication judgment: <与 IgH 相同的判定对象>
test exit status: 0
```

硬性失败条件：

```text
严重超周期次数 > 0
不完整周期或无过程数据周期 > 0
OP 后 major fault > 0
从站掉线、AL 状态异常、恢复事件或程序非零退出
```

各组都没有硬性失败时，再比较最大抖动、DC 最大误差、CPU 占用、峰值 RSS
和上下文切换次数。

## IgH/SOEM 对比报告

为每个相同编号分别生成报告：

```sh
./generate_comparison_report.sh \
    /home/js/ETCAT/igh/myproject/SIASUN/test/results/<IgH结果目录> \
    ./results/<SOEM结果目录> \
    ./results/<编号>-igh-soem-comparison.md
```

报告会对比周期抖动、WKC、成功率、DC、CPU 和内存。IgH 的 DC sync
monitor 与 SOEM 的 DC PI 相位误差不是同一个物理量，报告会保留两组原始
数据并标注该限制，不直接用数值大小判断哪一个主站更好。
