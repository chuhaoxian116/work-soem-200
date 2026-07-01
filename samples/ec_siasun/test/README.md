# SOEM SIASUN 通讯测试

`ec_siasun_communication_test` 复用上层 `src` 和 `include` 中的 Axis XML、
SDO、PDO、DC、Servo 6 老化运动和通信质量统计实现。

测试 demo 默认只要求 Servo 1-6 进入 OP。程序仍会发现 EndIO 7 并尝试
配置它，但6轴模式使用独立过程数据组，EndIO 缺失或不能进入 OP 时不会
阻止通信统计。正式 `ec_siasun` 程序仍默认要求6个伺服和 EndIO 共7站
全部进入 OP。

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

- `require_endio_op=0`：只要求 Servo 1-6 进入 OP，WKC 不低于6轴期望值。
- `require_endio_op=1`：要求 Servo 1-6 和 EndIO 7 全部进入 OP。

开始测试前应停止 IgH Master，确保网卡没有被 `ec_igc` 占用，并确认 Servo
6 老化运动的方向、范围、机械空间和急停状态安全。

## 压力测试脚本

`run_realtime_test.sh` 的结果目录和主要变量与 IgH 测试脚本保持一致。
SOEM 当前固定使用 FIFO 最高优先级、mlock、每周期 DC PI 同步和 Servo 6
运动；脚本会拒绝改变这些固定条件，避免产生不可比的数据。

空载基线：

```sh
cd /home/js/ETCAT/soem/SOEM-2.0.0/samples/ec_siasun/test

sudo env \
    TEST_TAG=T01-SOEM \
    TEST_BIN=../../../build/samples/ec_siasun/test/ec_siasun_communication_test \
    IFNAME=enp2s0 \
    AXIS_DIR=../doc/gcr10_1300 \
    DURATION_S=1800 CPU_ID=2 \
    REQUIRE_ENDIO_OP=0 LOAD_PROFILE=idle \
    ./run_realtime_test.sh
```

CPU 压力：

```sh
sudo env \
    TEST_TAG=T03-SOEM \
    TEST_BIN=../../../build/samples/ec_siasun/test/ec_siasun_communication_test \
    IFNAME=enp2s0 \
    AXIS_DIR=../doc/gcr10_1300 \
    DURATION_S=1800 CPU_ID=2 \
    REQUIRE_ENDIO_OP=0 LOAD_PROFILE=cpu LOAD_CPUSET=1,5 \
    CPU_WORKERS=2 CPU_LOAD=100 \
    ./run_realtime_test.sh
```

混合压力：

```sh
sudo env \
    TEST_TAG=T07-SOEM \
    TEST_BIN=../../../build/samples/ec_siasun/test/ec_siasun_communication_test \
    IFNAME=enp2s0 \
    AXIS_DIR=../doc/gcr10_1300 \
    DURATION_S=1800 CPU_ID=2 \
    REQUIRE_ENDIO_OP=0 LOAD_PROFILE=mixed LOAD_CPUSET=1,5 \
    CPU_WORKERS=2 CPU_LOAD=100 VM_WORKERS=1 VM_BYTES=8G \
    ./run_realtime_test.sh
```

每轮结果包含：

```text
communication.log
communication.stderr.log
environment.log
stress.log              # 仅压力场景
status.log
```

## IgH/SOEM 对比报告

两组测试必须使用相同的时长、APP CPU、压力参数、运动轨迹和
`REQUIRE_ENDIO_OP`。生成 Markdown 报告：

```sh
./generate_comparison_report.sh \
    /home/js/ETCAT/igh/myproject/SIASUN/test/results/<IgH结果目录> \
    ./results/<SOEM结果目录> \
    ./results/igh-soem-comparison.md
```

报告会对比周期抖动、WKC、成功率、DC、CPU 和内存。IgH 的 DC sync
monitor 与 SOEM 的 DC PI 相位误差不是同一个物理量，报告会保留两组原始
数据并标注该限制，不直接用数值大小判断哪一个主站更好。
