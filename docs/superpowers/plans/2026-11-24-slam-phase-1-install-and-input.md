# SLAM 第一阶段实施计划

> 实施时按任务验证。已有用例覆盖相同行为时复用现有用例，缺失的故障用例写入现有测试文件。

这一阶段交付可安装的 SDK、明确的传感器模式，以及有上限的同步和 IMU 传播队列。独立 C++ 核心、Noetic 与 Humble 走同一套校验。 [总设计第一阶段](../specs/2026-11-24-slam-engineering-optimization-design.md#1-安装运行模式和输入上限)

执行时使用 C++17、项目现有的 GLOG 入口和 Google C++ 风格。函数定义之间留空行。运行参数从现有配置入口读取，回放和构建各在自己的工作目录执行。

## 实施前的基线

- [x] 保存实施前工作树的独立源码副本和构建参数，包含当前工作区已有迁移。用这个副本和修改后的副本配对回放。固定 bag、传感器参数、CPU 亲和性和 OpenMP 设置。
- [x] 记录安装失败的确切路径。当前 `cmake/core.cmake` 指向缺失的 `LICENSE.cartographer`。保留安装日志及当前 Noetic 的回放报告，便于分别判断安装和运行变化。 [安装规则](../../../cmake/core.cmake#L126) · [配对回放约定](../../global-pose-output.md#官方-bag-验证入口)

## 任务 1 恢复 SDK 安装

涉及 `docs/LICENSE.cartographer`、`cmake/core.cmake` 和 `docs/cartographer-scheduler.md`。

- [x] 从仓库中的 Cartographer 原始许可取出文本。

```bash
git show 805ecd2:third_party/cartographer/LICENSE > docs/LICENSE.cartographer
```

- [x] 更新 CMake 源路径和来源文档链接。安装位置为 `share/livo_atlas/licenses/cartographer/LICENSE`，用 `RENAME LICENSE` 保留安装文件名。
- [x] 重新生成独立核心构建目录，安装到新的临时前缀。比较原始文本与安装文本。从该前缀构建并运行已存在的安装消费测试。

```bash
cmake --install "$sdk_build" --prefix "$sdk_prefix"
cmp docs/LICENSE.cartographer "$sdk_prefix/share/livo_atlas/licenses/cartographer/LICENSE"
cmake -S tests/installed_admission_consumer -B "$consumer_build" \
  -DCMAKE_PREFIX_PATH="$sdk_prefix"
cmake --build "$consumer_build" --parallel 1
(cd "$consumer_build" && ctest --output-on-failure)
```

`sdk_build` 是按独立 SDK 文档配置并编译的目录。两次配置使用同一套 Sophus、GTSAM 4.2、PCL 和 glog 依赖路径。 [独立 SDK 复现步骤](../../core-build.md#普通-cmake-工程)

## 任务 2 从传感器配置派生运行模式

涉及 `include/livo_atlas/core/system_options.h`、`src/core/system_options.cc`、`src/core/livo_system.cc`、两端 `livo_node.cc` 和 `tests/core/livo_system_test.cc`。

- [x] 在现有系统测试中覆盖合法三种组合。LiDAR 开启且图像、IMU 均关闭得到 LO，开启 IMU 得到 LIO，再开启图像得到 LIVO。每种继续通过既有处理用例。
- [x] 覆盖表外组合。LiDAR 关闭，或图像开启同时 IMU 关闭时，直接构造 SDK 抛出 `std::invalid_argument`。检查输出目录尚未创建，已有哨兵文件内容保持原值。先复现当前 LIVO 错误组合被接受。
- [x] 添加 `FrontendMode SystemOptions::frontend_mode() const`，由现有三个传感器选项派生。由 `PrepareSystemOptions()` 在任何文件 I/O 之前调用。同步器、关键帧选择和 ROS 发布初始化从系统模式取值。导出器和着色器使用准备后的输出选项。
- [x] 在两端现有参数测试中加入错误组合，验证 ROS 参数加载和直接构造 SDK 的校验一致。使用 `common.lidar_en`、`imu.imu_en` 和 `common.img_en`。

## 任务 3 给缓冲区设限并定义超限结果

涉及 `system_options.{h,cc}`、`src/core/livo_config.cc`、`livo_system.{h,cc}`、`config/backend.yaml` 与现有系统、参数和停止测试。

- [x] 加入 `SensorBufferOptions` 和 `SystemOptions::sensor_buffer`。初始帧数是 LiDAR 64、IMU 4096、图像 32。初始时间跨度是 3.0 秒。用 `sensor_buffer.max_lidar_scans`、`max_imu_samples`、`max_images` 和 `max_span_s` 读取。数目须为正整数，跨度须为有限正数。参数测试验证合法覆盖值，准备阶段拒绝零、负数和非有限跨度。
- [x] 在已有系统测试里分别停止一条必需流，让其他传感器继续到达。将待测队列限额设为 2，在确认两条有效输入被接纳后验证第 3 条抛出 `std::overflow_error`。用可精确表示的间隔验证时间跨度刚好等于上限时接纳，第一条超出时拒收。等待且输入暂停时维持 `SyncStatus::kWaiting`。
- [x] 在追加前检查拟保留的队列。LiDAR 按现有时间回退清空规则确定拟保留范围。IMU 在完成配置偏移和驱动时间修正后检查同步队列与传播队列。图像在完成现有采样频率和时间过滤后检查。条数与同流校正时间跨度分别判断。
- [x] 根据最近一次估计时间裁掉传播缓存中已覆盖的 IMU，保持现有严格小于估计时刻的裁剪边界。对剩余需要重放的 IMU 计入限额，用初始传播状态和已更新传播状态各验证一次。
- [x] 在 `LivoSystem` 锁存首个超限异常。异常文本记录队列名、占用、限制和最老待处理样本。后续输入、推进及显式结束报告同一故障。沿原失败收尾关闭线程和文件，已提交视图仍可检查，失败会话保持未完成。
- [x] 在首个有效 LiDAR 到达前，启动队列采用原启动计数上限与配置上限中的较小值。首扫交接用其时戳作为驱动修正锚点，在临时缓冲中整理有效样本并选最新的有界后缀，随后交付队列与首扫游标。已有启动计数分别记录淘汰和无效样本。
- [x] 验证首扫缺席时的有界缓存、覆盖超过 3 秒的启动历史、带 `driver_fix` 的 IMU 修正、负时间 LiDAR 样本与无效扫描，以及限制内和参考后缀的数值等价。复用现有启动顺序、缓冲上限和驱动修正用例。 [当前输入实现](../../../src/core/livo_system.cc#L89) · [既有启动测试](../../../tests/core/livo_system_test.cc#L349)

## 任务 4 验证两端节点和实际输入

- [x] 在独立 SDK 构建中运行 `livo_system_test`、`synchronizer_test` 和 `input_seal_test`。同时运行已有回环测试，覆盖正常结束和故障后读取已提交前沿。

```bash
cmake --build "$sdk_build" --parallel 1 --target livo_system_test synchronizer_test \
  input_seal_test loop_backend_test
(cd "$sdk_build" && ctest --output-on-failure \
  -R '(livo_system|synchronizer|input_seal|loop_backend)_test$')
```

- [x] 在 Noetic 运行现有 `lidar_callbacks.test`。在 Humble 运行 `ros2_node_test`。两端校验参数类型、无效模式、限额以及暂停时的等待行为。节点运行期超限以独立进程验证，检查退出码、错误记录和归档状态。
- [x] 先跑 10 秒，再在同机对照完整 `Retail_Street.bag`。至少覆盖纯 LIVO 与带回环优化的完整配置。配对检查输入计数、LIO/VIO 次数、`trajectory.tum`、`mat_pre.txt`、`mat_out.txt`、`imu.txt`、原始关键帧以及现有 ROS 输出语义。保存队列故障日志与回环提交状态。
- [x] 正常输入在初始限额内完成回放。按实测峰值校准限额并记录依据。下一条输入将使队列超限时按策略停止，已提交前沿可读。启用回环归档时，用 `livo_atlas_check_loops` 核对故障会话的未完成状态。 [双 ROS 验证入口](../../ros-builds.md) · [回环归档检查](../../online-loop-backend.md#文件与只读检查)

Noetic 的完整 SLAM 回放使用下面的入口。基线和改动侧各用自己的源码副本、二进制和新输出目录。用 `playback_duration=10` 做短段验证，用 136 秒请求读完整包。纯 LIVO 运行省略 `--loop-closure`。

```bash
OMP_THREAD_LIMIT=1 nice -n 10 python3 tools/run_baseline.py \
  --workspace "$snapshot_workspace" --bag "$retail_bag" \
  --output "$run_output" --duration "$playback_duration" --rate 1 \
  --cpu-affinity "$test_cpu" --image-queue-size 32 --semantic-output --loop-closure
```

这一阶段的结果是可单独验收的安装与输入变更。后续的计时、ROS 调度和异步位姿图迁移按总设计逐批实施。

## 实施与验收记录

- 独立 SDK 的 12 组 CTest 与安装消费用例通过。已安装许可与 Cartographer 原始文件相同，旧的位置式 `SystemOptions` 初始化也通过外部消费验证。
- Noetic 的 40 项节点测试与 Humble 的 6 组 CTest 通过。Humble 另外回放转换后的 10 秒 bag，收到完整输入并完成 98 次 LIO、97 次 VIO。
- Noetic 以 1× 回放速度对照完整包。纯 LIVO 和带回环优化两种配置均得到 1,352 次 LIO、1,351 次 VIO。前端轨迹及三个日志文件逐字节一致，ROS 点云、图像和里程计载荷一致。完整 SLAM 的 172 个原始关键帧逐文件一致，接受 2 次回环并提交位姿版本 2。两份归档都通过一致性检查。
- 两端先回放正常输入，再暂停并中断 IMU 话题。LiDAR 队列占满配置的 4 扫后，下一扫触发退出码 1。已写入关键帧可读，`livo_atlas_check_loops` 返回 `incomplete`，故障信息与节点日志一致。

完整 SLAM 的同步队列峰值由验证进程记录。维持初始配置，为不同机器的调度和输入抖动留出余量。IMU 传播限额由核心回归用例覆盖。

| 同步队列 | 峰值条数 | 峰值传感器跨度 |
| --- | ---: | ---: |
| LiDAR | 2 | 0.100 s |
| IMU | 47 | 0.220 s |
| 图像 | 3 | 0.201 s |

本机源码与运行库快照、配对结果 `paired-replay-results.json`、断流归档检查和测试日志位于工作区的 `validation/slam-input-20261124/`。
