# Humble 完整回放验证（2026-11-26）

当前版本已通过两次官方 Retail_Street 完整 Humble 回放，包含实时前端、异步回环、
位姿图优化、关键帧轨迹发布和正常停止。两次前端结果与最终位姿快照一致。
与 Noetic 的对照存在数值差异，详见后面的边界说明。

本轮只验证并补充文档，没有修改生产 C++、参数、构建逻辑或回放工具。

## 环境与输入

- 源码基线 `9bf72a7`，x86_64、Ubuntu 22.04.5 容器、ROS2 Humble。
- GCC 11.4、Eigen 3.4.0、PCL 1.12.1、OpenCV 4.5.4、GTSAM 4.2.2。
- 启动前核对331个源码、配置和工具文件；运行时记录节点实际加载的25个相关库及其 SHA256。
- 复用已有 rosbag2 转换数据，重新校验数据库、元数据及转换摘要的文件哈希。
  转换摘要的原始 bag 哈希与本机 `Retail_Street.bag` 一致；未再复制或转换一份大数据。
- 前端预算2、后端线程池4，节点继承6个逻辑 CPU 的亲和性；OpenMP 可用。
  Gaussian 关闭。本次测试不覆盖 ARM、GPU 建图质量或长距离数据。

输入跨度为135.470252209秒，两次按1倍速完整发布。输入 QoS 为 reliable，
图像发布队列为32，观察端按相应深度接收。两个进程使用不同的 ROS domain。

## 实际结果

| 检查 | 首轮 | 重复回放 |
| --- | ---: | ---: |
| 相关 CTest 子集 | 9/9 | 9/9 |
| IMU / LiDAR / 图像输入 | 27447 / 1355 / 1355 | 相同 |
| LIO / VIO 更新 | 1352 / 1351 | 相同 |
| 发布里程计 / 点云 / RGB图像 | 1352 / 2703 / 1351 | 相同 |
| 轨迹行数 / 关键帧数 | 1352 / 169 | 相同 |
| 回环完成 / 接受 / 拒绝 | 13 / 2 / 11 | 相同 |
| 最终位姿版本 | 2 | 2 |
| 资源受限 / stale / skipped 查询 | 0 / 0 / 0 | 相同 |
| 后台 work payload / pending 峰值 | 1 / 1 | 相同 |
| 实际发布时长（秒） | 135.602 | 135.538 |
| 节点采样 RSS 峰值（MiB） | 900.99 | 896.45 |

两次订阅观察计数、节点输入计数均与30157条输入一致，DDS 序列化载荷哈希一致。
输出计数、位姿有限性、四元数规范化及正常退出检查均通过。
正式回环检查工具均返回 `consistent`，不存在未完成或失败归档。

重复回放的以下结果逐字节一致：

- `trajectory.tum`、`mat_pre.txt`、`mat_out.txt`、`imu.txt`、`odometry.txt`。
- 338份原始关键帧 PCD/PNG 的 SHA256，以及最终 `online_submaps/poses/2.yml`。

两次点云及图像的消息语义摘要逐项一致。归档中的8个中间文件哈希不同，包含候选耗时，
以及版本1提交时覆盖138/137帧的差异；该版本的优化前缀均为137帧。
异步工作完成时新增尾帧的数量可以不同。两份归档均独立校验通过，最终版本2一致；
本报告没有把全部中间元数据声明为逐字节相同。

## 与 Noetic 的对照边界

对照上一轮 Noetic 完整回放的1352行轨迹，时间戳逐行相同，不做轨迹对齐：

| 指标 | 差异 |
| --- | ---: |
| 位置差异 RMS | 9.385 mm |
| 位置差异最大值 | 49.896 mm |
| 最后一行位置差异 | 1.048 mm |
| 姿态角差异最大值 | 0.1304° |
| 关键帧数 | Humble 169，Noetic 172 |

180个公共生效参数中，相机、算法及线程预算一致；差异为输出目录、轨迹名称、传输 QoS。
Humble 两次回放重复一致，为本环境的结果稳定性提供证据；跨环境差异的原因尚未定位。
两端的编译器和依赖版本不同，尚未通过控制实验分离这些变量，不能直接归因于某个库。
这些数值是两条估计轨迹之间的差异，不是相对于真值的定位精度。

下一步可按传感器转换、预处理、IMU/LIO、VIO 的顺序定位首次差异，再决定是否需要
增加跨环境数值一致性的约束。当前验收不包含跨 ROS 逐位等价承诺。

## 产物与输入保护

统一入口先完成回放检查和回环归档检查，保存逐文件 SHA256 清单后，
每次精简338份本轮载荷，共216744280字节（约206.7 MiB）。轨迹、日志、元数据和检查结果保留。
精简后的关键帧目录不能继续用来读取原始点云；需要后续建图研究时请使用 `--retain all`。

原始 ROS1 bag、3份转换数据文件、331个源码文件、节点可执行文件和25个已加载库的
前后校验均通过。容器已恢复停止。两次均为独立进程，不能替代单进程耐久或长距离测试。

证据位于工作区 `validation/humble-full-20261126/`：

- `run.sh`、`repeat.sh` 和对应 driver 日志保存实际调用。
- `container-results/run/`、`container-results/repeat/` 保存统一报告、XML、回放日志、
  文件清单及回环检查结果。
- `source-before.json`、`container-results/preflight.json`、`runtime.json`（位于 container-results）
  保存来源、输入与实际加载信息。
- `verification.json`、`cross-ros-comparison.json` 保存最终核验和跨 ROS 差异。

## 在其他 Humble 环境执行

先按 [双 ROS 构建说明](ros-builds.md) 准备依赖、消息 underlay、构建目录和转换 bag。
在 livo-atlas 包目录执行，输出目录须不存在：

```bash
source /opt/ros/humble/setup.bash
source /path/to/message-underlay/install/setup.bash
build_dir=/path/to/configured/humble/build
bag_dir=/path/to/Retail_Street.ros2
export LD_LIBRARY_PATH="$build_dir:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$build_dir/rosidl_generator_py:${PYTHONPATH:-}"
export OMP_NUM_THREADS=1 OMP_DYNAMIC=FALSE OPENBLAS_NUM_THREADS=1
python3 tools/run_regression.py --ros-build "$build_dir" --jobs 2 \
  --replay ros2 --loop-checker "$build_dir/livo_atlas_check_loops" \
  --retain compact --output ../regression/humble-full \
  --replay-args --bag "$bag_dir" --executable "$build_dir/fastlivo_mapping" \
  --duration 136 --rate 1 --domain 93 --image-queue-size 32 \
  --loop-closure --keyframe-trajectory --stage-timing --default-threads
```

`--default-threads` 避免旧回放器为节点设置 `OMP_THREAD_LIMIT=1`；前端仍按 YAML 的
`frontend.openmp_threads` 使用预算。上述命令未限定 CTest 子集；本轮实际子集正则和
测试名记录于证据中的 `run/summary.json`。完整接口见 [工程回归工具](regression-tools.md)。
