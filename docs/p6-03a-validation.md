# P6-03a 重建观测验证

本轮实现从 `51d8ac7` 开始，随后修正了公开相机内参的像素中心约定；设计及实施计划分别见
[观测准备设计](superpowers/specs/2026-11-15-p6-03a-reconstruction-view-design.md)和
[实施计划](superpowers/plans/2026-11-15-p6-03a-reconstruction-view.md)。日志、隔离构建、独立
消费者、输入哈希与对照数据位于工作区包外 `validation/P6-03a-20261115/`；设计阶段的投影
探针和旧 COLMAP 参考位于 `validation/P6-03-design-20261115/`。它们不随源码仓库提交。

## 核心与安装消费

先在实现前运行已有 COLMAP 相机测试，11/11 通过；新增视图测试因公共头不存在而按预期编译失败，
随后实现并通过新旧相机用例。缺少点云的错误消息及公开 `CameraConfig` 的半像素约定
各有单独的失败复现和修复验证；最终相机与视图测试为 19/19。半像素用例通过现有
`PinholeCamera` 投影验证相机模型和深度数组的索引一致。
独立 SDK 全部 60 项 CTest 通过，新增头参加了单头编译检查。

本机 CTest 3.16.3 不支持 `--test-dir`，第一次误用该参数时在源码目录报告“没有测试”且返回 0；
那次输出与生成目录已保存，未计为通过。随后用 `cmake -E chdir <构建目录> ctest ...` 实际运行
了完整测试。Google 风格格式检查保留 include 分组，并检查修改的函数定义之间留有空行。

安装消费者先针对旧 SDK 验证新头缺失，再从新安装前缀调用原始与优化位姿的视图；相机变换、
4 米深度、掩码、图像所有权和既有 LO/LIO/VIO、COLMAP、在线子地图及几何消费均通过。
实际链接路径指向本轮 `sdk-install-final/lib`，不是原构建树；消费者也验证公开内参的像素中心。

## Retail_Street 数据

使用上一阶段 Noetic 默认启动缓冲、完整官方 bag 产生的 172 帧关键帧数据集，版本 0 为原始
里程计位姿。一个只链接已安装 `livo_atlas::io` 的独立 C++ 程序逐帧生成视图，并与实施前冻结的
Python 投影结果比较：172 帧的有效图像和深度像素数量全部相同；前三帧的去畸变图像、图像掩码和深度
掩码逐像素一致，有效深度最大差约 1.89e-6 米，来自 float32 表示。实施前 Python 投影还与
现有 COLMAP 导出器对前三帧独立核对过像素集合及相机 z 深度。

图像尺寸 640×512，去畸变有效区域 324,038 像素。单帧有效深度中位数 13,419 像素，约占该
区域的 4.14%；没有观测的像素不参与深度监督。只读探针验证了 517 个原始文件的 SHA-256，
没有修改关键帧资产。这个比较没有逐像素覆盖全部 172 帧，也不是深度真值验证。

单逻辑 CPU、OpenCV 单线程、`OMP_THREAD_LIMIT=1` 的一次本机检查中，172 帧含文件读取的总
时间约 1.92 秒，单次视图转换中位数约 4.40 ms、P95 约 4.80 ms，进程峰值 RSS 约 84.1 MiB。
这些数字只反映该数据集和 CPU 准备程序，不代表 ROS 端到端延迟、GPU 显存或在线训练速度。

## 双 ROS 与边界

本机主机为 Ubuntu 20.04 / Noetic，Humble 位于 Ubuntu 22.04 容器。两端使用独立源码副本和
构建目录，应用相同的普通 C++ 接口；均构建了 `fastlivo_mapping` 并通过 19/19 项相机与
重建视图测试。两端节点的动态链接检查均未发现 vikit、CUDA、Torch 或缺失库。516 个源码
快照文件的哈希与测试日志保存在证据目录中；这些是构建和单元测试，不是本轮 ROS 回放。
主机 GPU 为 RTX 4060 Laptop、8,188 MiB 显存；本轮没有安装 CUDA 编译器或训练依赖，
现有 Humble 容器也没有分配 GPU。

本阶段不修改 ROS 输入、前端算法或已有 COLMAP 导出路径，没有重复完整 bag 回放。它也没有
安装渲染器或执行高斯训练；未来 mapper 仍需实现活动工作集、视图丢弃与可靠位姿修正、过期
训练结果拒绝及 GPU 资源管理。绝对定位误差、深度真值和跨机器逐位一致性不在本轮结论范围内。

## 跨电脑复现

按[独立核心说明](core-build.md)安装本项目 SDK；使用[双 ROS 构建说明](ros-builds.md)分别
构建 Noetic 和 Humble。示例安装消费者位于 `tests/installed_consumer`。例如：

```bash
cmake -S tests/installed_consumer -B /path/to/view-consumer-build \
  -DCMAKE_PREFIX_PATH=/path/to/livo-atlas-sdk
nice -n 10 cmake --build /path/to/view-consumer-build -- -j1
OMP_THREAD_LIMIT=1 /path/to/view-consumer-build/installed_consumer
```

若 SDK 构建开启 GTSAM，还需为消费者提供该平台实际的 GTSAM 配置路径。不要复制旧机器上的
CMake 缓存直接使用；跨平台应重新编译。真实数据可由新的 Retail_Street 回放生成，或单独复制
上面两个 `validation/` 目录与关键帧数据集；复制后对照各目录的 SHA-256。证据里的独立程序
接受数据集路径和新的输出目录；复制数据后，可用 `compare_real_views.py --dataset <新路径> --output <新报告路径>`
重定向数据集并避免覆盖原报告。改变输入或平台时应重新生成对照，不把本机耗时或精确数值直接套用到新机器。

复制两份证据目录后，如需对同一关键帧资产复跑实际 SDK 消费，可设置 `LIVO_EVIDENCE` 为包含
两个阶段目录的 `validation` 路径、`LIVO_DATASET` 为复制后的 keyframes 目录、`LIVO_SDK`
为新机器的安装前缀，`LIVO_RUN` 为新的空输出目录。例如：

```bash
mkdir -p "$LIVO_RUN"
cmake -S "$LIVO_EVIDENCE/P6-03a-20261115/real-consumer" \
  -B "$LIVO_RUN/build" -DCMAKE_PREFIX_PATH="$LIVO_SDK"
nice -n 10 cmake --build "$LIVO_RUN/build" -- -j1
OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1 nice -n 10 \
  "$LIVO_RUN/build/real_view_consumer" "$LIVO_DATASET" "$LIVO_RUN/views"
/usr/bin/python3 "$LIVO_EVIDENCE/P6-03a-20261115/compare_real_views.py" \
  --dataset "$LIVO_DATASET" --views "$LIVO_RUN/views" \
  --output "$LIVO_RUN/comparison.json"
```

回放或重生成了不同的关键帧数据时，应先用 `P6-03-design-20261115/probe_dataset.py` 对新数据
生成新的输入清单和参考统计；旧参考只能用于确认同一批资产。跨平台结果若有数值差异，应先
核对源码、标定、数据哈希及浮点环境，再判断原因。
