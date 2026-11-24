# 阶段 4 命名收敛与高斯联合验证实施计划

> 执行时使用 `superpowers:executing-plans`，按下列检查项记录进度。

目标是落实已确定的 `GaussianMapping` 命名及位姿访问语义，用完整回放验收异步位姿图与高斯建图的交接。依据为 [工程优化设计](../specs/2026-11-24-slam-engineering-optimization-design.md#4-随职责迁移的命名)。

## 约束与接口

沿用当前开发分支，只修改 `livo-atlas`。C++ 使用 Google 风格，函数定义之间空一行。
`GaussianMapping` 位于应用层，组织 `OnlineGaussianMapper`、ROS 预览发布、轮询和封存。平台能力宏使用 `LIVO_ATLAS_HAS_LINUX_GAUSSIAN_BRIDGE`，由 CMake 的平台构建结果给出。运行开关使用 `config/gaussian.yaml` 中的 `gaussian.enabled`。

`KeyframePose::parent_from_body() const` 返回 `const RigidTransform&`。所属坐标系取自包含它的 `PoseSnapshot::parent_frame`。现有聚合初始化、公开字段 `map_from_body` 和存档键继续供已有调用方使用，内部读取迁移到访问器。

## 任务 1 命名和位姿访问

涉及 `src/app/online_gaussian.{h,cc}` 到 `gaussian_mapping.{h,cc}` 的迁移、`src/app/livo_main.cc`、`cmake/node.cmake`、位姿头文件及使用它的内部读取代码。更新当前架构文档和节点装配链接。

- [x] 在现有 `pose_snapshot_test` 中覆盖访问器对原字段的引用、版本 0 与优化后坐标，以及聚合初始化兼容性。先确认缺少访问器导致编译失败。
- [x] 实施命名和访问器迁移，保持序列化字段与数值计算一致。在安装后消费者中使用访问器读取优化尾帧。
- [x] 构建独立 SDK、Noetic 和 Humble 共用节点，运行相关已有测试及安装后消费者。另编译无 Linux 桥接能力的应用分支，验证 `gaussian.enabled` 的平台检查。

## 任务 2 完整联合验证

工作区证据使用 `validation/slam-naming-gaussian-20261124/`。冻结修改前的节点和动态库，两侧使用同一份 bag、配置、CPU 亲和性与 OpenMP 预算。

- [x] 执行修改前后各 10 秒及完整 `Retail_Street.bag` 回放。完整回放开启回环、持久高斯子地图、联合渲染、ROS 预览与最终封存。
- [x] 比较前端轨迹、原始关键帧和成员修订。核对正式位姿、回环接纳、预览版本、联合渲染来源及最终地图覆盖范围。
- [x] 检查高斯子地图刚性修正复用与相对位姿变化重建。结合实际回环和现有确定性测试验证两条路径，并核对 worker 正常退出。
- [x] 运行现有高斯故障隔离测试，确认输出失败的状态记录与 SLAM 归档检查结果。
- [x] 进行一次独立代码复核并记录验证结果。

## 复核重点

类名替换应准确覆盖应用类及 CMake 源文件。平台能力宏应覆盖 Linux 实现与备用实现。访问器应返回原位姿的常量引用，旧存档字段应可读写。异步输出按实际位姿版本和覆盖帧数比对，必要的终止交付应在完成摘要之前结束。验证目录遵守本机空间限制。

## 验证记录

- 独立 SDK 的 76 个 CTest 目标和 4 个安装后消费者通过。Noetic 与 Humble 各通过 8 个相关 CTest 目标，涵盖位姿访问器、回环、高斯建图、联合渲染和封存。无桥接应用分支在本机单独编译，配置关闭时通过，启用时报告 Linux 平台要求。
- 两侧完整回放均输出 1352 次 LIO、1351 次 VIO，保存 172 个关键帧。轨迹、原始关键帧及成员修订一致。13 次回环查询全部完成，接纳 2 次，最终位姿最大绝对差约为 `2.16e-15`。两侧归档检查返回 0。
- 两侧最终高斯地图均有 13 个已验证且可用的持久子地图，覆盖位姿版本 2 和 172 帧。修改后在播放期间收到 73 份有效联合渲染预览，其中 61 份包含两个子地图。首次预览约在播放后 4.23 秒到达，退出时封存成功。
- 最终关键帧轨迹一致，11 份相同版本和覆盖范围的 ROS 几何地图一致。修改后 ROS 录制端最后收到的地图覆盖版本 1 和 170 帧，几何输出的结束统计覆盖版本 2 和 172 帧。在版本 2 和 172 帧上用最终归档重建几何地图，两侧 100000 个 XYZI 点一致，位置差为 0。停机期间的发布与归档按各自覆盖范围核对。
- 用最终高斯资产进行 GPU 冷加载，13 个资产可恢复，匹配的 2 份预览最大 RGB8 差为 0。使用真实观测和人为施加的位姿修正验证复用与重建，刚性修正训练步数为 0，相对位姿变化重建执行 180 步。输入资产的校验值一致。
- 实际 Python worker 的所有者故障测试通过。所有者断开时 worker 以连接关闭状态退出，socket 被清理。正常回放中的 worker 完成封存后退出。

详细证据位于工作区 `validation/slam-naming-gaussian-20261124/`，包括 `paired-verification.json`、`cuda-lifecycle/report.json` 和各构建、测试日志。

其他电脑上的构建和运行入口见 [独立 SDK](../../core-build.md)、[ROS1 与 ROS2 构建](../../ros-builds.md) 和 [高斯 worker 与节点配置](../../online-gaussian.md)。回放继续使用 `tools/run_baseline.py` 的 `--loop-closure`、`--geometry-submaps`、`--geometry-map`、`--keyframe-trajectory`、`--gaussian-persist-submaps`、`--gaussian-finalize-on-shutdown`、`--gaussian-joint-render` 和 `--gaussian-publish-preview` 选项，socket 和输出目录按运行环境提供。
