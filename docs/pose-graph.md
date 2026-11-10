# 可选 GTSAM 位姿图

位姿图是独立的普通 C++ 库，默认不编译。它接收已提交关键帧的原始里程计位姿和调用者
明确提供的相对位姿约束，使用 GTSAM 批量求解并返回一个 `PoseUpdate`。本阶段没有真实
闭环检测、自动优化线程或 `map→odom` TF；启用库不会改变 ROS 节点的运行方式。

## 构建

验收使用 GTSAM 4.2.2。先在独立前缀安装与工程使用相同 Eigen 的 GTSAM，再指定其前缀：

```bash
cmake -S /path/to/gtsam -B /path/to/gtsam-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON \
  -DCMAKE_CXX_EXTENSIONS=OFF \
  -DCMAKE_INSTALL_PREFIX=/path/to/gtsam-install \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_WITH_TBB=OFF
cmake --build /path/to/gtsam-build
cmake --install /path/to/gtsam-build

cmake -S /path/to/livo-atlas -B /path/to/livo-build \
  -DLIVO_ATLAS_ROS_VERSION=0 -DLIVO_ATLAS_WITH_GTSAM=ON \
  -DCMAKE_PREFIX_PATH="/path/to/gtsam-install;/path/to/sophus-install"
cmake --build /path/to/livo-build
(cd /path/to/livo-build && ctest --output-on-failure)
cmake --install /path/to/livo-build --prefix /path/to/livo-install
```

ROS1 和 ROS2 构建同样使用 `LIVO_ATLAS_WITH_GTSAM=ON` 并提供 GTSAM 前缀；其余 ROS
依赖和构建步骤见工程原有说明。例如在已配置的工作空间中：

```bash
# Noetic
catkin_make -DLIVO_ATLAS_WITH_GTSAM=ON \
  -DGTSAM_DIR=/path/to/gtsam-install/lib/cmake/GTSAM

# Humble
colcon build --packages-select livo_atlas --cmake-args \
  -DLIVO_ATLAS_WITH_GTSAM=ON \
  -DGTSAM_DIR=/path/to/gtsam-install/lib/cmake/GTSAM
```

关闭时工程不查找 GTSAM，也不导出 `livo_atlas::backend`。
普通 CMake 和 ROS2 的安装消费者可链接 `livo_atlas::backend`；ROS1 catkin 消费者使用
`${catkin_LIBRARIES}`，并在查找包时提供 GTSAM 前缀。GTSAM 是手动启用的可选依赖，
当前 `package.xml` 不会让 rosdep 自动安装它。GTSAM 和调用方应使用兼容的编译器、标准库、
Eigen 版本与对齐选项，尤其不要混用不同的 Eigen 构建。

## 数据与调用顺序

`RigidTransform` 记作 `T_A_B`，将 B 系坐标变换到 A 系。关键帧按从 0 开始的连续 ID
调用 `AddKeyframe(id, odom_from_body)`；相邻帧约束由原始位姿自动构造。显式回环的
`from_body_from_to_body` 是 `T_Bfrom_Bto`，要求两个关键帧已经存在、`from_id < to_id`
且不相邻；同一个有序帧对不能重复加入。旋转和平移标准差分别以 rad 和 m 为单位。
标准差及其对应精度 `1 / sigma²` 必须能以有限、非零的 double 表示；不支持通过极小
标准差隐式创建硬约束。
默认先验标准差为 0.001 rad、0.001 m，里程计标准差为 0.05 rad、0.1 m；这些只是
实验权重，不是标定结果或统计一致性的保证。

图和 `LivoSystem` 必须由同一个串行所有者使用。在线通知回调中只复制本次关键帧 ID 与
原始位姿，不要在回调中调用系统或求解。每次 `ProcessNext` 返回后立即消费该单槽；
不要用 `ProcessAvailable` 让多帧覆盖单槽。收到真实有效的相对位姿约束后，可以执行：

```cpp
// graph 已按顺序接收所有已提交关键帧；constraint 来自调用者。
graph.AddLoopConstraint(constraint);
const auto current = system.GetPoseSnapshot();
const auto candidate = graph.Optimize(*current);
if (!candidate.iteration_limit_reached) {
  system.ApplyPoseUpdate(candidate.update);
}
```

`Optimize` 不修改图或输入快照。优化后、提交前可以处理更多帧；新增尾帧沿最后一个已优化
锚点的刚体校正延续。若期间已有其他位姿版本提交，旧候选会被拒绝，应取得最新快照重新求解，
不要修改候选的 `base_version`。`iteration_limit_reached` 只表示达到迭代上限，不是收敛
判断；上例选择不提交此类候选。图误差下降也不代表地图或约束正确。

新版本使用投影后的刚体尾帧策略，保存为位姿文件 schema 2。新读取器仍支持 schema 1；
旧读取器会明确拒绝 schema 2。原始关键帧不被改写。`PoseUpdate` 和 `PoseSnapshot` 增加
了策略字段，依赖旧二进制布局的程序需要重新编译。图、快照与批量求解内存随帧数增长，
本阶段不承诺长距离实时性能。更多设计和验收边界见
[P6-02b 设计](superpowers/specs/2026-11-11-p6-02b-pose-graph-design.md)。
