# ROS1 基线与回归验证

本文命令从包含 `src/livo-atlas`、`src/rpg_vikit` 和 `Retail_Street.bag` 的工作区根目录执行。
所有路径根据当前目录生成，不依赖本机用户名。目标环境是 ROS1 Noetic；Humble 验证流程在 P4 实施时增加。

## 1. 准备环境和记录版本

需要已可用的 Noetic、C++17 编译器、CMake、Eigen、PCL、OpenCV、Sophus 和现有 vikit。
验证脚本使用系统 Python3 的 `rosbag`、`rospy`、`numpy`、`psutil` 及 ROS 标准消息模块。
先使用与原工程兼容的依赖版本；本工程当前使用 `<sophus/se3.h>` 和 `libSophus.so`，不能未经构建验证直接替换成不同接口的 Sophus。
如果 Python 导入失败，先补齐对应系统/ROS 依赖，再运行验证。

```bash
source /opt/ros/noetic/setup.bash
export WS_ROOT="$(pwd)"
export PACKAGE_ROOT="$WS_ROOT/src/livo-atlas"
export RUN_ROOT="$WS_ROOT/validation/manual-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$RUN_ROOT/ws/src"

/usr/bin/python3 -c 'import rosbag, rospy, numpy, psutil; print("Python dependencies OK")'
git -C "$PACKAGE_ROOT" rev-parse HEAD > "$RUN_ROOT/revision.txt"
git -C "$PACKAGE_ROOT" diff HEAD > "$RUN_ROOT/source.patch"
git -C "$PACKAGE_ROOT" status --short > "$RUN_ROOT/source-status.txt"
git -C "$WS_ROOT/src/rpg_vikit" rev-parse HEAD > "$RUN_ROOT/vikit-revision.txt"
git -C "$WS_ROOT/src/rpg_vikit" status --short > "$RUN_ROOT/vikit-status.txt"
c++ --version > "$RUN_ROOT/compiler.txt"
cmake --version > "$RUN_ROOT/cmake.txt"
uname -a > "$RUN_ROOT/platform.txt"
rosversion -d > "$RUN_ROOT/ros-distro.txt"
sha256sum "$WS_ROOT/Retail_Street.bag" > "$RUN_ROOT/bag.sha256"
```

另外记录实际使用的 Eigen/PCL/OpenCV/Sophus 来源和版本；CMake 配置日志及缓存会保留查找路径。
`source.patch` 不包含未跟踪的新文件，所以必须一并保存后面的源码副本和 `source-status.txt`。
初次参考环境为 GCC 9.4.0、CMake 3.16.3、PCL 1.10、ROS1 Noetic / x86_64。

## 2. 创建源码副本

当前算法会向编译时 `ROOT_DIR` 下的 `Log/` 写文件，因此验证节点必须从源码副本构建。
`run_baseline.py` 会拒绝直接使用开发包自身作为测试源码。
vikit 只通过符号链接参与构建，不修改其源码。

```bash
/usr/bin/python3 - "$PACKAGE_ROOT" "$RUN_ROOT/ws/src/livo-atlas" <<'PY'
import shutil
import sys

shutil.copytree(sys.argv[1], sys.argv[2],
                ignore=shutil.ignore_patterns('.git', 'build', 'devel', 'install', '__pycache__'))
PY
ln -s "$WS_ROOT/src/rpg_vikit" "$RUN_ROOT/ws/src/rpg_vikit"
ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake "$RUN_ROOT/ws/src/CMakeLists.txt"
```

每次测试新变更创建新的 `RUN_ROOT`，避免旧二进制、参数和结果混入比较。
跨电脑复制验证目录后，重新建立依赖符号链接并重新构建，不复用含旧绝对路径的 CMake 缓存。

## 3. 构建与测试

默认单线程构建，确认机器资源足够后再调整并行度。回放与大规模编译依次执行。

```bash
cmake -S "$RUN_ROOT/ws/src" -B "$RUN_ROOT/ws/build" \
  -DCATKIN_DEVEL_PREFIX="$RUN_ROOT/ws/devel" \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  > "$RUN_ROOT/configure.log" 2>&1

nice -n 10 cmake --build "$RUN_ROOT/ws/build" \
  --target fastlivo_mapping pcl_memory_compatibility_test -- -j1 \
  > "$RUN_ROOT/build.log" 2>&1

cmake --build "$RUN_ROOT/ws/build" --target run_tests -- -j1 \
  > "$RUN_ROOT/tests.log" 2>&1
catkin_test_results "$RUN_ROOT/ws/build/test_results"

cd "$RUN_ROOT/ws/src/livo-atlas"
/usr/bin/python3 -m unittest discover -s tests -p 'test_*.py' -v
cd "$WS_ROOT"
```

依次检查每条命令的退出码，构建或测试失败时停止回放并保留日志。
PCL 内存用例实际调用系统 `VoxelGrid` 分配点云并释放它，可捕获 native 编译引起的分配约定不兼容。
Python 用例校验轨迹比较的位移、旋转、四元数符号和时间序列检查。

## 4. 回放短片段和完整 bag

脚本启动自己的本机 ROS master，不连接正在使用的 ROS graph；端口被占用时会拒绝启动。
加载原 `avia.yaml` 与 `camera_pinhole.yaml`，打开轨迹输出，使用仿真时间，不需要 RViz。
脚本结束时清理自己启动的节点。输出目录必须尚不存在；中断后的重试采用新的目录名。

```bash
/usr/bin/python3 "$PACKAGE_ROOT/tools/run_baseline.py" \
  --workspace "$RUN_ROOT/ws" --bag "$WS_ROOT/Retail_Street.bag" \
  --output "$RUN_ROOT/short_a" --duration 30 --rate 0.5 --port 11345

/usr/bin/python3 "$PACKAGE_ROOT/tools/run_baseline.py" \
  --workspace "$RUN_ROOT/ws" --bag "$WS_ROOT/Retail_Street.bag" \
  --output "$RUN_ROOT/short_b" --duration 30 --rate 0.5 --port 11346

/usr/bin/python3 "$PACKAGE_ROOT/tools/run_baseline.py" \
  --workspace "$RUN_ROOT/ws" --bag "$WS_ROOT/Retail_Street.bag" \
  --output "$RUN_ROOT/full" --duration 136 --rate 0.5 --port 11347
```

该 bag 记录时长约 135.47 秒，`--duration 136` 覆盖完整记录。完整 0.5 倍回放约需 4.5 分钟，另有启动与退出时间。
目前脚本针对这份 bag 的固定传感器/输出话题进行计数；使用其他数据集前，先适配其参数与话题。

每个输出目录包含：

- `summary.json`：快照源文件摘要、二进制摘要、命令、输入/输出计数、资源样本和退出结果。
- `parameters.yaml`：实际运行参数。
- `trajectory.tum`：传感器时间对应的位姿，顺序为 `t x y z qx qy qz qw`。
- `mat_pre.txt` / `mat_out.txt` / `imu.txt`：现有算法日志。
- `odometry_published.tum`：ROS 发布的位姿；原版发布时使用 ROS 当前时间，不用于传感器时间对照。
- `mapping.log`、`player.log`、`roscore.log`、`resources.csv`：运行、退出与资源证据。
- `failure.json`：存在验收失败时记录原因，不因轨迹已经生成而判定通过。

脚本在 EOF 后只推进 `/clock` 约 2 秒，帮助使用仿真时间的 ROS 循环排空队列，不增加虚构传感器数据。
输入观察者的计数只证明消息到达观察者；判断估计器处理情况还需节点日志、轨迹时间及更新序列。

## 5. 比较与判定

先确认 `clean_shutdown` 为 true、player 返回 0、没有输入计数不匹配、没有非有限轨迹与异常日志。
完整记录应观察到 IMU 27447、点云 1355、图像 1355；原版参考输出为 1351 个轨迹点。
轨迹必须使用相同传感器时间序列。比较工具不执行刚体对齐，也不偷偷只比较重叠区间。

```bash
/usr/bin/python3 "$PACKAGE_ROOT/tools/compare_trajectories.py" \
  --reference "$RUN_ROOT/short_a/trajectory.tum" \
  --candidate "$RUN_ROOT/short_b/trajectory.tum" \
  --max-position-m 0 --max-orientation-deg 0 \
  --output "$RUN_ROOT/repeatability.json"
```

纯结构迁移在同一机器、同一编译参数下，先以位置/姿态零差异作为导出精度内的检查；并检查状态日志。
需要跨工具链容差时明确传入阈值并记录依据，不使用此处的零阈值替代跨平台评估。
比较 native 构建与本次构建修复时省略两项阈值参数，量化差异；`within_limits: null` 表示没有作阈值验收，不能解释为精度达标。

```bash
/usr/bin/python3 "$PACKAGE_ROOT/tools/compare_trajectories.py" \
  --reference /path/to/reference/full/trajectory.tum \
  --candidate "$RUN_ROOT/full/trajectory.tum" \
  --output "$RUN_ROOT/full-comparison.json"
```

原版轨迹文本只有六位小数，不能据此证明内部浮点状态逐位相同。
状态日志不包含重力和完整协方差；这些量在后续结构迁移中通过直接状态测试或额外诊断记录验证。
原版崩溃可能丢失最后一项缓冲的打印计时表，打印表格数量不能直接当作真实更新次数。
本验证没有真值轨迹，不报告 ATE/RPE。计时和 RSS 受机器负载、编译参数及分配器影响，实时验收另做 1 倍回放和积压检查。

## 6. 中断与复现崩溃

终端正常中断时，脚本会清理它创建的进程；机器异常重启会使当次结果不完整。
查看 `summary.json`、`failure.json` 和完整退出日志；缺失摘要或只留下部分轨迹的运行标记为未完成，用新输出目录重试。
端口仍被占用时选择另一个空闲端口，不停止别人的 ROS master。

需要调用栈时，在短回放命令中增加 `--gdb`；系统需要已有 gdb。
此模式使用 C locale，要求日志明确记录被调试进程正常退出，并拒绝信号中断；gdb 自身返回 0 不足以判定节点成功。
`sampled_resource_process` 标明资源采样对象。gdb 模式采集的是调试器资源，不能用它评估估计器的内存和 CPU。
每次验证生成唯一的轨迹序列名，并拒绝读取已存在的目标文件，避免旧轨迹被误用为本次结果。
