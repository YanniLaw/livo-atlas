# ROS1 基线与回归验证

本文命令从包含 `src/livo-atlas`、`src/rpg_vikit` 和 `Retail_Street.bag` 的工作区根目录执行。
所有路径根据当前目录生成，不依赖本机用户名。目标环境是 ROS1 Noetic；Humble 流程见 [ros-builds.md](ros-builds.md)。

## 1. 准备环境和记录版本

需要已可用的 Noetic、C++17 编译器、CMake、Eigen、Google Test、PCL、OpenCV、Sophus；保留的 vikit 源码仅用于记录历史，不是当前依赖。
独立数学/状态测试只需要 CMake、C++17、Eigen 和 Google Test，不需要加载 ROS 环境。
验证脚本使用系统 Python3 的 `rosbag`、`rospy`、`numpy`、`psutil` 及 ROS 标准消息模块。
先使用与原工程兼容的依赖版本；本工程当前使用 `<sophus/se3.h>` 和 `libSophus.so`，不能未经构建验证直接替换成不同接口的 Sophus。
`vio/patch_size` 必须为正偶数，默认值为 8；初始化会拒绝奇数或非正值，匹配整个 patch 布局的尺寸约定。
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

P2-03 起通过 `output/directory` 写入本次证据目录，不再依赖源码路径。P2-02 及更早的节点仍向编译时 `ROOT_DIR` 下的 `Log/` 写文件，使用当前 `run_baseline.py` 验证这些旧节点时必须加 `--legacy-output`。
为固定实际测试源码与二进制，`run_baseline.py` 继续要求源码副本，拒绝直接使用开发包自身。本文后半的历史命令复现旧版本时，也应使用对应版本的工具或添加该兼容开关，不能覆盖冻结证据。
当前包不依赖 vikit；下面的链接仅保留历史工作区形态，不修改其源码。

```bash
/usr/bin/python3 - "$PACKAGE_ROOT" "$RUN_ROOT/ws/src/livo-atlas" <<'PY'
import shutil
import sys

shutil.copytree(sys.argv[1], sys.argv[2],
                ignore=shutil.ignore_patterns('.git', '.superpowers', 'build', 'devel', 'install', '__pycache__'))
PY
ln -s "$WS_ROOT/src/rpg_vikit" "$RUN_ROOT/ws/src/rpg_vikit"
ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake "$RUN_ROOT/ws/src/CMakeLists.txt"
```

每次测试新变更创建新的 `RUN_ROOT`，避免旧二进制、参数和结果混入比较。
跨电脑复制验证目录后，重新建立依赖符号链接并重新构建，不复用含旧绝对路径的 CMake 缓存。

## 3. 构建与测试

默认单线程构建，确认机器资源足够后再调整并行度。回放与大规模编译依次执行。

先独立编译数学与状态头，避免 catkin 的全局包含路径掩盖核心依赖。
下面的 `core_header_checks` 为每个核心头生成并编译一个只包含该头的源文件；用例使用同一份实现。

```bash
cmake -S "$PACKAGE_ROOT/tests/common" -B "$RUN_ROOT/core-build" \
  -DCMAKE_BUILD_TYPE=Release > "$RUN_ROOT/core-configure.log" 2>&1
nice -n 10 cmake --build "$RUN_ROOT/core-build" -- -j1 \
  > "$RUN_ROOT/core-build.log" 2>&1
(cd "$RUN_ROOT/core-build" && ctest --output-on-failure)
```

检查 `core-build/compile_commands.json` 中头文件检查的命令：只应有工程 include 和 Eigen 路径，不能出现 ROS、PCL 或 vikit 路径。
该独立入口覆盖数学、状态和标定；sensor、synchronization、writer 的独立入口及最新计数见 [P2-03 复现记录](p2-03-validation.md)。完整独立核心与安装消费见 [core-build.md](core-build.md)。
头文件检查直接读取 Eigen target 的包含路径，兼容独立入口声明的 CMake 3.10 下限；
object library 的链接支持在 [CMake 3.12](https://cmake.org/cmake/help/v3.12/release/3.12.html#commands) 才加入。
本文命令使用 Noetic 自带的 CMake 3.16，未单独实测旧版工具链。

```bash
cmake -S "$RUN_ROOT/ws/src" -B "$RUN_ROOT/ws/build" \
  -DCATKIN_DEVEL_PREFIX="$RUN_ROOT/ws/devel" \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  > "$RUN_ROOT/configure.log" 2>&1

nice -n 10 cmake --build "$RUN_ROOT/ws/build" \
  --target fastlivo_mapping pcl_memory_compatibility_test \
    core_math_state_test shared_math_contract_test \
    vio_reference_patch_test vio_sampling_test -- -j1 \
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
核心用例覆盖数学分支与 19 维状态行为，行为契约用例检查原 Euler 约定及 ResetPose 保留状态/协方差。
VIO 回归用例调用真实参考 patch 评分，验证新计算和已缓存的均值产生相同的正确评分。
采样用例调用真实 VIO 更新，覆盖粗层边界、安全细层和全部测量无效时的协方差。
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

### 建立受控的数值参考

P1-01 验证发现，P0 修复后同一个二进制的两次默认并行完整回放也有数值差异。
遇到这种情况，保留默认结果，先检查原参考的重复性；不能直接把差异归因于新代码，也不能通过放宽阈值宣称纯迁移一致。

原 VIO 使用浮点误差的 OpenMP 归约；[OpenMP 规范](https://www.openmp.org/spec-html/5.0/openmpsu107.html)不保证不同并行运行的归约结果逐位一致。
可对参考与候选统一设置 [OMP_THREAD_LIMIT](https://www.openmp.org/spec-html/5.0/openmpse58.html) 为 1，控制参与 OpenMP 计算的线程数，再做零差异比较。
仅设置 `OMP_NUM_THREADS` 不适合本工程：现有代码会调用 `omp_set_num_threads`。
这是额外的验证运行条件，算法源码与默认运行配置保持原样。

原版还存在 [缓存均值未初始化](vio-cached-mean-fix.md) 和 [粗层图像采样越界](vio-sampling-bounds-fix.md) 两项问题；只控制 OpenMP 或只修复均值仍不足以建立可靠的精确参考。
最终审阅又补上正偶数窗口的启动约束，避免不受布局支持的奇数窗口进入处理。
修复后的默认并行短回放也出现重复性差异。因此短片段和完整记录都需要统一运行条件。
下面从 P0 构建修复提交建立参考，依次应用两项修复及启动约束；候选包含同样的代码。
先完成候选的构建与测试，再构建参考，随后依次回放。保留默认模式和修复前失败结果，不复用其他电脑的二进制。

```bash
export REFERENCE_WS="$RUN_ROOT/reference_ws"
mkdir -p "$REFERENCE_WS/src/livo-atlas"
git -C "$PACKAGE_ROOT" archive 72f68d897a35993ec230e1dca8d9c7063c5580d7 \
  | tar -x -C "$REFERENCE_WS/src/livo-atlas"
git -C "$PACKAGE_ROOT" show --format= \
  b222e2d0ddf7bfeb250c53fea7df06a276df2963 -- src/vio.cpp \
  > "$RUN_ROOT/cached-mean-fix.patch"
(cd "$REFERENCE_WS/src/livo-atlas" &&
  patch -p1 < "$RUN_ROOT/cached-mean-fix.patch")
git -C "$PACKAGE_ROOT" show --format= \
  59a8af52323ff80a27a7cd2322cf588a1672edfa -- src/vio.cpp \
  > "$RUN_ROOT/sampling-bounds-fix.patch"
(cd "$REFERENCE_WS/src/livo-atlas" &&
  patch -p1 < "$RUN_ROOT/sampling-bounds-fix.patch")
git -C "$PACKAGE_ROOT" show --format= \
  0bfb12a4974c7de54afd091f80d4a452f06c8dd0 -- src/vio.cpp \
  > "$RUN_ROOT/patch-dimensions-fix.patch"
(cd "$REFERENCE_WS/src/livo-atlas" &&
  patch -p1 < "$RUN_ROOT/patch-dimensions-fix.patch")
ln -s "$WS_ROOT/src/rpg_vikit" "$REFERENCE_WS/src/rpg_vikit"
ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake \
  "$REFERENCE_WS/src/CMakeLists.txt"

cmake -S "$REFERENCE_WS/src" -B "$REFERENCE_WS/build" \
  -DCATKIN_DEVEL_PREFIX="$REFERENCE_WS/devel" \
  -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  > "$RUN_ROOT/reference-configure.log" 2>&1
nice -n 10 cmake --build "$REFERENCE_WS/build" \
  --target fastlivo_mapping -- -j1 > "$RUN_ROOT/reference-build.log" 2>&1
```

确认参考与候选的八个算法 `.cpp` 文件相同；编译命令在替换工作区路径后相同。
保存两份源码、补丁、编译日志和本机依赖版本。两份数学/状态头的区别才是本次结构迁移的比较对象。

```bash
mkdir -p "$RUN_ROOT/serial"

OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE \
/usr/bin/python3 "$PACKAGE_ROOT/tools/run_baseline.py" \
  --workspace "$REFERENCE_WS" --bag "$WS_ROOT/Retail_Street.bag" \
  --output "$RUN_ROOT/serial/p0_short" --duration 30 --rate 0.5 --port 11361

OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE \
/usr/bin/python3 "$PACKAGE_ROOT/tools/run_baseline.py" \
  --workspace "$RUN_ROOT/ws" --bag "$WS_ROOT/Retail_Street.bag" \
  --output "$RUN_ROOT/serial/short_a" --duration 30 --rate 0.5 --port 11362

OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE \
/usr/bin/python3 "$PACKAGE_ROOT/tools/run_baseline.py" \
  --workspace "$RUN_ROOT/ws" --bag "$WS_ROOT/Retail_Street.bag" \
  --output "$RUN_ROOT/serial/short_b" --duration 30 --rate 0.5 --port 11363

OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE \
/usr/bin/python3 "$PACKAGE_ROOT/tools/run_baseline.py" \
  --workspace "$REFERENCE_WS" --bag "$WS_ROOT/Retail_Street.bag" \
  --output "$RUN_ROOT/serial/p0_full" --duration 136 --rate 0.5 --port 11359

OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE \
/usr/bin/python3 "$PACKAGE_ROOT/tools/run_baseline.py" \
  --workspace "$RUN_ROOT/ws" --bag "$WS_ROOT/Retail_Street.bag" \
  --output "$RUN_ROOT/serial/p1_full" --duration 136 --rate 0.5 --port 11360

/usr/bin/python3 "$PACKAGE_ROOT/tools/compare_trajectories.py" \
  --reference "$RUN_ROOT/serial/p0_full/trajectory.tum" \
  --candidate "$RUN_ROOT/serial/p1_full/trajectory.tum" \
  --max-position-m 0 --max-orientation-deg 0 \
  --output "$RUN_ROOT/serial_full_vs_p0.json"
```

使用同样的零阈值比较两次候选短回放与参考短回放，以及两次候选短回放之间的重复性。
检查五份 `mapping.log` 都记录 `OMP_THREAD_LIMIT = '1'`，保存环境设置与参数，并比较 `mat_pre.txt`、`mat_out.txt`：

```bash
/usr/bin/python3 - "$RUN_ROOT/serial" <<'PY'
from pathlib import Path
import sys
import numpy as np

root = Path(sys.argv[1])
for reference, candidate in [('p0_short', 'short_a'), ('p0_short', 'short_b'),
                             ('short_a', 'short_b'), ('p0_full', 'p1_full')]:
    for filename in ['mat_pre.txt', 'mat_out.txt']:
        old = np.loadtxt(root / reference / filename, ndmin=2)
        new = np.loadtxt(root / candidate / filename, ndmin=2)
        assert np.isfinite(old).all() and np.isfinite(new).all()
        assert old.shape == new.shape and np.array_equal(old, new), (
            reference, candidate, filename)
print('All paired state logs match at the exported precision')
PY
```

单线程回归的结论只覆盖该受控运行条件，不证明默认并行运行逐位确定，也不替代默认模式的正常退出和输入检查。

## 6. 中断与复现崩溃

终端正常中断时，脚本会清理它创建的进程；机器异常重启会使当次结果不完整。
查看 `summary.json`、`failure.json` 和完整退出日志；缺失摘要或只留下部分轨迹的运行标记为未完成，用新输出目录重试。
端口仍被占用时选择另一个空闲端口，不停止别人的 ROS master。

需要调用栈时，在短回放命令中增加 `--gdb`；系统需要已有 gdb。
此模式使用 C locale，要求日志明确记录被调试进程正常退出，并拒绝信号中断；gdb 自身返回 0 不足以判定节点成功。
`sampled_resource_process` 标明资源采样对象。gdb 模式采集的是调试器资源，不能用它评估估计器的内存和 CPU。
每次验证生成唯一的轨迹序列名，并拒绝读取已存在的目标文件，避免旧轨迹被误用为本次结果。
