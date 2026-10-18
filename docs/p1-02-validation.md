# P1-02 配置与标定边界实测记录

范围为 [P1-02 执行计划](superpowers/plans/2026-10-18-p1-02-calibration.md)。本阶段只修改 `livo-atlas`，vikit 源码保持不变。
P1-02 已完成。最终源码 `ac64ce4` 通过直接测试、40 项启动检查、短段和完整配对、默认并行检查，独立审阅的一项网格问题已闭环。

## 配置入口

普通类型分别位于 `sensor/extrinsics.h`、`camera/camera_config.h`、`frontend/calibration.h`。
`RigidTransform` 表达 `p_A = R_A_B * p_B + t_A_B`，平移为米；`Calibration` 组合 `T_I_L`、`T_C_L` 和相机标定。
这些类型及校验组成 `livo_atlas_calibration`，独立构建只需要标准库、Eigen。

`ros1::ReadCalibration` 读取原外参参数和节点私有相机参数，成功后才提交输出对象。
ROS 数值标量允许整数/浮点表示，布尔值和字符串不当作数值；图像宽高要求整数类型。
缺失的 scale 使用 1，畸变系数使用 0；提供错误类型时不再悄悄使用默认值。
默认节点名仍为 `laserMapping`，节点重命名后会读取对应的私有标定。

| 配置 | 内部含义 / 规则 |
| --- | --- |
| `extrin_calib/extrinsic_R`、`extrinsic_T` | `T_I_L`，9 个行优先旋转系数、3 个平移系数 |
| `extrin_calib/Rcl`、`Pcl` | `T_C_L`，使用相同数组规则 |
| 相机原始宽高、fx/fy/cx/cy、scale | 保留原构造参数，由相机只缩放一次；正数尺寸乘积截断为整数 |
| `Pinhole` 的 cam_d0–cam_d3 | 径向 k1/k2、切向 p1/p2，继续使用原 vikit 模型 |
| `EquidistantCamera` 的 k1–k4 | 继续使用原 vikit 鱼眼模型 |

外参校验检查有限值、右手旋转及正交性，正交残差的最大系数和行列式误差阈值均为 `1e-4`，不修改标定系数。
包内四套 YAML 的旋转舍入残差约为 `1e-6`，全部可加载。非单位旋转和非零平移的真实 VIO 初始化测试确认 Rci/Pci 的方向和组合。
相机校验检查支持的模型、有限正焦距和缩放、有效的缩放尺寸、整数像素索引范围，以及原针孔 OpenCV float 矩阵的表示范围。
有限的图像外主点保留。其他 vikit 加载器曾提供、但包内配置未使用的 Ocam/ATAN/PolynomialCamera 不在本阶段支持范围内，启动时明确报告。

LIVMapper 在文件输出、估计初始化和订阅注册前完成配置校验；另检查所用 VIO 网格、正偶数 patch、金字塔索引及乘积范围。
当前 COLMAP 导出调用针孔去畸变功能，因此鱼眼加 COLMAP 在启动时拒绝。其他前端参数和传感器消息内容的契约按后续阶段迁移。
VIO 外参 setter 接受已解析的 `RigidTransform`，原 Rli/Pli、Rci/Pci、滤波更新、预处理、时间同步和调度公式保持不变。

## 相机所有权与本机兼容性调查

相机现在由 mapper 的 `unique_ptr` 拥有，VIO 保留非拥有指针，相机生命周期覆盖 VIO。
第一次真实工厂测试在相机删除时崩溃；gdb 栈顶为 `free(0x291)`。
vikit 使用 `-march=native`，当前包使用默认指令集；本机 Eigen 3.3.7 对两者采用不同的分配/释放约定。

同一个 vikit 库的最小复现实测：

| 创建/删除路径 | Eigen 默认对齐 / malloc 已对齐 | 结果 |
| --- | --- | --- |
| 默认指令集创建原相机、vikit 虚析构删除 | 16 / 1 | SIGSEGV |
| 与 vikit 相同的 native 编译 | 32 / 0 | 正常删除 |
| 默认指令集的内部派生相机 | 16 / 1 | 正常删除 |

工厂使用没有数据成员的内部派生类，将分配和虚 deleting destructor 留在同一编译目标；投影、反投影和原相机构造仍使用 vikit 实现。
它仅用于这段迁移期的 ABI 兼容，P3 替换 vikit 时一并移除。未更改 vikit 编译选项或源码，也未给本包重新开启 native。
两种模型的真实测试都覆盖创建、缩放、投影、反投影及销毁。

## 环境、直接测试和启动检查

本机 Noetic / Ubuntu 20.04 / x86_64，GCC 9.4.0、CMake 3.16.3、Eigen 3.3.7、OpenCV 4.2、PCL 1.10。
构建使用 Release、`nice -n 10`、`-j1`，重编译与回放依次进行。
实际证据位于工作区 `validation/P1-02-noetic-20261018/`；产物不进入源码仓库，迁移电脑时需单独复制。

| 检查 | 结果 |
| --- | --- |
| 普通标定行为测试 RED | 14 个测试完成编译，占位实现使其中 11 个失败 |
| 工厂测试 RED | 两个实际相机创建用例失败；随后暴露并定位虚析构释放问题 |
| 原节点启动 RED | 重命名、短旋转数组、错误可选类型三项不符合新边界要求 |
| 独立构建 | 数学/状态 14 个及标定 14 个用例通过；6 个公共头分别包含编译通过 |
| Noetic C++ | 42 个实际用例通过，包含原有 24 个及新增标定 14 个、相机/VIO 外参 4 个 |
| Python | 原有 10 个工具用例通过 |
| 真实启动 | 40 项通过：9 个有效配置场景、31 个错误配置场景 |
| 诊断和副作用 | 31 个错误场景均返回 1、有相关参数诊断，未注册传感器订阅且未改变 Log 文件指纹 |

独立构建的编译命令没有 ROS、PCL、vikit、native 或 fast-math 路径/参数。
相同的核心用例也由 catkin 执行，不把两次执行重复累计成独立用例数量。
本机 `catkin_test_results` 显示 84，因为它累计 XML suite 与子项；实际数量按 7 个 XML 根节点统计为 42。
格式按 Google 布局人工检查并运行 `git diff --check`；本机没有 clang-format，不声称已运行自动格式检查。

## 完整数值回归

参考为 P1-01 最终提交 `2b515ae471451e073dd26fd8add5bf6a9830801e`，最终候选为 `ac64ce4f75878211baf77776d3b0e81e2336a1e5`。
两边分别创建源码副本和 CMake 构建，不复用其他路径的缓存；统一 `OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE`、0.5 倍回放。
每次运行日志确认实际 OpenMP 限制。此条件用于纯结构迁移的精确对照，不改变默认运行设置。

| 配对 | 参考 / 候选轨迹点数 | 最大位置差 | 最大姿态差 | 状态日志 |
| --- | --- | --- | --- | --- |
| 30 秒短段 | 296 / 296 | 0 m | 0° | mat_pre 597×19、mat_out 591×20，逐值相同 |
| 完整 bag，136 秒覆盖 | 1351 / 1351 | 0 m | 0° | mat_pre 2707×19、mat_out 2701×20，逐值相同 |

四次运行均正常退出；完整输入为 IMU 27447、点云 1355、图像 1355，观察者未发现数量不匹配。
对应的传感器时间、全部导出位姿值、LIO/VIO 更新次数也一致。未进行轨迹对齐或时间裁剪，未放宽零阈值。
更新次数为完整记录 LIO 1351、VIO 1350；状态日志还包含原版记录的初始化/交替更新行，不能将日志行数直接当作轨迹点数。
候选默认并行另跑 45 秒，输出 446 个位姿，输入计数匹配、状态有限、节点正常退出。
它用于默认模式的运行检查，不用于证明默认并行的数值重复性。

五个未改变的算法 `.cpp` 字节相同；含 VIO 在内的六个估计源文件编译命令替换工作区路径后相同。
VIO 源文件的变化仅为接收已解析外参的 setter；标定检查不正交化矩阵，图像处理与状态公式未改。
检查针对原导出精度，不证明内部浮点状态逐位相同；状态文本也不包含完整协方差和重力向量。

| 包外证据 | 内容 |
| --- | --- |
| `reference_ws/`、`candidate_ws/` | 冻结源码、独立构建、编译命令及测试 XML |
| `core-red.log`、`camera-red-evidence/`、`startup-red-retry/` | 行为测试与原节点的 RED 证据 |
| `camera-delete-diagnostic/` | 原始虚析构问题的独立复现代码、命令和结果 |
| `startup-green/` | 最终全部 40 个真实节点案例、参数、日志、摘要 |
| `reference_short/`、`candidate_short/`、`reference_full/`、`candidate_full/` | 每次运行的实际参数、轨迹、状态、输入/退出证据 |
| `candidate_default_45/` | 默认并行的 45 秒正常运行检查 |
| `short-comparison.json`、`full-comparison.json` 及 `*-states.json` | 两组零阈值比较与状态数组检查 |
| `runtime-artifacts.json`、`build-settings-comparison.json` | 两边可执行文件及 7 个运行库摘要、编译参数比较 |
| `verify.sh`、`check_results.py`、`acceptance.json` | 最终测试命令、摘要核对逻辑及验收汇总 |
| `final-review/reviewed-version/` | `54d73a1` 的原源码、运行库、37 项启动检查及三份候选回放，修复前归档 |
| `final-review/grid-red/`、`final-review/review-report.md` | 审阅问题的真实 RED 证据、独立审阅结论及执行裁决 |

最初的沙箱套接字拒绝记录在 `startup-red-runner.log`，它不是功能测试结果；获准建立本机私有 ROS graph 后重新运行并保存真实 RED/GREEN 结果。
原节点未通过的历史记录与相机删除崩溃日志均保留，没有作为通过结果计入。

## 独立审阅与网格修复

独立只读审阅覆盖 `2b515ae..54d73a1` 全部变更，发现 0 项 Critical、1 项 Important、0 项 Minor。
原 VIO 将整数除法结果再传给 ceil，实际仍按向下取整分配网格。640×512、grid_size=300、patch_size=8、pyramid=4 时只分配 2 格；
像素 (320,400) 通过 border=80 的图像检查，但得到索引 3。相同问题也能由 grid_n_height 推导出的单元尺寸触发。

两个真实节点用例在修复前都错误地完成启动；修复后均返回 1、没有订阅和日志副作用。
修复只在启动时检查 `width % cell_size <= border`、`height % cell_size <= border`，保留原网格分配和计算公式。
新增 grid_size=150 的安全非整除案例，验证不会把所有非整除网格都拒绝；四套既有 YAML 仍通过。
候选修复版为 `ac64ce4f75878211baf77776d3b0e81e2336a1e5`；参考源码和二进制未变化，重用本阶段已实测的参考短段/完整结果。
修复版候选重新执行 30 秒、完整 136 秒和默认并行 45 秒，分别输出 296、1351、446 个位姿，全部正常退出；上方两组零差异表已对应最终修复版。
主工作区的 `fastlivo_mapping` 也已重新构建。最后汇总核对最终 5 份运行记录及归档的 3 份审阅版候选记录的源码、运行库和可执行文件摘要。
审阅发现的唯一 Important 用实际节点用例先复现后修复，全部直接测试与最终 bag 回归通过；没有遗留 Minor，也没有用第二轮审阅代替回归测试。

## 实施取舍与代价

| 决定 | 理由与代价 |
| --- | --- |
| 在当前 refactor-fast_livo2 分支原位实施 | 沿用已确认的执行方式；并发编辑时需要协调，回放输出通过隔离源码副本保护。 |
| 按本轮授权直接完成文档、编码及验证 | 不重复请求同一执行步骤的许可；若范围理解有误，可回退这些本地提交。 |
| 删除未初始化且立即被覆盖的 mapper grid_n_width | 不影响 VIO 网格公式；若外部消费者直接使用旧字段，需要补兼容入口。 |
| 针孔参数同时检查 float 表示范围 | 原 vikit 会创建 float OpenCV 矩阵；超范围标定将被拒绝，而不是构造 Inf 矩阵。 |
| 临时内部相机派生类保持分配与释放约定一致 | 修复真实删除崩溃、保持 vikit 不变；不同工具链需要重新执行相机生命周期回归。 |
| 保留既有鱼眼特殊值行为直到 P3 | 本阶段保持模型计算；相关输入仍受原实现限制，兼容测试不等于模型正确性证明。 |
| 非标定参数的全面校验随模块迁移 | 控制本阶段范围；未迁移选项仍可能保留原来的默认回退行为。 |
| 结论限定于本机已测条件 | 当前不保证 ARM/Humble、实时性或真值精度；这些目标需要对应环境和数据验证。 |

## 在其他 Noetic 电脑复现

使用当前阶段完整源码、原始 vikit 和同一 bag，按 [通用验证说明](validation.md) 创建隔离源码副本。
本阶段的参考直接取 `2b515ae471451e073dd26fd8add5bf6a9830801e`；它已经包含 P1-01 的全部修复，不需要额外补丁。
候选取本阶段源码提交。不要复用其他目录/电脑的 CMake 缓存或二进制。

```bash
source /opt/ros/noetic/setup.bash
set -euo pipefail
export WS_ROOT=/path/to/livo-atlas_ws
export PACKAGE_ROOT="$WS_ROOT/src/livo-atlas"
export RUN_ROOT="$WS_ROOT/validation/p1-02-reproduce"
export CANDIDATE_REV=ac64ce4f75878211baf77776d3b0e81e2336a1e5
mkdir -p "$RUN_ROOT"

cmake -S "$PACKAGE_ROOT/tests/core" -B "$RUN_ROOT/core-build" \
  -DCMAKE_BUILD_TYPE=Release
nice -n 10 cmake --build "$RUN_ROOT/core-build" -- -j1
(cd "$RUN_ROOT/core-build" && ctest --output-on-failure)
```

`CANDIDATE_REV` 固定到本阶段最终源码版本，后续文档提交不改变该版本的生产代码。
创建两个独立源码副本，参考只需构建节点，候选运行全部测试：

```bash
for P102_NAME in reference_ws candidate_ws; do
  mkdir -p "$RUN_ROOT/$P102_NAME/src/livo-atlas"
  if [ "$P102_NAME" = reference_ws ]; then
    P102_REVISION=2b515ae471451e073dd26fd8add5bf6a9830801e
  else
    P102_REVISION="$CANDIDATE_REV"
  fi
  git -C "$PACKAGE_ROOT" archive "$P102_REVISION" \
    | tar -x -C "$RUN_ROOT/$P102_NAME/src/livo-atlas"
  ln -s "$WS_ROOT/src/rpg_vikit" "$RUN_ROOT/$P102_NAME/src/rpg_vikit"
  ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake \
    "$RUN_ROOT/$P102_NAME/src/CMakeLists.txt"
  cmake -S "$RUN_ROOT/$P102_NAME/src" -B "$RUN_ROOT/$P102_NAME/build" \
    -DCATKIN_DEVEL_PREFIX="$RUN_ROOT/$P102_NAME/devel" \
    -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  nice -n 10 cmake --build "$RUN_ROOT/$P102_NAME/build" \
    --target fastlivo_mapping -- -j1
done

nice -n 10 cmake --build "$RUN_ROOT/candidate_ws/build" \
  --target run_tests -- -j1
catkin_test_results "$RUN_ROOT/candidate_ws/build/test_results"
(cd "$RUN_ROOT/candidate_ws/src/livo-atlas" &&
  /usr/bin/python3 -m unittest discover -s tests -p 'test_*.py' -v)
```

节点测试需要独立本机端口，输出目录必须不存在；`--workspace` 必须指向含源码副本和 devel 的工作区：

```bash
/usr/bin/python3 "$PACKAGE_ROOT/tools/check_calibration_startup.py" \
  --workspace "$RUN_ROOT/candidate_ws" --output "$RUN_ROOT/startup" --port 11372

OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE \
/usr/bin/python3 "$PACKAGE_ROOT/tools/run_baseline.py" \
  --workspace "$RUN_ROOT/reference_ws" --bag "$WS_ROOT/Retail_Street.bag" \
  --output "$RUN_ROOT/reference_short" --duration 30 --rate 0.5 --port 11373

OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE \
/usr/bin/python3 "$PACKAGE_ROOT/tools/run_baseline.py" \
  --workspace "$RUN_ROOT/candidate_ws" --bag "$WS_ROOT/Retail_Street.bag" \
  --output "$RUN_ROOT/candidate_short" --duration 30 --rate 0.5 --port 11374
```

短段轨迹和状态日志通过零阈值比较后，依次对参考和候选运行 `--duration 136 --rate 0.5`，输出到不同新目录并比较完整结果。
默认并行的候选另跑 45 秒，检查输入、有限输出及正常退出。详见通用说明中的比较命令和限制说明。
本机 `run_replays.py` 保存完整顺序，`verify.sh` 重跑直接测试并核对运行、源码及二进制摘要；这些证据脚本随包外验证目录迁移。
源码交接包位于 `validation/P1-02-noetic-20261018/handoff/livo-atlas-p1-02.bundle`，包含完整本地 Git 历史；同目录提供摘要和恢复说明。
bag、vikit 和验证产物需单独携带，编译产物及旧 CMake 缓存不用于另一台电脑的构建。

## 验证范围

普通标定库可以无 ROS 构建；完整前端仍依赖 ROS1 和 vikit。
当前结果不覆盖 ARM、Humble、默认并行的逐位确定性、1 倍实时性、带真值的定位精度或其他数据集的运行精度。
启动时加载 HILTI/NTU/MARS 配置只证明配置与初始化可用，其数据集的完整算法回放尚未执行。
鱼眼模型沿用 vikit 既有的光轴等特殊值行为，P3 替换模型时需要独立处理；当前相机测试证明与旧实现一致，不证明模型对全部特殊输入都数学正确。
非标定 ROS 参数的全面严格校验随对应模块迁移推进，本阶段没有引入完整前端配置框架。
后续阶段是 P1-03：内部 IMU、图像、点云测量类型及 ROS1 转换。
