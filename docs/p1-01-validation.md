# P1-01 数学与状态隔离实测记录

2026-10-11 开始实施，2026-10-18 接续验证。范围为 [P1-01 计划](superpowers/plans/2026-10-11-p1-01-core-state.md)。验证目录沿用初始日期。
P1-01 已完成。数学/状态隔离、直接测试、完整配对、短回放重复性、默认模式及独立审阅问题闭环均已通过。

## 实施范围

- 六个 Eigen 类型别名和 SO(3) 运算迁入 `include/livo_atlas/core`。
- 19 维状态迁入 `core/state.h`；旧 `StatesGroup` 是同一类型的别名，旧数学入口转发到核心。
- 核心头只依赖 Eigen 和标准库，保留原阈值、旋转右乘、状态索引、协方差初值与重置范围。
- 旧字段与 `resetpose` 保留兼容；新增代码采用 Google C++ 命名和布局。

数值调查发现原版 VIO 的缓存均值未初始化和粗层图像采样越界，分别提交为 `b222e2d`、`59a8af5`。参考与候选应用同样的修复；数学/状态迁移不改标定、同步或状态公式。
最终审阅在 `0bfb12a` 补上正偶数窗口启动约束。合法默认配置的采样、状态代码保持不变，完整配对测量仍对应审阅版本 `4af32de`；最新代码另有直接回归和默认 45 秒验证。
完整前端库、ROS 测量去耦、Humble、vikit 替换、GTSAM 与 3DGS 属于后续阶段。

## 环境和证据

本机为 Noetic / Ubuntu 20.04 / x86_64，GCC 9.4.0、CMake 3.16.3、Eigen 3.3.7、Google Test 1.10.0、PCL 1.10、OpenCV 4.2。
构建使用 Release、编译器基准指令集、`nice -n 10`、`-j1`；编译与回放顺序执行。

下文路径相对于工作区 `validation/P1-01-noetic-20261011/`：

| 路径 | 内容 |
| --- | --- |
| `../P1-01-core-tests/` | 仅依赖 Eigen/GTest 的测试与三头独立包含检查 |
| `ws/`、`short_a/`、`short_b/`、`full/` | 首轮迁移的冻结源码、构建及回放 |
| `serial/` | 修复 VIO 前的单线程配对结果 |
| `vio_mean_fix/archive_before_bounds/` | 仅修复均值时的源码、二进制和测试快照 |
| `vio_mean_fix/archive_before_review/` | 完整配对实际使用的审阅版本源码、二进制及测试快照 |
| `vio_mean_fix/p0_ws/`、`p1_ws/` | 本机参考与最终候选构建目录 |
| `vio_mean_fix/bounds_fix/` | 新配对回放、运行条件、二进制摘要及比较报告 |
| `final-review/` | 审阅、配置用例红/绿证据、最新二进制和默认 45 秒回放 |
| `numerical-diagnostic/` | 冻结旧/新数学与状态的 20,000 组直接对照 |
| `vio-reference-tests/` | 缓存均值用例的红/绿证据 |
| `vio_mean_fix/sampling-diagnostic/` | 实际越界诊断与采样用例红/绿证据 |

每次回放保存实际参数、输入计数、传感器时间轨迹、状态文本、退出结果、源码摘要与二进制摘要。
旧摘要中的工作区路径对应当时的位置；边界修复前的源码和二进制按表中归档路径核对。旧输出和失败报告保持原样。
新增未跟踪文件也包含在源码副本中，`source.patch` 本身不能替代完整快照。
产物在包外，迁移电脑时需单独复制；重建方法见 [验证说明](validation.md)。

## 构建与直接测试

| 检查 | 实测结果 | 证据 |
| --- | --- | --- |
| 数学测试红阶段 | 依赖配置成功，因缺少核心头编译失败 | `P1-01-math-red.log` |
| 状态测试红阶段 | 依赖配置成功，因缺少 `state.h` 编译失败 | `P1-01-state-red.log` |
| 独立数学/状态用例 | 14 个通过：float/double 数学 8 个、状态 6 个 | `core-tests.log` |
| 核心头独立包含 | 三个翻译单元通过；无 ROS/PCL/vikit、native 或 fast-math 路径/参数 | `core-compile-commands.json` |
| 候选 Noetic 节点和用例 | 最终节点构建成功，24 个 C++ 用例通过 | `final-review/green-build-tests.log`、`vio_mean_fix/p1_ws/build/test_results/` |
| 旧数学/状态参考 | 节点构建成功，7 个 C++ 用例通过 | `vio_mean_fix/bounds-p0-build-tests.log`、`vio_mean_fix/p0_ws/build/test_results/` |
| 完整配对时的算法编译条件 | 8 个 `.cpp` 字节相同；编译命令替换工作区路径后相同 | `vio_mean_fix/bounds_fix/algorithm-build-settings-identical.json` |
| Python 工具用例 | 10 个通过 | `vio_mean_fix/python-tests.log` |
| Google 布局 | 人工检查；本机没有 clang-format | `.clang-format` 使用 Google / C++17，另执行 `git diff --check` |

最终整套验证日志为 `final-suite.log`，汇总为 `acceptance.json`：24 个 C++ 用例、10 个 Python 用例、4 组零差异比较均通过；19 份正常退出记录的源码与二进制摘要已核对。
19 份记录包含调查期间数值比较失败的历史运行，只表示运行完整、输入与摘要有效；数值验收结论来自最新 4 组配对。
汇总还逐字核对最终生产源码与审阅版本的差异：只增加初始化尺寸校验、异常头和空指针初始化，其他生产源码、合法窗口的采样和状态表达式均相同。

24 个 C++ 用例为核心 14 个、旧入口兼容 2 个、PCL 内存兼容 1 个、缓存均值 2 个、采样与配置 5 个。
本机 `catkin_test_results` 显示 48，因为该版本累计 XML 的 suite 总数与子项；实际独立用例数按五个 XML 根节点计数。
P0 参考的 7 个用例为 PCL 1 个和 VIO 6 个，不含新核心测试。

## 数值差异调查

初始两次默认短回放正常退出，各有 296 个位姿，导出轨迹和状态文本与当时 P0 短参考相同。
完整回放同样正常退出，但出现差异。随后重复相同参考二进制，并控制 OpenMP，仍发现参考自身的重复性问题：

| 运行条件与比较对象 | 最大位置差（m） | 最大姿态差（°） | 报告 |
| --- | --- | --- | --- |
| 首次默认 P1/P0 完整配对 | 0.0316640549 | 0.0889367648 | `full_vs_p0.json` |
| 相同 P0 二进制默认完整重复 | 0.0255223343 | 0.1394732380 | `p0_parallel_repeatability.json` |
| 修复 VIO 前单线程完整配对 | 0.0266823167 | 0.0863724649 | `serial_full_vs_p0.json` |
| 仅修复均值后默认短重复 | 0.0039549373 | 0.0266537489 | `vio_mean_fix/fixed_short_repeatability.json` |
| 仅修复均值后单线程完整配对 | 0.0210170225 | 0.1067354047 | `vio_mean_fix/fixed_serial_full_vs_p0.json` |
| 仅修复均值后相同 P0 单线程完整重复 | 0.0238095179 | 0.0848919667 | `vio_mean_fix/fixed_serial_p0_repeatability.json` |

以上差异均保留，不放宽零阈值，也不进行位姿对齐或裁剪时间区间。
旧/新数学和状态的 20,000 组确定性输入没有系数差异；这项直接诊断不替代完整前端验证。

随后用真实 VIO 方法分别定位并验证：

- [缓存均值未初始化](vio-cached-mean-fix.md)：两个评分用例先失败、后通过。
- [粗层图像采样越界](vio-sampling-bounds-fix.md)：实际回放捕获越界；四个直接用例中两个无效测量用例先失败，修复后全部通过。

两项修复均独立于结构迁移提交。新配对参考为 P0 原数学/状态加上同样两项修复；候选为新核心头加上同样修复。
所有新配对回放统一 `OMP_THREAD_LIMIT=1`、`OMP_DISPLAY_ENV=TRUE`，节点日志确认运行限制，回放速度为 0.5。

## 新配对回放

完整参考和候选均正常退出，各有 1351 个位姿；输入 IMU 27447、点云 1355、图像 1355 全部匹配。
传感器时间及全部导出位姿相同，最大位置差 0 m、姿态差 0°，没有应用位姿对齐。
`mat_pre.txt` 的 2707 × 19 数值及 `mat_out.txt` 的 2701 × 20 数值也完全相同且有限。
证据为 `vio_mean_fix/bounds_fix/controlled_full_vs_p0.json` 和两份运行摘要。

参考及两次候选短回放均正常退出，各有 296 个位姿，输入 IMU 6081、点云 300、图像 300 匹配。
两次候选分别与参考比较，以及两次候选互比，传感器时间和导出位姿均相同，位置/姿态差为 0。
每对 `mat_pre.txt` 的 597 × 19 数值及 `mat_out.txt` 的 591 × 20 数值也全部相同。
比较报告为 `vio_mean_fix/bounds_fix/controlled_short_a_vs_p0.json`、`controlled_short_b_vs_p0.json`、`controlled_short_repeatability.json`。

当前代码的默认并行 45 秒回放也正常退出，输出 446 个位姿；输入 IMU 9113、点云 450、图像 450 匹配，状态文本全部有限。
节点日志的 `OMP_THREAD_LIMIT` 为 `4294967295`，没有采用单线程运行限制。此检查覆盖原问题区间，验证正常运行，不作并行零差异声明。
审阅版本证据为 `vio_mean_fix/bounds_fix/default_45/summary.json` 和 `mapping.log`。
启动约束修复后的最新代码也完成相同 45 秒检查，正常退出且输入匹配，另有 446 个位姿，见 `final-review/default_45/summary.json`。

## 独立审阅闭环

独立只读审阅发现 1 项 Important、0 项 Critical、0 项 Minor：奇数窗口仍可能越界。
配置用例先失败后通过，启动约束已独立提交为 `0bfb12a`；最新完整测试与默认片段验证通过后关闭审阅。
未重复派发审阅。原报告、完整配对时的 23 用例测试/汇总和最新 24 用例结果分别保留，避免混用不同二进制的测量结论。

## 结论边界

零差异指原导出精度内的行为一致，不能证明所有内部浮点状态逐位相同，也不证明默认并行模式逐位确定。
状态文本没有覆盖重力和完整协方差；直接状态用例验证这些量的初始化、复制、增量和重置。
输入观察者计数不能单独证明估计器处理次数，需结合轨迹时间、节点输出及正常退出判断。
没有真值轨迹，不报告 ATE/RPE；未做 1 倍实时验收、ARM 构建、Humble 或反向组合模式专项验收。
本机零阈值不能替代跨平台数值评估。
