# P5-01：关键帧快照与数据集验证

本阶段从用户分支 `refactor2` 的 `c84368f` 开始，实现默认关闭的关键帧选择、
独立图像/点云快照、目录式保存和重读，以及独立的优化位姿版本。
设计和任务边界见[设计](superpowers/specs/2026-10-26-p5-keyframes-design.md)与
[计划](superpowers/plans/2026-10-26-p5-keyframes.md)，使用方法见[数据集说明](keyframe-dataset.md)。

## 实现边界

- `mapping/keyframe` 和 `keyframe_selector` 属于普通 `livo_atlas::keyframes` 库。
- `io/keyframe_store` 负责 schema 1 数据集与位姿版本，复用 PCL/OpenCV，无新依赖。
- `LivoSystem` 统一组合，Noetic/Humble 保持同一 ROS 入口与两端对称接入职责。
- `FrontendResult` 增加云源时刻和有效图像时刻；估计、同步、发布及原有导出的计算流程保持。
- 新检查工具安装到 `bin/livo_atlas_inspect_keyframes`；两个回放工具增加 `--save-keyframes`。

新的数据集保存未叠字的缩放 BGR 图像、body 坐标下的独立云、原始位姿和固定标定。
优化版本只增加 `poses/<version>.yml`，不修改原始帧。
尚未实现 GTSAM、回环、子地图管理或新的重建导出。

## 执行环境与证据

本机 Ubuntu20.04/Noetic；已有 `ros2` 容器中的 Ubuntu22.04/Humble。
所有新产物位于工作区 `validation/P5-01-20261026/`，容器写入根目录为
`/tmp/livo_atlas_p5_01`。旧阶段证据只读。
沿用已有 Sophus 和官方 Livox 消息定义，兼容范围与[双版本说明](ros-builds.md)一致。
构建使用 `nice -n 10`、`-j1`，与 bag 回放串行，不启动 RViz。

输入 `Retail_Street.bag` 为 1,961,140,106 字节，SHA-256：

```text
1745bee1d570749679bad7a0392100477ab62d50aa77f7f629bc57f2a72e4427
```

同环境数值参考为冻结的 `P4-naming-20261025`；本阶段基线源码树与该参考一致。
两种环境分别对照自身参考，跨系统读取另作检查，不要求不同依赖版本产生相同估计值。

## 已执行的自动检查

| 检查 | 结果 |
| --- | --- |
| 改动前普通核心基线 | 152 项通过 |
| 最终普通核心 | 180 项通过 |
| Noetic 单元 / 真实 ROS 回调 | 200 / 8 项通过 |
| Humble C++（含真实 DDS） | 216 项通过 |
| shared / static SDK 单元与公开头独立编译 | 各 180 项 / 42 个头通过 |
| 关闭 OpenMP 的 SDK | 不编测试，移动安装前缀后实际 LO/LIO/VIO 与数据集消费通过 |
| Humble ament SDK、实际 colcon 节点 | 安装迁移消费、实际运行库检查通过 |
| common / io 独立构建、catkin 外部消费 | 通过 |
| Noetic / Humble 实际进程启动检查 | 50 / 16 项通过 |
| Python 工具 | 本机 13 项通过、1 项版本跳过；Humble 对应 5 项通过 |
| 检查工具 | 字面数据集、只读、参数错误、缺失数据、未知 schema 通过 |

新增 28 项普通行为测试覆盖：阈值/重复/时间回退，ROI 图像与云独立性，非平凡旋转下的
坐标字面值，精确 double 时间往返，图像与点云损坏，已有文件保护，写失败和位姿版本。
系统测试逐项比较三种模式开启/关闭的观测，后续前端更新不会修改已存首帧。
安装后的真实外部程序连续运行 LO/LIO/VIO 并保存、重开、变换关键帧和写读版本。
独立库编译命令和消费者链接检查均不含 ROS 或 vikit。

新增接口在实现前的真实失败分别保存于 `model-red.log`、`store-red.log`、
`integration-red.log`；`frontier-red.log` 记录旧位姿版本被新增原始帧错误失效的问题，
其回归测试已通过。检查工具的缺失可执行文件也先作为失败证据记录。

## 官方 bag 对照

四组回放均开启 `--save-keyframes`。首次验收全部通过；独立评审修复后已使用最终节点
重新执行四组，旧数组、发布、产品与全部关键帧检查再次通过。

| 环境 / 输入 | LIO / VIO | 带图像关键帧 | 点数 |
| --- | --- | --- | --- |
| Noetic / 3 秒 | 26 / 25 | 2 | 40,086 |
| Noetic / 136 秒 | 1,351 / 1,350 | 172 | 3,763,228 |
| Humble / 3 秒 | 26 / 25 | 2 | 40,086 |
| Humble / 136 秒 | 1,351 / 1,350 | 172 | 3,762,300 |

四组的旧数组和发布语义均与各自冻结参考精确相同，两端短段各 57 个原有产品逐字节一致。
新关键帧经实际 C++ 读取器读取，并核对时间、位姿与原状态导出的关联。
Humble 数据集另用本机 Noetic 构建的读取器检查，验证格式能跨这两个环境消费。
两端完整数据集各 517 个文件。各端完整数组尺寸如下，最大绝对差异均为 0：

| 文件 | 行 × 列 |
| --- | --- |
| trajectory.tum | 1,351 × 8 |
| mat_pre.txt | 2,707 × 19 |
| mat_out.txt | 2,701 × 20 |
| imu.txt | 27,365 × 7 |

关键帧的 cloud_timestamp 与轨迹中对应 LIO 时刻的最大差异为
`1.1920928955078125e-7` 秒，来自原轨迹文本的打印精度；新数据集元数据本身保留精确 double。
位姿关联按原 mat_out 的六位有效数字核对，这不改变上述旧数组零容差回归。
两种依赖环境的点数差异沿用原估计结果，未将跨环境差异当作同环境回归容差。

## 复现和迁移

先按 [core-build.md](core-build.md) 或 [ros-builds.md](ros-builds.md) 在目标机编译。
按[数据集说明](keyframe-dataset.md)的两端回放命令，选择新的输出目录并增加
`--save-keyframes`；完整输入使用 `--duration 136 --rate 0.5`，短段使用
`--duration 3 --rate 0.5 --save-products`。两端保存原数组、发布摘要和数据集。
停止写入后可整体复制 `keyframes/`，在目标机运行安装的检查工具逐帧读取。

本轮包外验证脚本记录本机和容器的实际源码、节点与 14 个自有动态库摘要，回放前后核对；
`compare_same_environment.py` 严格比较四数组、发布摘要和旧产品，
`verify_keyframes.py` 用真实 C++ 读者消费每个资产，再检查原状态/时间关联。
这些辅助证据需与 bag 单独搬运，不进入源码仓库。

## 执行决定与收尾

每个优化版本冻结写入时的帧前缀，新增原始帧不会使旧版本失效。
消费者须按 `ReadPoses(version)` 返回的 ID 集合消费；若把后来的所有帧都当成旧版本，
会混用不同代的位姿。此决定已由真实回归测试验证。

一次 fresh 整阶段独立评审检查了设计边界、原始 XML、175 个源码摘要、四组数值对照，
并实际重读 Humble 的 172 帧。发现两项 Important，均由本会话一次修复：

- PCD 预检查曾接受重复 POINTS 和带后缀的关键字，而 PCL 会按前缀解释；
  现在只接受本格式的精确关键字并拒绝重复。回归测试在地址空间上限 4 GiB 的子进程
  中用微小文件声明 32 GB 分配，确认在自身头检查中拒绝。
- 点云曾只验证有限 XYZI，未检查尺寸。现在要求 width/height 为正且 uint64 乘积等于点数，
  写入前拒绝无效形状，ID 和帧目录不变，之后仍能成功提交有效帧。

两项真实 RED 见 `review-red.log`，完整普通测试 180 项以及 Noetic/Humble、SDK、
修复后的四组实包均已通过。最终业务源码提交为 `e3a9648`，后续收尾只更新文档。
PCL 行为依据[该版本读取器实现](https://raw.githubusercontent.com/PointCloudLibrary/pcl/pcl-1.10.0/io/src/pcd_io.cpp)核对。
未派发第二次评审。

延期 Minor：部分输入验证异常尚未带数据集/帧/版本路径。它们仍会在输出变化前拒绝输入，
实际读写异常保留路径上下文；同时处理多个数据集时，前置验证的诊断可能有歧义。

评审未纳入的四项范围由执行者明确保留：一份数据集仅单写者串行调用、索引不自动刷新
（并发使用可能覆盖或读到旧索引）；关闭文件加 rename 不承诺 fsync/断电事务
（可能丢失最近数据）；跨依赖环境以数据可读为契约，不保证重新估计逐位一致；
只有 XYZ/强度有物理含义契约，不能把附带的 normal/curvature 用作已变换的法向或其他测量。

最终源码/运行库摘要与完整历史 bundle 恢复结果在包外阶段证据中记录。
保留当前开发分支，不自动合并、推送或修改已有 tag/master。

收尾备份文件为工作区 `livo-atlas-p5-01-20261026.bundle`，包含所有引用的可达历史。
在另一台电脑建立源码副本可用：

```bash
git clone -b refactor2 /path/to/livo-atlas-p5-01-20261026.bundle /path/to/new-workspace/src/livo-atlas
```

精确保留全部引用的恢复检查使用新建空 bare 仓库，fetch `+refs/*:refs/*` 后比较全部引用、
历史计数和最终 HEAD，再运行 `git fsck --full`。实际结果与 bundle 摘要保存于
`validation/P5-01-20261026/recovery.json`。bag、数据集和包外验证证据需另行复制。
