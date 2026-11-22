# 回环准入与离线图试算

P6-02c2a 在 C1 几何验证之后增加多候选一致性、相关约束去重、鲁棒位姿图试算及优化后几何复核。
普通 C++ 接口返回提案，由调用者决定是否接纳。离线工具顺序接纳到自己的内存视图，保存完整诊断，
不修改输入数据集。实测范围与结果见 [C2a 验证记录](p6-02c2a-validation.md)。

本页描述独立的离线准入接口。ROS 节点现已通过 [C2b 在线协调层](online-loop-backend.md)
接入封存调度和持久接纳，启动时默认关闭；全局 TF 仍属于后续阶段。
后续在线 3DGS 可使用统一位姿版本和重建后的几何，但本阶段没有 Gaussian 优化器。

## 构建与安装

常规构建生成 `livo_atlas::loop_backend` 和 `livo_atlas_propose_loops`。

```bash
cmake -S /path/to/livo-atlas -B /path/to/build \
  -DLIVO_ATLAS_ROS_VERSION=0 -DBUILD_TESTING=ON \
  -DGTSAM_DIR=/path/to/gtsam/lib/cmake/GTSAM \
  -DSophus_DIR=/path/to/compatible-sophus-config \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/path/to/install
nice -n 10 cmake --build /path/to/build -- -j1
(cd /path/to/build && OMP_THREAD_LIMIT=1 ctest --output-on-failure)
cmake --install /path/to/build
```

外部 CMake 消费者通过 `find_package(livo_atlas CONFIG REQUIRED)` 和
`target_link_libraries(my_target PRIVATE livo_atlas::loop_backend)` 使用普通接口。
公共头不包含 ROS、GTSAM 或 PCL registration 类型。静态工程库使用 `BUILD_SHARED_LIBS=OFF`；
关闭 OpenMP 使用 `LIVO_ATLAS_USE_OPENMP=OFF`。
安装消费示例见[测试工程](../tests/installed_admission_consumer/CMakeLists.txt)。
Noetic 和 Humble 的工作空间准备见[双 ROS 构建](ros-builds.md)。

## 运行已结束的数据集

输入必须包含 `dataset.yml`、`frames/` 和 `online_submaps/`，成员索引完整且所有子地图已封存。
本工具从原始版本 0 和空回环历史开始；不能继续未知的外部优化历史。
输出目录必须尚不存在，并位于输入数据集之外。

```bash
OMP_THREAD_LIMIT=1 nice -n 10 /path/to/install/bin/livo_atlas_propose_loops \
  /path/to/keyframes /path/to/new-proposals --pose-version 0
```

工具按子地图 ID 遍历查询。每次试算都包含已结束数据集的全部 N 个关键帧和原始里程计边，
而候选搜索只允许查询之前的历史子地图。因此这是固定完整数据集上的顺序批处理，
不能用来证明在线逐帧可见性、处理延迟或过期结果提交的正确性。超过图预算时显式拒绝，不裁剪 N。

几何从原始 body 点云按请求的位姿快照重建，不读取图像。缓存最多 3 块几何子地图，
切换当前/预览版本时清空旧缓存；体素大小等固定几何参数写入摘要。
缓存容量不包含调用者持有的引用、临时图、PCL/GTSAM 工作内存，也不等于进程 RSS 上限。

退出码 0 表示遍历完成，包括零提案；命令行语法错误为 2，运行失败为 1。
重试必须使用新的输出目录。

## 配置与判定

`--config FILE` 接受 OpenCV YAML，可配置 `search`、`registration`、`graph`、`admission` 四组。
省略字段保留默认值，未知键、重复键/组、非数值、非有限值和非法整数均被拒绝。
完整字段见[准入头](../include/livo_atlas/backend/loop_admission.h)、
[几何头](../include/livo_atlas/backend/loop_closure.h)和[位姿图头](../include/livo_atlas/backend/pose_graph.h)。
例如：

```yaml
%YAML:1.0
---
search:
  search_radius_m: 15.0
  max_candidates: 5
registration:
  match_distance_m: 0.5
graph:
  loop_huber_delta: 1.345
  max_iterations: 100
admission:
  loop_translation_sigma_m: 0.2
  loop_rotation_sigma_rad: 0.05235987755982989
  max_factor_residual: 3.0
  max_keyframes: 5000
  max_loops: 200
```

一轮查询依次执行下列检查，前级拒绝后不执行后级：

| 检查 | 规则与默认值 | 拒绝状态 |
| --- | --- | --- |
| 数据和预算 | 验证快照、成员、raw 前缀和历史；最多 5000 帧、200 条回环 | 非法输入抛异常；超预算 `resource_limit` |
| 候选几何 | 沿用 C1，最多 5 个候选 | `no_candidate` / `geometry_rejected` |
| 多位置歧义 | 所有通过项推导的查询世界位姿两两相差 ≤0.5 m、5° | `ambiguous` |
| 相关性 | 任一端主成员相交，或对应两端子地图 ID 差都 ≤2 | `correlated` |
| 优化 | 有限且不增加的同图目标，未到迭代上限 | `optimization_rejected` |
| 因子残差 | 所有里程计和回环的原 sigma 白化范数 ≤3；历史回环增加 ≤0.5 | `constraint_residual` |
| 位姿改变量 | 每帧相对当前快照 ≤3 m、20° | `pose_correction` |
| 预览几何 | 历史与新约束的测量、预测相对位姿分别通过固定变换双向质量检查 | `geometry_changed` |
| 全部通过 | 仅此状态提供 `LoopProposal` / `PoseUpdate` | `proposed` |

候选通过项按最小覆盖率降序、最大 RMSE 升序、锚点距离升序、目标 ID 升序选择。
最优项相关时直接拒绝，不回退到次优项。此规则保守，可能丢掉有效约束。
阈值包含等号，具体质量门限沿用 [C1 说明](loop-verification.md)。

回环使用固定标准差：平移 0.2 m、旋转 3°，不从 RMSE 或体素协方差估计测量噪声。
Huber 对整个白化六维回环残差加权，默认 delta=1.345；先验和里程计仍为原高斯模型。
通用 `PoseGraphOptions::loop_huber_delta=0` 保持旧 Gaussian 行为，但准入接口强制 delta>0。
这些值是实验起点，不能解释为六维置信概率或误检率保证。

残差门限使用实际因子的 `unwhitenedError()` 除以原始 sigma，避免鲁棒降权把坏约束伪装成小残差。
位姿平移改变量直接取世界坐标差，旋转比较使用投影后的合法旋转，避免原始矩阵舍入误差缩放门限。
`EvaluateLoopGeometry()` 只检查给定变换及其逆，不重新配准，不把人为互逆的闭合误差当作独立证据。

## 普通 C++ 调用契约

`ProposeLoopUpdate(input, reader, options)` 顺序调用 reader，不启动线程、不写资产或提交正式位姿。
输入包含当前 `PoseSnapshot`、连续完整原始位姿、封存元信息、查询 ID 和已接纳约束。
版本必须等于已接纳条数；版本 0 必须与原始矩阵及平移逐项一致，后续版本遵守投影尾帧策略。
既有约束的 sigma 必须与当前配置一致。会话内配置保持固定。

reader 按指定快照返回几何；成员、锚点、视图字段和逐源局部位姿都必须匹配。
仅改旧几何的版本号不合法。缓存重建后的 representation revision 可变化，不能将它当全局唯一身份。
reader 应拥有自己的缓存并读取稳定资产，不能捕获正在变化的前端系统。

合法拒绝只有诊断，没有可提交更新。有效图的数值求解失败转换为 `PoseGraphOptimizationError`，
准入返回 `optimization_rejected` 并保存原因；非法输入、reader/I/O 错误、分配失败和未知异常向外传播。
详细所有权和未来在线提交要求见 [C2 设计](superpowers/specs/2026-11-13-p6-02c2-loop-admission-design.md)。

## 输出与失败恢复

所有记录使用 schema 1：

- `summary.yml`：`complete`、`frontier=closed_dataset`、帧/封存块/查询/提案数、最终内存版本、全部实际选项和状态计数。
- `queries/ID.yml`：查询前后版本、成员、C1 候选诊断、可选图试算、各因子残差和固定变换几何检查。
- `proposals/VERSION.yml`：约束、两端成员、原验证结果及完整候选位姿更新。
- `failure.yml`：中途失败原因；保留已写记录，摘要保持 `complete=0`。磁盘无法写入时报告本身也可能失败。

图试算记录 `graph_build_seconds` 与 `solve_seconds`，分别覆盖输入图装配/构造/初始目标计算和 LM 构造/求解。
查询总耗时另记 `elapsed_seconds`。计时不参与准入，未执行指标明确缺失。
只有 `complete=1` 表示完整运行；文件中存在 proposal 不等于正式地图已提交。
工具不写输入的任何位姿版本文件，也未提供从部分输出恢复继续提交的机制。
