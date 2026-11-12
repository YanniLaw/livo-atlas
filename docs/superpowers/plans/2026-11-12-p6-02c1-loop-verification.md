# P6-02c1 Geometric Loop Verification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans in this session. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 实现先验辅助的封存子地图候选检索、双向 GICP 与独立质量检查，提供只读数据集验证工具。

**Architecture:** 普通 `livo_atlas::loop_closure` 库只处理轻量元信息与 `GeometrySubmap`。
离线 I/O 放在独立源文件，复用现有 GeometryMapper；按候选顺序输出独立记录，不累计全部云或结果。
本阶段不向 PoseGraph 插入约束，不改动 ROS 节点或前端流程。

**Tech Stack:** Google C++17、Eigen、PCL 1.10/1.12、OpenCV FileStorage、GTest、Noetic/Humble。

**Spec:** [C1 设计](../specs/2026-11-12-p6-02c-loop-closure-design.md)。用户于本轮确认按推荐继续；
沿用当前分支、当前会话直接实施及自主验证的授权，不再次询问执行方式。

## 全局约束

- 只修改 livo-atlas；直接结构与成员，无 Pimpl；函数定义之间空行并补充坐标、拒绝条件注释。
- 默认 `LIVO_ATLAS_WITH_LOOP_CLOSURE=OFF`；开启后仅新增 PCL registration/kdtree 依赖，GTSAM 可独立关闭。
- 旧数据集、原始 bag 和 P6-02b 证据只读。新证据目录为 `validation/P6-02c1-20261112/`。
- 单任务 `nice -n 10`、`-j1` 构建，与回放串行，`OMP_THREAD_LIMIT=1`，不启动 RViz。
- C1 不实现在线调度、约束准入、图优化或全局 TF；零匹配/全拒绝是正常检测结果。
- 设计中的默认值、双端闭合差与初值修正公式原样采用，不为实包通过率放宽阈值。

## 审阅重点

1. 非交换旋转及大锚点间距：正反向变换、双端闭合和两端初值修正必须使用设计中的公式，Task 2 固定字面预期。
2. 元信息伪造/混合视图：版本、覆盖范围、成员来源、封存状态与重复 ID 在求解前拒绝，Task 1/2 覆盖。
3. 点数多但几何不可观：低残差平面/直线仍退化，互为最近邻避免重复计数，Task 2 覆盖。
4. PCL 到迭代上限也返回 converged：包装层必须拒绝，Task 2 运行真实 GICP 的上限场景。
5. 离线输出路径与错误：已有输出、输出位于输入中、损坏元信息和部分失败不得覆盖源数据，Task 3 覆盖。

## 文件与接口

| 文件 | 责任 |
| --- | --- |
| `include/livo_atlas/backend/loop_closure.h` | 搜索/配准选项、普通元信息、候选、来源与可选诊断 |
| `src/backend/loop_closure.cc` | 配置、输入与元信息验证、确定性候选排序 |
| `src/backend/loop_registration.cc` | 临时降采样、GICP 适配、结果判定 |
| `src/backend/loop_quality.cc`、`loop_internal.h` | 独立互为最近邻、退化及变换误差计算；内部接口供数学测试 |
| `src/io/loop_verifier.h/.cc` | 只读数据集验证、配置与逐候选 YAML 输出，内部工具接口 |
| `tools/verify_loops.cc` | CLI 参数与退出码 |
| `cmake/loop_closure.cmake` | 可选库、工具、安装；更新三种包导出的条件依赖 |
| `tests/backend/loop_*_test.cc`、`tests/io/loop_verifier_test.cc` | 行为与数据集边界 |
| `tests/installed_loop_consumer/` | 独立安装消费者，验证开关及静态传递链接 |

普通 API 固定为：

```cpp
std::vector<LoopCandidate> FindLoopCandidates(
    const LoopSubmapInfo& query, const std::vector<LoopSubmapInfo>& history,
    const LoopSearchOptions& options = {});
LoopVerification VerifyLoop(const GeometrySubmap& target,
                            const GeometrySubmap& source,
                            const LoopRegistrationOptions& options = {});
```

`LoopSubmapInfo` 持有 OnlineSubmap、首末时间、父系锚点位姿和 pose/frame/optimized 前缀信息；
`LoopCandidate` 只持 source/target ID 与距离。VerifyLoop 从同视图的两份几何计算初值。
`LoopVerification` 保留两端成员/几何修订与共同位姿视图、正反求解结果、可选质量/误差与耗时。
只有 `kVerified` 才提供 `target_from_source`；缺少诊断时用 optional，不用零误差伪装未计算。

搜索默认值为 gap=3/50/30 s、半径15 m、最多5候选。配准选项完整采用设计默认值；
计数选项验证正值与相互关系，几何上限最多100000、降采样云最多20000；所有实数有限，
比例在(0,1]、旋转角在(0,180]。非法输入抛异常，正常失败返回稳定枚举原因。

状态（2026-11-13）：Tasks 1–4 已完成阶段验收，产品代码为 `64054a7`。
[验证记录](../../p6-02c1-validation.md)列出全部平台证据和三个延期 Minor；
下方未勾选行专门保留测试覆盖缺口，不能解释为已验证。

### Task 1：候选检索与可选构建

**Consumes:** OnlineSubmap、RigidTransform、轻量时间与位姿视图。
**Produces:** LoopSearchOptions、LoopSubmapInfo、LoopCandidate、FindLoopCandidates；条件安装目标。

- [x] 先写 `loop_candidates_test`：距离排序与 ID 平局、top-5、各 gap/半径的含等号边界、空历史；
  future/self/相邻/共享成员不能入选，非法时间、重复 ID 和混合视图拒绝。运行看到缺少 API 的失败。
- [x] 实现头和搜索：历史可不连续但 ID 唯一且早于 query，逐项验证后筛选；所有旋转校验后投影副本。
  查询和历史必须为封存块；共享成员等属于不合格候选，损坏成员/视图属于异常。
- [x] 新增条件库和配置分支，OFF 不创建可选目标、不发现 registration；ON 支持普通头独立编译。
  catkin 的可选库排在依赖前；SDK/ament 条件查找 registration/kdtree；GTSAM 保持独立选项。
- [x] 运行候选测试及已有完整核心测试，提交 `feat: add optional geometric loop candidates`。

### Task 2：双向 GICP 与独立质量判定

**Consumes:** Task 1 普通选项/验证、GeometrySubmap。
**Produces:** LoopRegistrationOptions、LoopVerification、VerifyLoop；内部质量计算接口。

- [x] 先写质量测试：多对一不能增加互为最近邻计数；0.5 m 半径以平方距离比较，p95 采用
  升序第 `ceil(0.95*N)-1` 项；单位法向的平面/直线退化，三维面组满秩。
  用固定非交换变换和非零平移分别验证 `F*B`、`B*F`、`inverse(I)*F`、`I*B`。
- [x] 写真实配准测试：不对称三维场景、已知旋转加平移与接近初值，恢复误差<=0.02 m/0.2°；
  保持原几何逐系数不变。覆盖同正版本、版本/前缀不符、sources不符、非法法向/点、极少点、
  部分重叠、离群点、输入预算、单次最大迭代、无有效法向及低残差平面。
- [x] 验证 RED 后实现：严格输入检查与范围安全的 floor 降采样，局部派生 helper 只读取 GICP
  迭代数；独立审阅后还包装求解更新以检查 float 运算包络。GICP 模板仅在源文件内；双向独立求解。超过迭代上限等早期失败保留已有诊断。
- [x] 质量计算使用原体素质心/法向，互为最近邻，双方向分别输出 RMSE/p95/覆盖和归一化 H；
  按点数、求解状态、重叠、残差、闭合、修正、退化顺序给出稳定首个拒绝原因。
- [x] 运行新增测试与全核心测试；Google 格式检查，提交 `feat: verify submap loops with geometric evidence`。

### Task 3：只读离线数据集工具

**Consumes:** Tasks 1/2、KeyframeStore、OnlineSubmapStore、GeometryMapper。
**Produces:** 安装工具 `livo_atlas_verify_loops DATASET OUTPUT --pose-version 0 [--config FILE]`。

- [x] 为 KeyframeStore 增加 `double ReadTimestamp(uint64_t id) const`；先测缺少图像/点云仍可读，
  非有限时间和 ID 不符拒绝。元信息解析复用已有私有 reader，不将 YAML 布局复制进 CLI。
- [ ] 先写 `loop_verifier_test`，创建小型真实关键帧存储和封存索引，覆盖零候选、一次几何判定、
  全拒绝、已有输出/输出在输入内部拒绝、损坏输入、配置非法和中途资产失败。
  比较原文件字节；完整记录应标 `complete=1`，部分失败保留可诊断输出而不冒充完整。
  **主要场景已覆盖；前序匹配记录保留及损坏数据集元信息的直接测试仍为延期 Minor。**
- [x] 实现内部 I/O：一次读取轻量时间/位姿、原有 GeometryMapper 缓存最多3块，query与单target引用；
  候选逐个写 `matches/TARGET-SOURCE.yml`，不在内存累积全部结果。
  `summary.yml` 记录 schema=1、complete、全部实际选项、候选/状态计数；失败输出 `failure.yml`。
  输出必须新建，拒绝位于输入目录内；工具只支持显式版本0，非法 CLI 返回2，运行失败返回1。
- [x] YAML 配置分 search/registration 两组，支持公共选项全部字段并拒绝未知键和非法类型。
  I/O 编译进内部静态目标供工具与测试复用，不新增无消费者的公共 I/O API。
- [x] 运行工具测试、完整核心和实际 CLI；提交 `feat: add read-only loop verification tool`。

### Task 4：安装、双 ROS 与真实数据验收

**Consumes:** 最终冻结源码、Retail_Street.bag 和 P6-02b 同环境基线。
**Produces:** docs/loop-verification.md、docs/p6-02c1-validation.md 与独立证据。

- [x] 普通 SDK shared/static 的 OFF/ON、无OpenMP ON 与独立安装消费者；额外 GTSAM+loops 同开
  运行既有图测试，确认两个开关独立。OFF 消费者不得出现可选目标。
- [x] 冻结最终源码副本；Noetic/Humble 各自 shared/static OFF/ON，检查新测试、实际节点测试、
  安装消费者及实际依赖。重型构建顺序执行，不覆盖 P6-02b 目录。
- [x] 用同一版本0数据集在 Noetic/Humble 分别运行离线工具，逐对保留全部质量与拒绝原因；
  对照只读分析的22粗候选/20候选上限；不得要求跨 PCL 版本数值逐字节一致。
- [x] 两端共享 ON/OFF 完整 bag 与短导出回放，固定单CPU与OMP线程上限，比较输入、四数组、
  发布、关键帧、在线索引和旧导出；采样实际节点库，C1库不自动进入普通 ROS节点。
- [x] 新功能只交付几何证据；真实回环准确率/召回率无标注时不作结论。记录点数、耗时、迭代、
  RSS、数据摘要和有限工作集局限。文档链接和 git diff --check 通过。
- [x] 整体独立代码审阅一次，修复重要问题后验证相应回归；提交最终验证文档，保留当前分支。

## 自审结论

设计的检索、坐标、双端误差、退化和只读边界分别由 Tasks 1–3 覆盖；Task 4 验证包边界。
为了保持质量计算和工具代码集中，增加一个内部质量源文件和内部 I/O 目标；不引入公共框架。
只读时间查询是 CLI 避免解码图像所需的最小新增存储 API。C2/C3 不在本计划内。
