# 几何完整构建累计优化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans.

**Goal:** 用真实点云证明并消除临时完整构建的逐帧体素表复制，保留公开增量更新的强异常保证。

**Architecture:** GeometrySubmapBuilder 的公开事务包装与私有原位累计共用同一算法；GeometryMapper 只在本地新建 builder 中使用私有入口。

**Tech Stack:** C++17、Eigen、PCL、GLOG、GTest、SDK / Noetic / Humble。

**Spec:** [本轮方案](../../geometry-build-reuse.md)。用户已批准真实点云基准、临时构建后统一发布及完整验证，沿用当前分支和本地提交授权。

## Global Constraints

只修改 livo-atlas；遵循 Google C++、函数间空行，不使用 Pimpl，不增加公共 API、数据字段、配置或功能宏。
逐点顺序、统计公式、快照排序、所有几何数值、异常类型与回滚、缓存预算保持不变。
公开入口在非法头部校验后复制，在副本上更新；私有入口只用于未发布 builder，所有异常向外传播。
基准与诊断不开启线程/分配审计探针的计时；分配次数使用独立单元测试。只本地提交、不推送；Humble 容器恢复停止。

## Review Focus

多帧后期非法点、点数/体素上限、分配失败不能破坏公开 builder 或已发布快照。
完整构建失败必须销毁部分累计结果，重试不能带入重复计数或错误表示版本。
私有入口不能用于 Append；新建的活动块后续增量更新仍走事务入口。
输入点序、协方差、法线符号、来源顺序和排序必须逐项比较，不能只比较点数。
离线计时排除加载与校验成本；公开入口与 mapper 路径互相独立对照，不能将局部收益宣称为全系统加速。

### Task 1: 离线基准与累计路径

Files: include/livo_atlas/mapping/geometry_submap.h、src/mapping/geometry_submap.cc、src/mapping/geometry_mapper.cc、tests/mapping/geometry_failure_test.cc。

- [x] 冻结修改前 SDK 安装及基准可执行文件；预载真实13块点云、位姿，交替多轮测量公开累计/快照与完整构建。Expected: 输入只读，mapper 与公开参考几何完全一致。
- [x] 扩展现有分配钩子，增加完整构建分配次数显著少于公开逐帧事务的测试。Expected: 旧实现超过宽松的1/2相对上限而失败。
- [x] 添加公开分配失败回滚、多帧构建中途失败/重试及快照不变检查。Expected: 先确认原保证，防止优化削弱异常安全。
- [x] 抽取私有头部校验及 AddKeyframeInPlace；公开入口使用副本和交换，Build 使用私有原位入口，Append 保持公开入口。Expected: 分配测试和全部几何测试通过，所有字段精确相同。

Interfaces: 不变的 AddKeyframe/Snapshot/GetSubmap；私有 AddKeyframeInPlace 在类外仅由友元 GeometryMapper::Build 调用，公开包装在自己的副本上调用它。

### Task 2: 重复对照与完整回归

Files: docs/geometry-build-reuse.md、docs/geometry-delivery-timing.md、docs/engineering-roadmap.md。

- [x] 用完全相同的已冻结基准源重新链接优化后的 SDK，多轮配对计时与输出几何指纹。Expected: 分配工作量下降，基准显示稳定收益，否则撤回优化并记录。
- [x] SDK 全套、Noetic/Humble 相关目标、安装后消费验证、完整 bag 与698cc6c冻结回放对照。Expected: 数值与资产、31次构建/51次读取/2次交付、位姿版本和归档一致。
- [x] 独立源码复核，记录 LIO/VIO 尾部、局部收益和限制，本地提交并清理仅本计划 scratch。Expected: 无重要未解决问题；Humble 恢复停止。

Interfaces: Task 2 直接复用 Task 1 的只读真实输入、冻结基准和几何指纹；新实现不能通过改变工作量或阈值获得收益。

结果：分配测试36925次→6164次（公开参考36921），三组7轮离线中位完整构建下降35.9%～39.0%。公开参考序列化SHA256一致，mapper逐字段数值相同（不区分正负零）；完整bag资产与数学诊断一致、归档consistent。SDK76/76、Noetic6/6、Humble8/8与安装后消费者通过。独立复核无重要问题；后续可加强常规分配用例中的非零协方差/法线覆盖。详细范围与限制见[结果记录](../../geometry-build-reuse.md)。
