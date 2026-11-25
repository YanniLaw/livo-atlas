# GICP CPU 采样与异常计时 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans.

**Goal:** 用 CPU 栈采样解释迭代成本，补齐估计器异常耗时，并据重复对照决定是否继续优化。

**Architecture:** 保留当前私有 Gicp 子类和 PCL 求解路径。采样工具仅放在验证目录；产品只做证据支持的小改动。

**Tech Stack:** C++17、PCL 1.10/1.12、GLOG、gperftools、ROS1 Noetic / ROS2 Humble。

**Spec:** [上一轮的结果与后续方向](../../gicp-covariance-reuse.md)。用户已批准当前分支直接执行、外部下载、测试及本地提交。

## Global Constraints

保持精确搜索、点序、迭代限额、质量阈值、公开接口和归档格式；关闭计时不新增读时钟。
不安装系统软件、不改变内核权限；不修改 vikit；不为优化复制 PCL 迭代器。
所有函数定义间空一行，注释解释必要的边界条件。只本地提交，不推送。

## Review Focus

异常类型与 PCL 捕获路径保持一致；计时样本不得遗漏失败调用或重复计数。
采样与非采样运行分开报告；父子阶段的累计时间不能重复相加。
SDK 与两种 ROS 构建均验证，实际 bag 回放只在 Noetic 上进行。
短 bag 和一次配对不能证明稳定收益；没有明确收益则停止增加复杂度。
原始资产和全部数学诊断保持一致，空间紧张时仅去重新生成的不可变资产。

### Task 1: 采样与异常计时

- [x] 在验证目录解包用户态 profiler，冻结现有运行库，以完整 bag 采样；Expected: 可解析的 CPU 栈和一致的归档。
- [x] 在现有 loop_task_test 或 loop_registration_test 添加真实 PCL 估计器抛异常的用例，对比计时开关的求解结果；Expected: 旧代码缺失异常调用计时而失败。
- [x] 最小化补齐异常计时，原样重新抛出；Expected: 用例及 SDK 全套 CTest 通过。

Interfaces: src/loop_closure/loop_registration.cc 的估计器包装、私有 timed VerifyLoop，测试只使用现有私有接口。

### Task 2: 证据驱动的收敛

- [x] 分析采样结果；只有无需改变算法语义的明确热点才加优化，并先有失败测试。Expected: 可追溯的实施或不实施理由。
- [x] 至少两组同预算、同 CPU 的非采样完整 bag 配对，对比数学诊断、原始资产及阶段耗时；Expected: 正确性一致，明确波动范围。
- [x] Noetic/Humble 相关测试、独立源码复核、结果文档及本地提交。Expected: 测试通过，重要问题已修复，Humble 容器恢复停止。

Interfaces: Task 1 的异常计时改变估计器样本数量，Task 2 必须用新的完整样本解释成本，不将旧的正常返回小计视为全部估计器成本。

证据目录：validation/gicp-cpu-20261125。每组结束后校验哈希并硬链接去重新生成的原始帧资产，保留日志和诊断。

结果：真实失败用例 RED→GREEN，SDK 76/76、Noetic 3/3、Humble 5/5；两组配对全部数学诊断与资产一致，修复版每次新增两条失败估计计时。耗时变化方向相反，不宣称加速。独立复核无重要问题；可后续补强固定失败状态断言。结果与取舍见 [采样记录](../../gicp-cpu-profile.md)。
