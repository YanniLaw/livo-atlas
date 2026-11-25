# 可复现回归入口 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans.

**Goal:** 将几何基准、日志汇总与现有 SDK/ROS/安装消费/可选回放组织成可跨电脑运行的轻量入口。

**Architecture:** 一个安装的 C++ 基准、一个标准库日志分析模块和一个顺序 Python 调度脚本，复用既有测试、消费者和 ROS 回放器。

**Tech Stack:** C++17/GLOG/GTest、Python3.8+、CMake3.16+/CTest；配置转接可选 PyYAML。

**Spec:** [本轮设计与使用](../../regression-tools.md)。用户已批准三项整理工作和直接实施，沿用现有开发分支及本地提交授权。

## Global Constraints

只修改 livo-atlas；不改算法、公共 SDK、参数架构、ROS共用主入口、vikit 或现有验证资产。
遵循 Google C++、函数定义间空行，不引入 Pimpl、内部功能宏、CI框架或自动依赖安装。
工具参数不得包含本机绝对路径、固定帧数/块数/位姿版本；版本0的明确缺省不推测最新版本。
未执行不等于通过；非零退出、无测试、坏报告、失败 GTest XML 和缺失请求依赖都必须如实报告。
只精简本次新输出且全部成功的回放载荷；先保存哈希清单，不修改输入或跟随符号链接。
Humble 容器恢复停止，只本地提交、不推送。

## Review Focus

基准不得计入加载/比较/指纹，不以大参考地图突破驻留预算；预加载上限明确不是整个进程内存上限。
公开参考与 mapper 比较所有字段及位表示，指纹仅提示，不能以64位碰撞代替逐项比较。
CTest零用例和 catkin包装返回0但XML失败不能冒充通过；子集范围、不可用环境和依赖阻塞需可见。
数据目录、路径含空格、既有输出、符号链接和失败产物的保留不得误删数据。
复用旧回放参数时输出路径归统一入口管理；校验完成/归档前不可压缩载荷。

### Task 1: 正式几何基准

Files: tools/geometry_benchmark.h、tools/geometry_benchmark.cc、tools/benchmark_geometry.cc、tests/geometry_benchmark_test.cc、cmake/core.cmake、cmake/tests.cmake。

- [x] 新增小数据集测试，先确认入口缺失而失败，覆盖3帧2块、版本0/1、轮数、非法参数、封口、资源上限和只读。Expected: 旧树无法提供新入口。
- [x] 实现只读、逐块双路径基准和小型JSON，预加载限制、所有字段精确比对、无大体积输出；注册并安装基准。Expected: 数据规模可变，时钟范围清晰，stdout可解析JSON。
- [x] 运行新测试及既有几何相关目标。Expected: 全通过，SLAM库无行为修改。

Interfaces: 安装bin/livo_atlas_benchmark_geometry，JSON schema1含工作量、两条指纹、匹配状态和每轮样本；私有工具入口RunGeometryBenchmark(vector<string>)供main及测试使用。

### Task 2: 分析与回归调度

Files: tools/timing_analysis.py、tools/analyze_timing.py、tools/run_regression.py、tests/test_regression_tools.py、cmake/tests.cmake。

- [x] 先写日志分析、真实小CTest项目、不可用环境、空/失败报告、保留策略与输出隔离测试。Expected: 新模块缺失或契约未实现而失败。
- [x] 实现标准库流式分析和明确请求范围的顺序调度，复用消费者/ROS回放器，记录命令日志与来源，成功后先清单再精简。Expected: 每个失败/未执行状态可见，退出码反映结果，既有数据不变。
- [x] 将标准库自测注册CTest，验证工具help可在无ROS环境运行。Expected: 单独与CTest均通过，原回放工具无需修改。

Interfaces: runner使用Task1安装基准；若不构建SDK，则显式提供基准路径。日志/摘要JSON保留，外部路径全部由参数或CMakeCache得来。

### Task 3: 实机验证与文档

Files: docs/regression-tools.md、docs/core-build.md、docs/engineering-roadmap.md、README.md。

- [x] 统一入口实际运行SDK及安装消费者、Noetic/Humble相关测试与真实基准。Expected: 来源、测试数和未请求范围正确，Humble不假称完整回放。
- [x] 原生Noetic回放验证compact策略，确认源数据/旧结果不变，摘要/日志/指纹仍在；all策略用测试和真实原数据基准验证。Expected: 成功检查后才精简，失败保留。
- [x] 独立源码复核、跨电脑命令与限制文档、本地提交。Expected: 无重要问题，容器恢复停止，工作区干净。

Interfaces: Task3消费两项工具固定CLI和JSON，不能在验证时靠修改旧验证脚本才能运行。
