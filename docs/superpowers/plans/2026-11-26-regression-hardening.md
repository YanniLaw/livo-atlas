# 回归工具工程收尾 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans.

**Goal:** 修复输出入口及阶段状态边界，归档构建版本与节点声明的线程配置。

**Architecture:** 继续使用顺序 Python 回归入口。CMake 将 find_package 实际解析的版本写为
构建目录中的键值文本；Python 读取并保存到阶段报告。现有 GLOG 分析器收集线程配置。

**Tech Stack:** Python3.8+、CMake3.16+；不修改生产 C++。

**Spec:** 用户已批准输出保护、阶段状态、数值依赖及线程记录三项收尾，直接在当前会话实施。
沿用当前开发分支、本地提交及主动验证授权。

## Global Constraints

不改 SLAM 算法、公开 SDK、ROS 入口、vikit、运行参数或依赖选择。
输出最终入口必须不存在；拒绝既有链接和悬空父链接，允许指向既有目录的父链接。
请求阶段先标 pending；前置失败用 blocked，中断未执行用 interrupted，未请求用 not_requested。
显式基准程序不依赖 SDK 安装成功；其他独立请求继续执行，验证阶段失败/中断时不精简载荷。
成功验证后的精简如被中断，明确报告清理可能部分完成，保留既有文件清单。
记录配置时解析的版本，不把它等同于运行时加载证明；旧构建缺少记录须明确 unavailable。
节点预算从日志读取，可记录多个不同实例；环境变量只代表回归进程，不推测实际线程团队。
不定义通用跨环境数值容差，不添加功能宏，不引入依赖安装或自动环境管理。

## Review Focus

既有目录、文件、悬空链接、父链接与路径别名不能改写已有数据。
SDK/consumer 失败后，隐式与显式 benchmark 的依赖关系必须正确。
中断的未执行请求不能被写成未请求，归档阶段中断也须保存准确总体状态。
记录必须来自当前 CMake 解析；缺失、坏 schema 或错误 OpenMP 值不能伪造有效版本。
配置预算与实际 OpenMP 团队不同；多个启动配置及旧日志不能被覆盖或补成默认值。

### Task 1: 输出保护及阶段状态

Files: tools/run_regression.py、tests/test_regression_tools.py。

- [ ] 写符号链接、阻塞、独立 benchmark、中断后续请求等复现测试。Expected: 旧实现失败。
- [ ] 在路径解析前检查入口；记录 requested/pending；传播真实依赖和中断状态。
  Expected: 原始数据不变、退出码及各阶段含义一致，旧成功路径保持可用。
- [ ] 跑完整工具自测并记录。Expected: 新旧用例通过。

### Task 2: 构建与线程记录及验收

Files: CMakeLists.txt、cmake/build_info.cmake、cmake/build_info.txt.in、tools/run_regression.py、
tools/timing_analysis.py、tests/test_regression_tools.py、docs/regression-tools.md、docs/engineering-roadmap.md。

- [ ] 写真实微型 CMake 元数据、缺失/坏记录、节点预算、多实例配置和旧日志测试。
  Expected: 缺少新接口/字段而失败。
- [ ] 配置时生成/安装 livo_atlas_build_info.txt；报告编译器、Eigen/PCL/OpenCV/GTSAM、
  目标处理器、ROS版本、构建类型及 OpenMP 能力；不存在的版本保留为空而不猜测。
  Expected: Python 报告和原始记录可关联，旧构建显式 unavailable。
- [ ] 分析器收集节点预算，统一报告保存固定白名单的父进程线程环境。
  Expected: 无记录时为空，多实例不覆盖，不声称测得实际团队大小。
- [ ] 实际 SDK 全部 CTest/安装消费者、Noetic/Humble 相关测试及短回放验证，更新文档。
  Expected: 版本及预算正确、工作区和输入安全；无需重复135秒算法回归。
- [ ] 一次独立只读复核，重要问题单次修正后验绿，本地提交并恢复容器停止。
  Expected: 无未解决重要问题，明确任何保留边界。
