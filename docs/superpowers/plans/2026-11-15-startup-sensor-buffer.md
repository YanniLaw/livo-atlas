# Startup Sensor Buffer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans in this session. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 防止首帧激光与 IMU/图像的回调先后改变初始化输入，并保留旧行为的明确开关。

**Architecture:** `LivoSystem` 有界暂存首帧前 IMU，图像进入有界的既有队列；首个有效扫描到达
后用其时间锚点处理暂存 IMU。共享参数读取和两端节点只负责配置及报告，不增加另一套 ROS 同步器。

**Tech Stack:** C++17、Eigen、OpenCV、GTest、ROS1 Noetic、ROS2 Humble。

**Spec:** [首帧传感器暂存设计](../specs/2026-11-15-startup-sensor-buffer-design.md)。

## Global Constraints

- 仅改 livo-atlas；保留 `framework_completion` 本地分支，不合并、推送或打 tag。
- Google C++17、直接成员、函数实现间空行；双 ROS 仍使用 `src/app/livo_main.cc`。
- `common.buffer_before_first_lidar` 默认 true；false 恢复原始首帧前拒绝行为。
- 首帧前最多 2,000 条 IMU、8 张图像；溢出丢最旧并计数；不承诺整体内存或任意乱序有界。
- IMU 偏移和范围在进入暂存前检查；driver fix 和最终时间范围在首帧锚点建立后检查。
- 保留 Hilti 每四帧选择、图像去重/回退、激光空帧拒绝和首帧后既有计算。
- 工作区证据目录 `validation/startup-sync-20261115/`；构建和回放串行、`nice -n 10`、`-j1`、
  `OMP_THREAD_LIMIT=1`。原始 `Retail_Street.bag` 及其他包保持不变。

## Review Focus

- 开启和关闭开关时，首帧前有效/无效输入及返回值均应符合各自契约；Task 1、2。
- 激光一直不来、空扫描、输入关闭和缓冲溢出不能造成无限增长或提前丢失暂存；Task 1。
- driver fix、IMU 范围边界和回退在暂存释放时不得让首帧回调意外抛出数据时间异常；Task 1。
- Hilti 采样、图像重复/回退和启动顺序变化不能导致不同有效样本；Task 1。
- 两端真实节点参数、统计与短 bag 场景一致；新默认行为与旧基线不能混为一谈；Task 2、3。

---

### Task 1: 有界的共享核心启动暂存

**Files:** Modify `include/livo_atlas/core/system_options.h`, `include/livo_atlas/core/livo_system.h`,
`src/core/livo_system.cc`, `tests/core/livo_system_test.cc`。

**Interfaces:** `SystemOptions::buffer_before_first_lidar` 默认 `true`。新增普通值类型
`StartupBufferStats`，提供暂存数量、溢出和释放时 IMU 拒绝计数；
`LivoSystem::GetStartupBufferStats() const` 返回只读快照。调用方仍串行拥有系统。

- [ ] 增加失败测试：同一 IMU/图像序列分别在首帧扫描前后到达，比较 LIVO 更新数、时间、
  状态及导出数组；首帧先到的测试组是对照。运行新增测试，确认旧实现失败。
- [ ] 先增加并运行边界测试：关闭选项保留旧拒绝，非法扫描不释放缓存，非有限及时间范围无
  副作用，IMU/图像容量及统计，正常扫描后释放，关闭输入释放无扫描缓存，Hilti/重复/回退。
- [ ] 增加 driver fix/时间偏移测试：同一首帧锚点下先后到达得到一致输入；暂存项校正后越界
  或回退只拒绝该项并计数，不使有效首帧失败。保持首帧后的旧规则。
- [ ] 实现上述选项、直接成员和只读统计，重用现有同步器；在每个新增或修改函数间留空行，
  注释非显然的启动时间与所有权。运行所有新增测试及普通 SDK 全套，提交核心改动。

### Task 2: 双 ROS 配置和启动统计

**Files:** Modify `src/core/livo_config.cc`, `src/ros1/livo_node.cc`, `src/ros2/livo_node.cc`,
`tests/core/global_output_options_test.cc`, `tests/ros1/node_options_test.cc`,
`tests/ros2/node_options_test.cc`；按实际需要修改相关测试与配置说明。

**Interfaces:** `LoadLivoConfig` 读取 `common.buffer_before_first_lidar`；两端 `ReportInputs()`
使用 `GetStartupBufferStats()`，保留已有输入计数文本并报告启动溢出/拒绝。

- [ ] 先写参数默认值、显式 false、两端参数名与统计用例，运行 RED 确认未接线。
- [ ] 接线共享参数及对称节点统计，运行普通 SDK、Noetic 和 Humble 对应节点/配置测试；
  验证原有输入计数解析不受影响。提交配置和接入改动。

### Task 3: 回放、文档与独立审阅

**Files:** Create `docs/startup-sensor-buffer.md`, `docs/startup-sensor-buffer-validation.md`；
modify `docs/README.md` and this plan/design status. Validation scripts and logs stay outside the package.

- [ ] 固定源码、原 bag、受控短 bag、工具及旧参考的 SHA；验证两份受控短 bag 的每话题消息
  序列完全相同。分别记录修复前 IMU 接收与初始化信息。
- [ ] 真实 Noetic/Humble 分别回放受控短 bag 的早/晚首帧；开启选项时同平台四数组、里程计
  数值、点云/图像语义和更新计数严格一致；关闭选项时保留旧差异作为兼容对照。
- [ ] 两端各完整回放原始 Retail_Street：关闭选项与同平台旧基线严格对照，开启选项验证输入、
  生命周期和输出；如两者有数值差异，记录原因及差异，不放宽或伪称旧数值不变。
- [ ] 完整 SDK 和双 ROS 节点/配置测试；按本机可用工具检查格式、构建隔离和依赖边界。
  写明跨机器复现、固定容量、溢出、driver fix 和同步保证范围；独立审阅、处理高优先级问题。
