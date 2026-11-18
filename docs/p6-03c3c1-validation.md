# P6-03c3c1 在线联合渲染验证记录

状态：候选平台矩阵、配对回放和故障验收已通过；一次独立整范围审阅与最终重跑尚未关闭。

设计与执行依据见 [设计](superpowers/specs/2026-11-18-p6-03c3c1-online-joint-rendering-design.md)
和 [计划](superpowers/plans/2026-11-18-p6-03c3c1-online-joint-rendering.md)。实现与操作见
[在线 Gaussian 使用说明](online-gaussian.md#在线联合渲染c3c1)。

## 候选源码与环境

- 实现候选：`929ac9182783257e8c9acc1e73f86fec746f893c`，当前分支 `framework_completion`。
- 新证据目录：工作空间外包目录 `validation/P6-03c3c1-20261118`；旧阶段资产、环境和 bag 保留。
- 主机：ROS1 Noetic；Humble 使用已有 `ros2` 容器中的新验证目录。
- GPU 环境沿用固定 Python3.10.22 / NumPy1.26.4 / PyTorch2.3.0+cu121 / gsplat1.5.3 预编译扩展。
- 输入：`Retail_Street.bag`，SHA256
  `1745bee1d570749679bad7a0392100477ab62d50aa77f7f629bc57f2a72e4427`，约135秒。

## 已执行的组件验证

71 项普通核心 CTest、171 项 Python/CUDA 和84项工具测试均通过。
公开只读选择器、完整 primary 传输、超过2^53的ID、显式能力协商、指标有效性与文件空值均有覆盖。
实际 GPU 验证包含第三块驻留而查询来自非驻留资产、旧 capture/当前投影查询、
参数/Adam/梯度和确认状态不变、真实40% allocator限制下OOM后继续20步训练。

候选丢弃后的会话 fence 与旧绑定代数分别核对；独立 CPU/Python/C++ 与真实 actor 回归已通过。
在线 allocator 统计保留累计历史；worker 时间与 RSS 在两次指标计算后采样。

真实 C++ owner 到 CUDA worker 的历史修复/双资产选择，以及最终 JSON rename 失败后继续训练和持久退出均通过。
文件测试核对不同 RGB 通道、JSON最后发布、现有路径保护、四对保留上限和保存失败停用。
验收器使用最后一次确认回放仍在运行的时间作为保守边界，结束后文件不满足在线接纳要求。

Noetic13项、Humble15项真实参数测试及两个共用入口的暂停仿真时钟/能力协商/持久退出检查通过。
Humble直接从构建目录启动需构建DDS类型支持插件并加入其库搜索路径；验证脚本已修正。
这里的 Humble 证据覆盖 CPU/配置/协议入口，CUDA实测位于Noetic主机。

## 候选平台与真实回放

shared/static 各 Gaussian ON/OFF 均完成安装消费和公开头检查；关闭导出不包含 Gaussian target。
Noetic 和 Humble 均完成 ON/OFF、真实参数与共用入口检查；Humble 全部74项CTest通过。
主机CMake3.16不支持 `ctest --test-dir`，第一次错误工作目录的“无测试”结果未计入；
改用 `cmake -E chdir` 后实际71项CTest全部通过，正式脚本同步修正。

Noetic 以 rate1、CPU0、OMP1、image queue32，完成10秒短段和约135秒完整段配对回放。
两侧均开启 Gaussian 持久训练与 Finalize，仅联合渲染开关不同。
短段接纳5次在线单块渲染；完整段在保守输入结束边界前接纳69次渲染，
其中56次双块对量化RGB产生实际贡献，跨12个查询组，超过6次/3组门槛。
完整ON共提交102次，接纳70次、过期32次、失败0次；pending和in-flight峰值均为1。
四对历史文件上限与保存身份核对通过。

OFF/ON严格比较的前端数值、原始关键帧观测和发布语义完全一致：
完整段1352次LIO、1351次VIO、1352行轨迹、172个关键帧。
两侧退出前均达到V2、172帧/169优化帧，13块全部current且durable；Finalize均成功。
训练异步计数允许不同，不能把这些计数差异解释为前端误差。
原C1短段关闭/开启Gaussian兼容对照亦通过。

候选完整OFF/ON worker累计allocator分配峰值同为96.53MiB，保留峰值分别112/114MiB；
worker实际生命周期RSS峰值约1144/1143MiB，节点采样RSS峰值约826/841MiB。
Finalize分别约14.05/15.05秒；这些是候选单次测量，不代表长序列资源保证。

真实短bag故障控制仅拦截首次联合推理：4GiB实际CUDA分配触发40% allocator预算下OOM，
联合预览failed/disabled各1，后续确认训练继续且Finalize成功；1ms超时使Gaussian会话失败，
前端完整运行。两例均与普通前端的数值、原始观测和发布语义完全一致。
控制仅用于验收，不引入产品故障开关。

下一步执行一次独立整范围审阅；重要修正须有失败复现与修复验证，
再固定最终源码重跑。最终记录补齐二进制/源码/日志/产物校验、资源统计、
清理清单和执行记录归档。

本阶段交付版本一致的在线文件预览；空间可见性选择、双ROS图像显示、跨块联合训练和完整在线3DGS SLAM属于后续阶段。
