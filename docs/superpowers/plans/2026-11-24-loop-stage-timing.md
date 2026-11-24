# 回环分段计时实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans.

目标是沿用 diagnostics.stage_timing，通过 GLOG 观察回环几何、候选、试算和交付耗时。
用户已授权在当前分支直接实施并本地提交。只修改 livo-atlas，不改变判定、调度或存档格式。

- [x] 测试先验证开关关闭不打印，开启打印准备、候选、批次、接纳和总查询阶段。
- [x] 内部查询接口增加默认关闭的计时参数；使用已有 GeometryMapper 统计区分点云读取、完整构建和快照。完整构建包含读取与快照，指标不能直接相加。
- [x] 沿 Task 依赖记录批次墙钟时间；结果计算完成到队列执行记录排队，Publish 到 Acknowledge 记录发布等待。计算与发布不增加锁或改变 Task 依赖。
- [x] 运行 SDK、Noetic、Humble 相关测试，用完整 Retail_Street.bag 核对轨迹、资产与去除时间后的诊断。
- [x] 独立复核、记录各阶段占比与范围，本地提交。

审核重点为计时关闭时不读额外时钟、并行阶段的写入依赖、独立 LoopBackend 的排队语义，以及异常路径与正常统计的区别。异常中止的阶段不承诺完整耗时；总查询是注册至完成回调前，批次包含候选任务等待，发布等待涵盖 owner 交付和回执。保持 Google C++ 风格与函数间空行。

结果和指标包含关系见 [分段计时](../../loop-stage-timing.md)。已完成完整 bag 数值与归档对照，具体性能建议以此记录为准。
