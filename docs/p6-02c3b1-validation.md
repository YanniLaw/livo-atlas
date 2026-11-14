# C3b-01 验证记录

状态：实现已冻结，正式构建矩阵、完整回放和整阶段审阅进行中。
产品代码 `e7043fc`，补充容量边界和安装消费测试后冻结为
`54ec060148b8310f770f883ccf74dcc287bc0c6f`。
证据保存在包外 `validation/P6-02c3b1-20261115/`；迁移电脑需单独复制。

范围及用法见[版本化轨迹说明](keyframe-trajectory.md)、
[设计](superpowers/specs/2026-11-15-p6-02c3b-history-design.md)和
[计划](superpowers/plans/2026-11-15-p6-02c3b1-keyframe-trajectory.md)。
保留 `framework_completion` 和共同 main；没有修改 vikit、合并、推送或打标签。

## 已取得的直接证据

普通 SDK 全套 57 个测试程序通过。新增测试覆盖逐帧优化、原始旋转投影、同采集时间换版本、
受控阻塞下交付和合并、时间缓存、错误隔离、Finish 与析构生命周期。
M=20001 超过默认预算时立即释放新增快照的持有、不读取时间，另有小预算真实存储测试。
安装消费者实际调用构建器、连接空在线系统、排空新组件；不只检查头文件能否包含。

两端各完成 3 秒真实入口试跑，记录 revision 1…3 对应 M=0、1、2，
typed 消息和 Path 均收到三次。独立脚本从落盘 frame.yml 重算后通过。
Noetic/Humble 实际 ROS 测试覆盖晚加入、清空、异常时间、20000 行真实序列化和关闭 context。
序列化大小分别为 1440108 和 1440116 字节，格式差异来自 ROS1 wire/CDR。
Humble 的安装消息消费者另外实际完成序列化和反序列化。

验收器另有 14 组人工输入，覆盖逐帧不同优化量、projected tail、混用版本、
错误时间、ID、N、版本回退、重复 revision、资源不足和 Path 跨话题独立丢样。
Path 可以有多个合理的版本解释，报告集合和非递减解释，不伪造消息里没有的版本号。

## 正式验收安排

`source-final.tar` 和 manifest 固定上述源码；编译按 SDK 六组及 Noetic/Humble 各四组串行执行。
各组保存独立测试 XML、安装消费者、实际 ldd 和日志；生成消息消费者覆盖八组 ROS。
静态 Humble 的限制是算法库静态，rosidl 生成的消息运行时共享模块仍保留。
统计 XML 时避免 catkin wrapper 重复累计。

每端完整回放三组：轨迹关/回环关、轨迹开/回环关、轨迹开/回环开；
另用短段检查 geometry 关闭、预算 max_keyframes=1 和旧导出产品。
统一 CPU0、nice10、OMP_THREAD_LIMIT=1、1 倍速、图像队列32、无 RViz。
完整控制组显式开启既有 geometry，以便与 C3a 冻结基线比较；轨迹自身不隐式开启它。

`comparison-inputs-frozen.json` 在正式回放前记录校验脚本与 3219 个参考文件的 SHA。
新回放记录实际节点及算法/生成消息运行库 SHA，避免把别的安装前缀作为本轮产品。
旧四数组、原始 odometry、云/图像语义和关键帧资产按同平台精确比较。
新 typed 历史按其明确的 v/M/N，逐项核对原始时间与正式版本位姿；Path 单独解释。
两话题没有原子交付承诺；停止后的完整最终结果以落盘数据为恢复依据。

## 失败与证据边界

保留过往 RED 及环境排查日志。Noetic shutdown 的前两次尝试属于测试环境准备失败，
正确失败为 `task4-noetic-shutdown-red3.log`；SDK 首次直接 g++ 探针缺 Eigen，
正确接口 RED 为 `installed-trajectory-sdk-red-build.log`。
Humble 普通 CTest 没有激活构建前缀，曾因 RMW 类型支持 dlopen 路径失败；
只添加构建库路径，同一二进制的新用例全部通过，安装消费者另用实际安装激活验证。

现有 bag 约135秒，合成规模测试和重复回放都不能替代真实长距离数据。
该组件的预算不能证明整个 SLAM 进程内存有界，也不保证任意消费者的延迟。
性能表和最终审阅结论将在本轮正式验收完成后补齐。
