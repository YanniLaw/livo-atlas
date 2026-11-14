# C3b-01 验证记录

状态：**实现、14组构建矩阵、22个安装消费者、10组官方数据回放及独立审阅问题闭环完成**。
初版产品代码 `e7043fc`，容量边界和安装消费测试冻结为 `54ec060`。
独立审阅修复停止竞态后，正式复验源码冻结为
`2bcb35616fe07c93d25ef8a187bf0465554e6ece`。
随后安装验收发现并修复 Humble 静态消息类型支持，最终源码为
`85132ac47cb9f330232420b511b978f9328165f2`；其 C++ 与 `2bcb356` 相同，仅 ROS2 构建分支改变。
证据保存在包外 `validation/P6-02c3b1-20261115/`；迁移电脑需单独复制。

范围及用法见[版本化轨迹说明](keyframe-trajectory.md)、
[设计](superpowers/specs/2026-11-15-p6-02c3b-history-design.md)和
[计划](superpowers/plans/2026-11-15-p6-02c3b1-keyframe-trajectory.md)。
保留 `framework_completion` 和共同 main；没有修改 vikit、合并、推送或打标签。

## 已取得的直接证据

普通 SDK 双开全套 57 个程序、472 个用例通过。新增测试覆盖逐帧优化、原始旋转投影、同采集时间换版本、
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

`source-final-validated.tar` 和 manifest 固定最终源码，原 `source-final.tar`、`source-closure.tar`
保留作为先前证据。SDK 六组和 Noetic 四组在 `2bcb356` 完成；
隔离的 `cmake/ros2.cmake` 修复后完整重跑 Humble 四组，未重复执行不受影响的十组。
各组保存独立测试 XML、安装消费者、实际 ldd 和日志；生成消息消费者覆盖八组 ROS。
静态 Humble 的限制是算法库静态，rosidl 生成的消息运行时共享模块仍保留。
统计 XML 时避免 catkin wrapper 重复累计。

每端完整回放三组：轨迹关/回环关、轨迹开/回环关、轨迹开/回环开；
另用短段检查 geometry 关闭、预算 max_keyframes=1 和旧导出产品。
统一 CPU0、nice10、OMP_THREAD_LIMIT=1、1 倍速、图像队列32、无 RViz。
完整控制组显式开启既有 geometry，以便与 C3a 冻结基线比较；轨迹自身不隐式开启它。

`comparison-inputs-frozen.json` 记录校验脚本与 3219 个参考文件的 SHA，
`comparison-after-review-frozen.json`、`comparison-final-frozen.json` 分别更新两次修复后的源码归档身份；
校验器和参考文件保持原 SHA。首个启动器误用保护检查后，只修正其工具调用位置，
再以 `comparison-harness-fixed-frozen.json` 保存最终启动器 SHA；正式回放均在此后运行。
新回放记录实际节点及算法/生成消息运行库 SHA，避免把别的安装前缀作为本轮产品。
旧四数组、原始 odometry、云/图像语义和关键帧资产按同平台精确比较。
新 typed 历史按其明确的 v/M/N，逐项核对原始时间与正式版本位姿；Path 单独解释。
两话题没有原子交付承诺；停止后的完整最终结果以落盘数据为恢复依据。

## 正式构建矩阵

| 组合 | 程序 | 用例 | 安装消费者 |
| --- | ---: | ---: | ---: |
| sdk/both-on | 57 | 472 | 1 |
| sdk/both-off | 42 | 356 | 1 |
| sdk/graph-only | 44 | 369 | 1 |
| sdk/loop-only | 47 | 389 | 1 |
| sdk/static-both | 57 | 472 | 1 |
| sdk/no-openmp | 57 | 472 | 1 |
| noetic/off | 43 | 376 | 2 |
| noetic/static-off | 43 | 376 | 2 |
| noetic/static | 58 | 492 | 2 |
| noetic/on | 58 | 492 | 2 |
| humble/off | 45 | 413 | 2 |
| humble/static-off | 45 | 413 | 2 |
| humble/static | 60 | 529 | 2 |
| humble/on | 60 | 529 | 2 |

Noetic 每组另计实际 ROS 用例28；Humble 的30项实际节点测试已包含在表内。
14组、22个安装消费者全部通过。双开普通 SDK 检查66个独立公开头。
Humble 每组29项 Python 工具测试通过；宿主25项中1个既有版本条件跳过。

## 官方数据与性能

| 平台/轨迹/回环/geometry/预算/秒 | LIO/VIO | typed/Path | 最后 wire v/M/N | RSS MiB |
| --- | ---: | ---: | --- | ---: |
| noetic-off-off-on-20000-136 | 1351/1350 | 0/0 | — | 792.4 |
| noetic-on-off-on-20000-136 | 1351/1350 | 125/125 | 0/172/0 | 788.8 |
| noetic-on-on-on-20000-136 | 1351/1350 | 126/126 | 2/172/169 | 813.1 |
| noetic-on-off-off-20000-3 | 26/25 | 3/3 | 0/2/0 | 198.1 |
| noetic-on-off-on-1-3 | 26/25 | 3/3 | 0/2/0 | 202.9 |
| humble-off-off-on-20000-136 | 1351/1350 | 0/0 | — | 826.6 |
| humble-on-off-on-20000-136 | 1351/1350 | 124/124 | 0/172/0 | 824.9 |
| humble-on-on-on-20000-136 | 1351/1350 | 123/123 | 2/172/168 | 852.4 |
| humble-on-off-off-20000-3 | 26/25 | 3/3 | 0/2/0 | 213.3 |
| humble-on-off-on-1-3 | 26/25 | 3/3 | 0/2/0 | 219.5 |

| 开启组 | max Observe µs | max Poll µs | max worker ms | 最后交付延迟 ms |
| --- | ---: | ---: | ---: | ---: |
| noetic-history-on-loops-off-geometry-on-cap-20000-136 | 1.96 | 18.60 | 0.178 | 1.565 |
| noetic-history-on-loops-on-geometry-on-cap-20000-136 | 1.57 | 3.64 | 0.147 | 1.516 |
| noetic-history-on-loops-off-geometry-off-cap-20000-3 | 1.05 | 3.80 | 0.120 | 4.044 |
| noetic-history-on-loops-off-geometry-on-cap-1-3 | 0.87 | 3.74 | 0.111 | 4.466 |
| humble-history-on-loops-off-geometry-on-cap-20000-136 | 6.23 | 19.16 | 0.241 | 3.291 |
| humble-history-on-loops-on-geometry-on-cap-20000-136 | 1.56 | 20.41 | 4.537 | 0.805 |
| humble-history-on-loops-off-geometry-off-cap-20000-3 | 1.02 | 5.32 | 0.148 | 0.676 |
| humble-history-on-loops-off-geometry-on-cap-1-3 | 1.42 | 5.45 | 0.136 | 1.774 |

`delivered` 是消费者返回，包含可能在 context 关闭后跳过 ROS 发送的最终 SDK 消费。
以上是单次运行观察值；worker 包括时间读取、构建和消费，不是硬实时上限。

| geometry 开启组 | Notify 快照残差 median/p95/max µs |
| --- | ---: |
| noetic-history-off-loops-off-geometry-on-cap-20000-136 | 7.33/12.38/16.83 |
| noetic-history-on-loops-off-geometry-on-cap-20000-136 | 7.45/12.99/15.27 |
| noetic-history-on-loops-on-geometry-on-cap-20000-136 | 7.26/25.06/28.80 |
| noetic-history-on-loops-off-geometry-on-cap-1-3 | 2.87/3.98/4.10 |
| humble-history-off-loops-off-geometry-on-cap-20000-136 | 5.78/8.44/10.03 |
| humble-history-on-loops-off-geometry-on-cap-20000-136 | 5.41/8.46/10.12 |
| humble-history-on-loops-on-geometry-on-cap-20000-136 | 5.67/25.23/29.69 |
| humble-history-on-loops-off-geometry-on-cap-1-3 | 2.80/4.24/4.40 |

残差逐次计算 `geometry_total_seconds - geometry_update_seconds`，包含原有完整位姿物化
以及调用边界开销，是该快照路径成本的估计上界，**不包含随后执行的 observer callback**。
geometry 关闭时没有这组既有计时，另用下方公开函数合成测试说明规模成本。

| 合成帧数/v | 位姿物化 median/max ms | 轨迹构建 median/max ms | pose/trajectory 字节 |
| --- | ---: | ---: | ---: |
| 172/0 | 0.007711/0.012763 | 0.013435/0.017147 | 17888/19264 |
| 172/1 | 0.015401/0.015877 | 0.004394/0.004482 | 17888/19264 |
| 20000/0 | 1.11879/1.26401 | 1.61706/1.62058 | 2080000/2240000 |
| 20000/1 | 1.88495/1.90258 | 0.530297/0.586117 | 2080000/2240000 |
| 100000/0 | 6.35976/6.90426 | 9.41212/9.60184 | 10400000/11200000 |
| 100000/1 | 10.8727/11.1541 | 3.29608/3.30621 | 10400000/11200000 |

合成测试在CPU0/nice10/OMP单线程下每组合测3次，调用安装 SDK 的公开函数；
字节数是 `sizeof` 数组载荷，不含容器、消息和分配器开销。
它测量规模成本，不能证明真实长距离或整个系统的时延/内存有界。


两端回环组均实际观察到版本0、1、2，最终覆盖172帧。Noetic/Humble 的正式优化前沿分别为169/168，
按各自持久文件核验，不宣称跨平台优化数组相同。所有四个短段各57个旧产品文件逐字节一致。
两次 cap=1 回放均观察到一次 ResourceLimited，最后 M=2、数组为空；没有静默截断。

## 失败与证据边界

独立整阶段审阅使用 fresh `gpt-6-astra/xhigh`，指出一个 Important、没有 Critical 或 Minor。
`Stop()` 原先不持等待锁便改变停止条件并通知，可能在 worker 检查条件与进入等待之间丢失唤醒。
新增 2000 次 idle 停止/析构回归，修复前 15 秒超时；在同一 mutex 下更新条件后 38 ms 通过。
双关全套 42 程序转绿，随后 SDK 六组、Noetic 四组及 Humble 四组全部通过。
原矩阵双关也曾停在 restart 用例，终止单个卡住测试后保留失败记录；
`/proc` futex 现场与该问题一致，但没有线程调用栈，不能据此声称该次挂起已被独立因果定位。
没有延期 Minor；几何显示、并发 owner 调用、外部改写已提交文件、强制中断阻塞消费者和全进程资源承诺
仍遵循已记录的阶段与所有权边界。

Humble 静态安装验收另发现一项 Important：Fast RTPS 生成器继承 `BUILD_SHARED_LIBS=OFF`，
只安装 `.a`，而 generic type support 按名称加载 `.so`。原构建目录残留共享库使内部 CTest 通过，
激活正确安装前缀的独立消息消费者仍失败，因此不能把构建测试等同于安装可用。
只在消息生成调用周围启用共享库，再恢复算法选项；同一消费者与 45 程序全套均转绿。
正式静态复验前保留并移开旧安装目录，从空前缀重新安装，防止旧产物掩盖缺陷。

保留过往 RED 及环境排查日志。Noetic shutdown 的前两次尝试属于测试环境准备失败，
正确失败为 `task4-noetic-shutdown-red3.log`；SDK 首次直接 g++ 探针缺 Eigen，
正确接口 RED 为 `installed-trajectory-sdk-red-build.log`。
Humble 普通 CTest 没有激活构建前缀，曾因 RMW 类型支持 dlopen 路径失败；
只添加构建库路径，同一二进制的新用例全部通过，安装消费者另用实际安装激活验证。

现有 bag 约135秒，合成规模测试和重复回放都不能替代真实长距离数据。
该 bag 没有本阶段可用的真值，验收检查行为一致性和版本协议，不代表绝对定位或闭环精度评测。
该组件的预算不能证明整个 SLAM 进程内存有界，也不保证任意消费者的延迟。
本轮修复在同一次验收闭环中完成，没有进行第二轮审阅；最终结论以修复后的真实验证为依据。

## 复现与迁移

源码、配置、普通构建和双ROS入口均在仓库内；参见[核心构建](core-build.md)、
[双版本构建](ros-builds.md)及[轨迹使用说明](keyframe-trajectory.md)。
本轮验收使用独立构建/安装目录，不会自动更新日常工作区原有的 devel。

包外证据需要另行复制，至少保留 `source-final-validated.tar`、对应 manifest、
`comparison-harness-fixed-frozen.json`、`validation-summary.json`、原始测试 XML/日志、
回放目录及校验脚本。完整参考对照还需要 `P6-02c3a-20261114` 中冻结的参考数据。
本机编排脚本包含 root、Sophus、GTSAM、Livox、Docker 等本机路径；
换机器重跑前调整这些路径，在新的证据目录运行，保留旧记录。
日常使用无需运行整套本机编排脚本，可直接按双ROS构建说明和轨迹文档回放新输出目录。

本轮验收入口：

```bash
# 在已完成本轮编排、证据仍位于原目录的机器上复核：
OMP_THREAD_LIMIT=1 python3 \
  /home/yanni/livo-atlas_ws/validation/P6-02c3b1-20261115/verify_task5.py
```

该命令重新运行普通57程序和14组验收器测试，再以只读方式复核各平台 XML、
全部历史记录及格式检查时的源文件 SHA，不覆盖原始回放。
`check_trajectories.py NEW_OUTPUT [max_keyframes]` 可用于新记录的独立检查；
它会新建 trajectory-checks.json，已有文件时拒绝覆盖。
正式位姿和原始观测仍是恢复权威，最终结果不依赖最后一条 ROS 消息。

## 实施裁定与代价

- 两端转换共用私有模板，保持协议一致；代价是模板报错比两个直接实现间接。
- CTest 激活生成库搜索路径，安装消费另行检查；构建残留可能掩盖安装缺陷，故不能省略后者。
- context 关闭后跳过 ROS 转换/发送，保留 SDK 消费；此时才遇到的 ROS 转换问题不会作为消费者失败报告，也不保证最终 wire 交付。
- 静态构建约束算法库，生成消息运行时保持共享；不能当作完全静态的 ROS2 分发。
- 冻结代码审阅与耗时验收并行；审阅当时证据尚未齐全，最终验收由实际测试闭环。
- C3b-02 几何显示与训练另行实施；需要额外阶段才能提供这些能力。
- owner 方法单线程且不重入，已提交文件不可改写；违反前者会竞态，违反后者可能使用过期缓存。
- 阻塞 I/O/消费者没有强制取消时限；外部阻塞可能延长 join。
- 本组件预算不覆盖全进程、无限期录制或长距离可靠性；这些目标仍需设计与真实数据。
- ROS2 构建分支修复后只复验受影响的四组 Humble；没有第二次运行不受影响的十组，以分支隔离和源文件相同作为复用依据。
