# P6-02c3a 验证记录

状态：实现与正式主验收完成，整阶段独立审阅待执行。
冻结产品提交 `39f74941256676fa7f2625a5a4e886a9230ff984`，实施起点 `3af98a3`。
所有原始日志、快照、安装目录和回放位于包外 `validation/P6-02c3a-20261114/`，迁移电脑时单独复制。

本轮只修改 livo-atlas，沿用 framework_completion 和单入口，没有修改 vikit、引入 Pimpl 或新增安装库。
设计、使用及范围分别见 [C3a 设计](superpowers/specs/2026-11-14-p6-02c3a-global-pose-design.md)、
[实施计划](superpowers/plans/2026-11-14-p6-02c3a-global-pose.md)和[全局输出说明](global-pose-output.md)。

## 已有直接证据

- 普通原生 55 个程序、454 个用例通过；覆盖非交换组合、非法/非有限值、溢出、近似 raw 投影、
  N<M、同版本前沿增长、正式提交后的通知失败、legacy 拒绝和普通结果回调可见性。
- Noetic 普通 56 个程序、474 个用例通过；实际节点/发布器 20 个用例与独立进程时钟 3 个用例通过。
- Humble 58 个程序、506 个用例通过，包含 25 个实际 ROS 节点用例。
- Python 3.10 + 既有 rosbags 前缀的 27 项工具测试通过；宿主 Python 3.8 为 22 项通过，
  另一个迁移测试模块按其既有版本条件跳过，已由 Humble 完整补跑。
- 新 SDK、catkin、ament 安装消费者实际调用普通修正接口和空在线系统 getter；矩阵进一步验证可选依赖边界。

新增测试先观察缺失接口/缺失接线/缺失 CLI 的失败，再实施。
停止测试曾继续处理已 Finish 系统的节点定时器，已按主循环真实顺序改为先停止节点回调、保留观察端再排空；
保留原失败日志，生产停止语义没有为测试改变。

## 正式验收安排与证据边界

归档 `product-source.tar` 含 443 个普通文件；Noetic/Humble 副本逐文件核对一致。
六组 SDK 与每端四组 ROS 构建均使用同一冻结源码，重型构建串行 nice10/-j1。
统计 XML 根节点，避免将 catkin wrapper 的嵌套计数重复累计。

每端完整 136 秒三组（global/loop：off/off、off/on、on/on），
3 秒短段三组（off/off、on/off、on/on），共 12 次；使用官方 Retail_Street 数据。
CPU0、OMP_THREAD_LIMIT=1、1 倍速、图像队列 32、无 RViz；实际节点、加载库 SHA、RSS 和执行条件另存。
回放显式开启原有几何子地图，使 C2b 前端参数及资产对照保持一致；全局功能本身不隐式开启几何。

旧四数组、关键帧/成员资产、云/图像语义以及原始 odometry 数值/数量严格对照同平台 C2b。
C2b 未保存 TF wire 记录，不能宣称对旧 TF 原始消息逐字节比对；本轮检查实际 raw TF 与原始 odometry 一致，
并用 global OFF/ON 控制、两端直接测试验证父边、时间和配对。

全局校验从正式 pose 文件和 raw 锚点按 projected 规则重新计算允许修正，
检查同一 TFMessage 的两条边、整数时间、单父关系和 global_pose 的整条变换链。
同值修正可能对应多版本，报告可解释集合；ROS 消息不提供唯一版本证明。
跨话题队列不保证最后样本必达，校验明确记录可能缺失的尾部 pose，不隐藏中段丢失。
校验器另用人工非零变换和错误位姿、第二父边、错误时间三个损坏用例检查。

负 `/clock` 探针在 Humble 上异常中止；移除 LivoPublisher 后仍复现同一时间源异常。
该外部依赖证据独立保存，正式时钟用例采用 0/10/10/9/11，覆盖零、暂停、回退与恢复。

## 正式构建矩阵

| SDK 组合 | 程序 | 用例 | 安装消费者 |
| --- | ---: | ---: | ---: |
| both-on | 55 | 454 | 1 |
| both-off | 40 | 338 | 1 |
| graph-only | 42 | 351 | 1 |
| loop-only | 45 | 371 | 1 |
| static-both | 55 | 454 | 1 |
| no-openmp | 55 | 454 | 1 |

| ROS / 组合 | 普通程序 | 用例 | 实际 ROS 用例 | 安装消费者 |
| --- | ---: | ---: | ---: | ---: |
| noetic / off | 41 | 358 | 23（另计） | 1 |
| noetic / static-off | 41 | 358 | 23（另计） | 1 |
| noetic / static | 56 | 474 | 23（另计） | 1 |
| noetic / on | 56 | 474 | 23（另计） | 1 |
| humble / off | 43 | 390 | 25（已计入） | 1 |
| humble / static-off | 43 | 390 | 25（已计入） | 1 |
| humble / static | 58 | 506 | 25（已计入） | 1 |
| humble / on | 58 | 506 | 25（已计入） | 1 |

14 组构建、14 个安装消费者全部通过。双开构建检查 63 个普通公开头。
关闭可选模块时对应目标和测试缺席，GTSAM 关闭时实际消费者/节点不加载 libgtsam；
静态组合不加载 livo-atlas 共享库。Humble 每组另有 27 个 Python 工具测试通过。

## 实际主入口启动

每端六组真实 main 检查均通过：启用全局输出、关闭后端时单独启用、publish_tf=false，
以及非法帧名、缺少存储、未编译回环后端却请求回环三个失败路径。
成功路径实际发现 global_pose publisher 并正常停止；失败路径均返回 1，保留原有 sentinel，
没有创建其他输出资产。Humble discovery 核对 reliable/volatile；DDS 未报告的 history depth 不作运行时推断，
depth=1 由发布器的显式 QoS 构造确定。日志及参数在 startup-noetic/、startup-humble/。

## 官方 bag 严格回归

| 平台 | 秒 | global / loop | 原始位姿 | map TF | global_pose | 正式磁盘版本 | RSS MiB |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: |
| noetic | 136 | off / off | 1351 | 0 | 0 | 0 | 787.2 |
| noetic | 136 | off / on | 1351 | 0 | 0 | 2 | 811.9 |
| noetic | 136 | on / on | 1351 | 1351 | 1351 | 2 | 812.9 |
| noetic | 3 | off / off | 26 | 0 | 0 | 0 | 193.5 |
| noetic | 3 | on / off | 26 | 26 | 26 | 0 | 197.7 |
| noetic | 3 | on / on | 26 | 26 | 26 | 0 | 197.7 |
| humble | 136 | off / off | 1351 | 0 | 0 | 0 | 829.6 |
| humble | 136 | off / on | 1351 | 0 | 0 | 2 | 851.0 |
| humble | 136 | on / on | 1351 | 855 | 855 | 2 | 859.1 |
| humble | 3 | off / off | 26 | 0 | 0 | 0 | 231.1 |
| humble | 3 | on / off | 26 | 11 | 11 | 0 | 226.6 |
| humble | 3 | on / on | 26 | 15 | 15 | 0 | 223.2 |

12 次全部通过同平台 C2b 严格对照：四数组逐元素一致、旧 odometry 数值与数量一致，
原始关键帧文件/成员索引/云图像语义一致，六次短段原有导出产品逐字节一致。
所有实际节点均为 CPU0、OMP_THREAD_LIMIT=1、nice>=10、图像队列32和1倍速；
运行时观察到的 19 个 livo-atlas 库、GTSAM 库和可执行文件 SHA 分别归档，未用构建目录名称代替实际加载证明。

Humble 的原始发布中存在重复 ROS 时间：完整 on/on 的 1351 次原始位姿只有 855 个严格递增时刻，
因此发布 855 对 map TF/global_pose；原始 TF 仍为 1351 次。两个 global ON 短段同理由 26 次原始发布
分别得到 11 和 15 个合法全局时刻。全部回放的合法全局样本均被观察到，尾部缺失为 0。
这些数量差异属于设计的时钟门控，不是丢失原始输出。

全局 TF/pose 数学与正式锚点校验通过；原始 TF 始终只有 camera_init 父边。
允许修正的版本集合及变换链最大误差见各 global-checks.json，冻结源码、完整计数和 RSS 见 final-evidence.json。
bag 不带 ground truth，本轮结果证明兼容性、坐标和交付契约，不构成定位精度或长距离闭环能力评价。

合成的真实 System/Node 场景强制产生非零 v1/v2 修正，验证发布后接纳与下一样本可见性；
bag 中同值修正可以解释为多版本，不将 ROS 消息冒称精确版本传输。

## 整阶段审阅

主验收已完成，整阶段独立审阅待执行；审阅结果与裁定在收尾提交补齐。
