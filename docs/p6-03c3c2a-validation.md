# P6-03c3c2a 空间联合渲染验证记录

状态：功能、双ROS矩阵、最终官方回放与故障复验通过；最终审阅及归档收尾中。

设计见 [空间候选设计](superpowers/specs/2026-11-18-p6-03c3c2-spatial-rendering-design.md)，
执行依据见 [实施计划](superpowers/plans/2026-11-18-p6-03c3c2a-spatial-rendering.md)，
配置与异机复现命令见 [在线 Gaussian](online-gaussian.md)。

本阶段关闭 C3c1 的五项可靠性遗留，新增显式开启的空间候选策略。
普通 C++ 核心从已确认的优化后范围和 retained 查询相机选择最多一个历史块；
同一 ROS main、owner 和 worker 完成接入。默认 temporal 及原资产/map schema 保持。
ROS 图像发布属于后续 C3c2b。

## 源码与环境

- 最终运行源码：`33d3408481c1f695b795f4cba1eead6ecb5f502a`；后续提交补充文档。
- 分支：`framework_completion`；原地开发，仅修改 livo-atlas，未推送或打标签。
- 新证据目录：工作空间 `validation/P6-03c3c2a-20261118`。
- Noetic 主机；Humble 使用已有 ros2 容器中的独立阶段目录。
- RTX4060 Laptop 8GiB，driver535.183.01；Python3.10.22、NumPy1.26.4、
  PyTorch2.3.0+cu121、gsplat1.5.3。构建与 GPU 回放串行，OMP1。
- Retail_Street.bag，SHA256：
  `1745bee1d570749679bad7a0392100477ab62d50aa77f7f629bc57f2a72e4427`。
- 原始源码归档、Noetic节点/动态库、Humble节点/动态库及bag都有SHA清单；回放前后核对。
- 旧阶段2695个保留文件在本阶段开始前全部通过SHA检查；环境与旧资产保留。

## 组件与平台矩阵

| 检查 | 结果 |
| --- | --- |
| 普通核心，Gaussian/GTSAM/loop ON | 72/72 CTest目标通过 |
| 固定环境 Python/CUDA | 191/191通过 |
| 回放与校验工具 | 103/103通过 |
| Noetic参数与入口 | 14项真实参数；开关入口、暂停时钟、持久化/Finalize通过 |
| Humble参数与入口 | 16项真实参数；75/75 CTest，开关入口及安装消费通过 |
| shared/static × Gaussian ON/OFF | 独立公开头、安装消费和关闭导出检查通过 |
| 旧C++17客户端 | 未改动的聚合初始化客户端重建、执行通过 |
| 动态链接 | 普通核心/客户端无Torch、CUDA、vikit依赖，OFF不导出Gaussian target |

Humble覆盖CPU、配置、协议入口与安装消费；实际CUDA验证在Noetic主机。
三个按需C++ owner集成用例显式启动真实worker运行，默认CTest中的跳过不作为GPU证据。

实际C++ owner → CUDA worker验证了非相邻0+3、第三块2驻留，以及历史修复期间读取双资产。
Python真实CUDA测试确认推理前后参数、Adam、梯度和训练前沿逐位一致。
固定种子108个投影案例，60个有实际投影/可见支持；实际gsplat中心、半径及alpha像素
均在C++保守矩形内。案例包含裁剪面、边缘、旋转、各向异性和大坐标抵消；有限样本
不能替代对所有浮点输入的证明。不确定数值保留完整画布，候选面积是保守上界。

## 最终官方回放

所有运行使用同一编译源码和库、rate1、CPU0、OMP1、image queue32。
三种主要配置均开启Gaussian持久训练和Finalize，只改变联合渲染开关与选择策略。
短段10秒，完整段请求136秒并自然读完bag；另外保留短段完全关闭Gaussian的前端参照。

两轮完整三组及短段对照均完成；下表只使用日志修正后的最终运行。
前端数值数组、原始关键帧观测及发布语义完全一致。
完整段均为1352次LIO、1351次VIO、1352行轨迹、172关键帧和517个原始观测文件。
每组最终13/13子地图current、durable，Finalize成功且worker正常停止。

Noetic接纳统计使用实际选中输入首末时间和播放器启动前的稳态时刻推导保守截止，
扣除10ms余量；检查器独立重算。播放器存活和退出耗时均不计入在线证据。

| 最终运行 | 截止前接纳 | 双块RGB贡献 | 查询组数 | 非相邻RGB贡献 | 提交/总接纳/过期 |
| --- | --- | --- | --- | --- | --- |
| short/temporal | 4 | 0 | 0 | 0 | 5/5/0 |
| short/spatial | 4 | 0 | 0 | 0 | 5/5/0 |
| full/temporal | 66 | 53 | 11 | 0 | 102/68/34 |
| full/spatial | 65 | 53 | 12 | 23 | 100/67/33 |

空间模式超过至少6次双块贡献、跨3组的门槛。Retail实际出现23次
非相邻RGB贡献；合成真实GPU用例另验证了明确的0+3选择和第三块驻留。
所有成功运行的pending/in-flight峰值为1/1，终端pending/in-flight/storage为0，失败计数0。
每个预览目录只保留最新四对PNG/JSON；原始接纳日志保留完整来源身份和选择摘要。

空间选块CPU时间平均0.0720ms、最大0.1952ms。
往返延迟平均0.189s，P95为0.237s；
对应worker时间平均0.170s，包含基准/联合推理及指标计算。

| 完整段模式 | 节点RSS峰值MiB | worker RSS峰值MiB | Torch allocated/reserved峰值MiB | 有效训练操作 | Finalize秒 |
| --- | --- | --- | --- | --- | --- |
| off | 829.36 | 1160.99 | 96.53/112 | 174 | 14.622 |
| temporal | 838.46 | 1161.48 | 96.53/114 | 172 | 14.581 |
| spatial | 831.38 | 1195.98 | 96.53/114 | 174 | 12.754 |

worker统计覆盖实际生命周期；Torch峰值为累计分配器计数。
异步训练次数随调度变化，Gaussian资产内容可以不同；这里的严格一致性结论针对前端和原始观测。

## 故障、审阅与保留

实际4GiB CUDA分配请求、最终JSON rename失败及1ms超时的最终故障复验全部通过。
OOM/写盘失败只停用联合预览，后续训练确认和durable Finalize成功，worker正常退出；
超时使Gaussian会话失败，前端继续完成98次LIO、97次VIO和98行轨迹。三种故障均与
关闭Gaussian的原始观测、前端数组及发布语义完全一致，终端渲染持有计数全部为0。
候选阶段在超时回放中发现终端stdout统计被stderr诊断插断；真实日志检查RED复现后，
在终端诊断前刷新stdout，并让故障检查器匹配确切计数前缀。原失败日志保留，未改写为成功。
修正后重跑完整平台矩阵、短段和完整三组，最终日志检查单独记录RED/GREEN。

最终独立审阅和保留体积将在本记录补齐。清理前归档执行ledger、原始审阅、测试日志、
源码、逐文件SHA和删除清单；完整空间结果作为本阶段保留数据。

C3c2b双ROS图像发布尚未实施。候选面积不保证可见性或图像质量提升；查询仍为retained训练帧。
跨块联合训练、densification、自由相机、长序列验证和完整在线3DGS SLAM属于后续工作。
本阶段元数据仅在内存中随确认内容管理，现有持久schema不支持整套会话断电恢复。
