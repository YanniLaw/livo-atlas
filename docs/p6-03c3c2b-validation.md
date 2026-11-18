# P6-03c3c2b：有版本双 ROS 预览验证

当前运行源码：`01203e2`，基于 `69c0e0e`，分支 `framework_completion`。
实现、使用方法见[预览说明](p6-03c3c2b-ros-preview.md)，边界见
[设计稿](superpowers/specs/2026-11-19-p6-03c3c2b-ros-preview-design.md)与
[实施计划](superpowers/plans/2026-11-19-p6-03c3c2b-ros-preview.md)。
本记录仍在收尾：构建矩阵、最终审阅与证据整理完成后补充最终结论。

## 已完成的行为检查

- 原基线72 CTest通过；当前73 CTest、196 Python/CUDA、118工具测试通过。
- 发布控制器11个用例：阻塞回调下100次替换、单 pending、版本缺口、同采集时间不同 job、
  首次/恢复READY不被状态流量推迟、清图错误分类、不可用 context、Finish join与释放。
- Noetic44个原生用例，Humble47个原生用例通过，含RGB/K/时间/逆相机位姿/来源、
  uint64大ID、晚加入READY与失效缓存、非法DTO清图、显式活context停止和关闭context。
- 两端同一main的暂停仿真时钟入口，发布OFF/ON通过；旧C++17聚合消费端通过。
- 接收校验覆盖旧revision、上下文倒退、旧session/异producer、像素/相机/来源篡改、
  截止后收到的旧采集时间图像，以及关闭context时部分native发布。

## 官方 Retail_Street.bag 成对回放

原bag：135.47秒，SHA256：
`1745bee1d570749679bad7a0392100477ab62d50aa77f7f629bc57f2a72e4427`。
两侧都打开Gaussian、持久化、空间联合渲染、Finalize；只切换ROS发布。
OMP/BLAS线程为1，节点CPU affinity为0，回放1倍，图像发布队列32。

| 项目 | 10秒短段 | 完整bag |
| --- | ---: | ---: |
| 严格成对比较 | 通过 | 通过 |
| 原始文件数（逐字节一致） | 16 | 517 |
| 实际收到组合消息 | 17 | 478 |
| 输入截止前实际READY | 4 | 67 |
| 截止前与保留PNG完成相机/RGB核对 | 3 | 2 |
| 标准镜像与组合图像匹配 | 5 | 69 |
| pending / in-flight峰值 | 1 / 1 | 1 / 1 |
| 不可用context尝试 | 1 | 1 |

前端 `trajectory.tum/mat_pre.txt/mat_out.txt/imu.txt` 数组、原始文件和原有话题语义
严格相等。完整两侧均172帧、13个回环查询、2次接受、pose_version2；最终13块current且durable。
原始文件数不是关键帧数量。

SIGINT后ROS context已关闭，实际未收到STOPPED，报告明确记为false；没有把native完成计数
当作远端送达。已开始的native调用可完成一个旧as-of版本，串行revision和接收端拒旧约束
保持顺序；标准Image跨话题不具版本权威，也不承诺RViz自动清屏。
保留PNG为有限历史，因此仅列明确匹配数量，其余READY验证其原owner接纳身份。
Humble验证为CPU/真实DDS/入口，不声称做过Humble CUDA bag回放。

## 证据位置与复现

本机独立阶段：`validation/P6-03c3c2b-20261119`。
`paired-results.json`、`cand-{short,full}-comparison.json`保存实际比较；
`product-source.tar.gz`及SHA、`native-libraries.sha256`、`bag.sha256`固定代码/二进制/输入。
`run-publication-replay.sh short|full off|on`复现实验；运行前使用新阶段路径，禁止覆盖旧结果。
`run-all.sh`执行普通C++、旧SDK与Python工具/GPU套件；全部编译或GPU任务按workload.lock串行。

最终矩阵、审阅、归档与保留大小将在收尾时更新。
