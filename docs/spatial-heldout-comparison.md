# 完整 bag 空间留出画质对照

本轮验证 Noetic 原生节点、实际 CUDA worker、完整回环/位姿优化、在线留出评分与最终空间评分。
`Retail_Street.bag` 为完整135.47秒输入，`uniform` / `coverage` 各运行两次。仅改变采样策略，
保留相同线程/亲和/GPU预算、冻结相机与留出标签；异步合并导致实际训练调用次数仍会不同。

## 完成情况

- 四次节点、worker、评分器均正常退出；每次在线20成功、1正常取消、0失败，最终21/21成功。
- 四次均1352次LIO、1351次VIO；每次516个帧记录/PNG/PCD与同环境参考逐字节一致。
- 四次最终物化的172帧位姿矩阵最大绝对差为0；所有训练/损失ID均为train。
- 全部在线/最终报告与产物SHA核对通过，每个浮点样本的固定指标均精确重算。
- 补齐高度单独不匹配及真实双块报告单测；SDK82项、Python/CUDA317项及回放CLI42项全部通过；双ROS节点及Humble原生选择用例通过。

## 同源画质结果

下表使用四次共同的19个联合渲染查询，先在每次运行内按像素/窗口统计量聚合。
PSNR不是逐查询dB的简单平均；跨重复的均值只作两次结果描述。

| 运行 | 联合PSNR(dB) | 联合SSIM | 图像覆盖 | 共同深度L1(m) | 共同深度RMSE(m) |
| --- | ---: | ---: | ---: | ---: | ---: |
| uniform-1 | 10.9416 | 0.2750 | 87.53% | 0.2414 | 0.9719 |
| uniform-2 | 11.0319 | 0.2782 | 88.19% | 0.2311 | 0.9254 |
| coverage-1 | 11.8403 | 0.2843 | 92.06% | 0.2475 | 0.9362 |
| coverage-2 | 11.8685 | 0.2855 | 92.14% | 0.2368 | 0.8798 |

共同深度使用四次均满足alpha≥.05的224170个参考像素，
占这些查询LiDAR有效标签的87.92%。这样避免将各自不同的覆盖集合混作同一误差口径。

两次均值：`coverage`联合PSNR约11.8544 dB，`uniform`约10.9867 dB；SSIM为0.2849/0.2766，
图像覆盖为92.10%/87.86%。共同深度L1没有同步改善，RMSE则较低，不能据RGB收益声称几何全面提高。
在线成绩另按20个共同在线查询保存，只能连同各自as_of阅读，不与最终模型混合汇总。

## 开销与解释边界

| 运行 | 实际训练调用 | 训练P95(ms) | 前端P95(ms) | 节点RSS峰值(MiB) |
| --- | ---: | ---: | ---: | ---: |
| uniform-1 | 269 | 177.60 | 45.68 | 860.15 |
| uniform-2 | 269 | 174.33 | 45.55 | 880.19 |
| coverage-1 | 281 | 177.53 | 45.62 | 872.44 |
| coverage-2 | 273 | 176.01 | 45.88 | 876.89 |

GPU分配器累计allocated峰值约96.5MiB、reserved114MiB；这不是显卡总占用或单查询增量。
计时是本次系统运行的描述，uniform-1期间还有一次短时辅助工具构建，不作性能因果结论。
实时调度、pending合并和回环重建会改变实际训练观测/步数；因此本轮是端到端运行方式的比较，
不能将全部差异归因为固定训练序列上的采样算法收益。单一街景、每种两次，也不能推广到其他场景。
默认worker.seed_policy仍保留uniform；coverage可作为后续实验候选，不自动替换默认配置。

## 复现入口

1. 在同一Noetic/依赖环境生成完整参考关键帧，再用`prepare_holdout_split.py --frame-count 172`
   冻结划分。本轮split_id为`f38246f0fceb838292c5a2bf29d38396ae2f7582b80b5981c9d0151d847bbe5e`，22个heldout中预选21个。
2. 从`config/gaussian.yaml`分别保存独立参数快照，worker.seed_policy设为uniform/coverage；
   joint_render_policy=spatial，evaluation开启隔离与在线，max_output_mib=1024、min_free_disk_mib=1024。
3. 空模型启动worker，`run_baseline.py`完整回放（duration136、rate1、image_queue32、在线子地图、
   回环、keyframe轨迹、持久化Gaussian、Finalize）；前端2线程、后端4线程，固定逻辑CPU2/4/6/8/10/12，
   worker的OMP/BLAS为1。按策略各两次，GPU任务顺序执行，不复制bag。
4. 正常封口后调用`evaluate_heldout_snapshot.py`，给出同一参数快照、当前准备器和
   `--selector-executable /absolute/sdk/bin/livo_atlas_select_gaussian_neighbor`，输出到全新目录。
5. 核对共同查询、完整训练历史、最终物化位姿、浮点指标、全部状态与输入资产，再解读差值。

## 证据与实现

工作区`validation/spatial-heldout-comparison-20261126/`保留四次命令/参数/日志/原生输出/模型/评分，
以及`comparison-audit.json`、`common-depth-audit.json`、`selector-runtime-audit.json`。
`run_comparison.py`、`audit_comparison.py`等是本机可执行记录；跨电脑命令按上面替换路径。
本轮选块使用静态SDK工具，排名代码在被校验的工具本体内。审查后另以共享Noetic工具验证了
动态库变更拒绝，并用更新后的执行保护逐查询重查四次全部选择，结果与归档一致。
最终评分新增工程动态库指纹、硬输出文件限制，以及中断时保留计划并退出130；不改变训练和选块算法。
独立审查剩两项小边界：自定义错误工具的candidate_count目前仅受全局31上限检查；
单独调用原生CLI尚有输出存在检查与创建之间的竞争，评分常规入口使用私有临时目录。

## 后续方向

已完成[固定训练预算对照与空洞诊断](controlled-gaussian-sampling.md)：两策略各两次，
每次150次训练/3000步，冻结同一参考空间选块。覆盖/RGB收益仍存在，单块SSIM下降，
共同深度L1略增、RMSE降低。下一项优先研究定额内的空洞预留比例，不叠加增密或定位反馈。
