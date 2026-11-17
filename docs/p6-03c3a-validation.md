# P6-03c3a：Gaussian 最终收敛验证

2026-11-18。实现按[设计](superpowers/specs/2026-11-18-p6-03c3a-gaussian-finalization-design.md)和
[实施计划](superpowers/plans/2026-11-18-p6-03c3a-gaussian-finalization.md)推进。
当前候选实现及 7 次成组 bag 验收已通过；最终整范围审阅和审阅后复验进行中。

## 实现与兼容性

- 成功完成的 `LivoSystem::IsFinished()` 才允许冻结目标，单独封输入或终端失败不会放行。
- `Finalize()` 排空旧 accept 回执后固定已有内容 E，只修复历史几何状态，随后执行原持久收尾。
  普通 Finish、默认关闭的新参数、C1 及原持久模式保持。
- 最后一个 dirty 模型也会保存，目录确认和 stop 完成后才计算结果；报告发布单独记账。
- `finalization.json` 上限 16 KiB，已有 IPC、资产和 map.json schema 均未改变。
  具体配置、预算和报告语义见[使用说明](online-gaussian.md#可选的最终位姿收敛)。

## 环境及验证入口

新产物只写入 `validation/P6-03c3a-20261118`；保留原 C2b 正式数据、官方 bag 和已有依赖。
宿主 Ubuntu 20.04 / ROS1 Noetic / GCC 9 / CMake 3.16；Docker `ros2` 为 Ubuntu 22.04 / Humble。
沿用 Sophus、GTSAM 及 P6-03b 的 Python 3.10、NumPy 1.26.4、PyTorch 2.3+cu121、
gsplat 1.5.3 预编译环境，实际 GPU 为 RTX 4060 Laptop 8 GiB。
工具测试另外使用阶段内隔离的 rosbags 0.10.9、PyYAML 6.0.2 前缀，不改变已有 GPU 环境。

```bash
cd /path/to/livo-atlas
cmake -E chdir /path/to/phase/core ctest --output-on-failure -j1
/path/to/phase/run-python.sh -m unittest discover -s tests/3dgs_probe -v
/path/to/phase/run-tools.sh -m unittest discover -s tests -p 'test_*.py' -v
```

上述包装脚本包含本机路径，换机器须按使用说明重建环境和设置相应目录。
Noetic/Humble 仍使用同一个 `src/app/livo_main.cc`；Humble 在此仅验证真实入口与受控 IPC，
实际 CUDA 和官方 ROS1 bag 在宿主运行，不能将 peer 验证解释为 Humble GPU 验证。

## 已完成的基础验证

| 项目 | 结果 |
| --- | --- |
| ROS 无关完整核心、普通公开头 | 67/67 CTest 通过 |
| Python/真实 CUDA 套件 | 110/110 通过，要求实际 GPU，无回退 |
| 完整工具测试 | Python 3.10：76/76；宿主 3.8：71 通过，1 项迁移版本限制跳过 |
| Noetic 参数及完整 catkin 汇总 | 12/12 参数；catkin_test_results 输出 1318 tests、0 failures |
| Humble 完整构建与测试 | 70/70 CTest，参数 14/14 |
| 双 ROS 共用入口 | C1、默认持久、最终收敛空集合、报告发布失败均通过；模拟时钟暂停 |
| 双 ROS 编译关闭 | 运行时请求在创建输出前拒绝，关闭的节点未链接可选 Gaussian 库 |
| 普通 SDK 安装 | shared/static 各 OFF/ON 消费通过；OFF 无可选目标 |
| Humble ament 安装消费 | 新的 IsFinished/Finalize 符号由安装库解析 |
| 真实 worker 新 API | 两块已有内容最终修复、最后保存、报告发布及正常停止通过 |

受控终端测试有 22 个常规 CPU 场景、3 个独立 GDB 调度暂停场景，另一个实际 worker 用例独立运行。
普通 CTest 跳过外部控制的 GDB/GPU 用例，其独立实际运行才计为成功。覆盖晚到 accept、未决候选拒绝、前台重试阻塞历史修复、尾部位姿、
空/缺失/容量、幂等、刚性重用、修复超时和独立收尾、错误 stop、map 提交不确定、
报告 rename 后目录同步失败。原持久 peer 和协议/写盘测试继续通过。
报告发布失败的双 ROS 用例会返回 1，同时仍结束其余轨迹及几何消费者。

## 官方回放与身份

使用唯一的 `Retail_Street.bag`，约 135 秒，SHA256：
`1745bee1d570749679bad7a0392100477ab62d50aa77f7f629bc57f2a72e4427`。
新阶段 `freeze.py` 比对开发源码和隔离 Noetic 源副本；产品文件哈希及完整源码归档、
21 个节点/库文件哈希、bag 哈希随运行保留。回放使用 CPU 0、rate 1、图像发布队列 32、
单线程 OMP/BLAS、相同后端/标定，构建和其他 GPU 测试不与正式回放并行。

10 秒和完整 bag 均运行 OFF、原持久模式、Finalize；C1 另跑 10 秒。
检查四数组逐值、图像/点云发布语义、原始关键帧逐字节、后端最终前沿及接纳数。
Finalize 额外核对完整 canonical primary、逐资产 SHA/数组、冻结 E、报告与日志、发布及退出。
在线 resources.csv 和停止期间 shutdown-resources.csv 分开；原 Gaussian 总计数和延迟桶
累计到停止，before/after 只分隔 Finalize 入口，不能将终端修复算成纯在线吞吐。

候选 `cand` 共 7 次运行通过。10 秒片段有 5 帧/1 块，完整数据有 172 帧/13 块。
完整数据 OFF、默认持久、Finalize 均为 1352 LIO / 1351 VIO；两次后端接纳，最终 V2。
四数组完全一致：trajectory 1352×8、mat_pre 2707×19、mat_out 2703×20、imu 27385×7；
517 个原始文件逐字节相同，发布语义一致。C1 短段也通过。

| 完整数据候选模式 | current | needs_rebuild | missing/dirty/未引用 |
| --- | --- | --- | --- |
| 原持久模式 | 5 | 8 | 0/0/0 |
| Finalize | 13 | 0 | 0/0/0 |

候选 Finalize 在入口前已有 9 次累计 rebuild，终端另做 9 次，accepted 172→181、
effective_training 165→174、checkpoint 29→38。修复 13.543290s，最后持久收尾
0.504965s，整个节点退出约 15.064s；修复预算为 60s，持久收尾预算为 30s。
最终 map_sequence=408，引用 13 个资产、703,846,913 字节。
两个启用模式的在线异步 GPU 确认数量可不同，因此默认的 5/8 与 Finalize 入口前的
4/9 分类不同；原始观测、最终前沿及前端行为完全相同，E 始终是实际确认的 13 块。

候选二进制 SHA256：`7a7e526105c54f18cfecad4cef9acc437f6d4ce36ed709873d2554dda545ab49`。
新 GPU 实例已逐块冷恢复候选的 13 个资产，参数及 origin ID 逐值一致，渲染均有限；
其中 1 张与保留确认预览匹配，RGB8 最大差异为 0。
受控真实观测还通过拒绝恢复、刚性零步重用、窗口外成员变化与跳版本重建、32/33 容量边界、
提交不确定时资产保留。其位姿修正和容量 ID 是合成序列，RGB/深度来自真实资产，
不能当作第二段独立道路数据。GPU 峰值 allocated 101,128,704、reserved 117,440,512 字节。
最终复验数据将在实际完成后补入，本文件当前不宣告最终阶段验收通过。

## 独立审阅与修正

一次完整独立审阅发现 1 项 Important、无 Critical/Minor。在线程被调度暂停或目标准备跨过
截止时间后，旧实现可能仍派发修复、接纳候选或进入成功收尾。三种边界均已用真实 owner +
GDB 300ms 暂停、200ms 预算确定性复现；修正后均通过。产品没有新增测试时钟或调度钩子。
目标校验后、mailbox 提交前及成功收尾入口重新核对 steady deadline，原有允许慢 cleanup
超出修复预算的正例仍通过。按计划只做这一轮统一修复，不追加复审。

```bash
python3 tests/gaussian/finalization_deadline_test.py \
  --binary /path/to/phase/core/gaussian_finalization_test
```

核心 67/67、Python/CUDA 110/110、工具 76/76 在修正后重新通过，受影响的可选 ON 构建和
安装消费正在复验；最终成组回放尚待完成。普通 OFF 源文件及公开接口未受此修正影响。

## 限制

succeeded 只说明 E 的已有内容与指定最终位姿一致并完成持久收尾；每块仍至多 10 个视图、
10,000 个 Gaussian，不表示全部关键帧/场景已训练，也不表示画质、定位精度或完整 3DGS SLAM。
missing 明确保留；正式 bag 正例另要求 missing=0。超时可同时 durable=true，仍不得当作成功。
一个可见报告不能证明原进程已完成 fsync，必须结合 report_persisted 和正常退出。
现有短 bag 不能替代长距离精度、地图质量或长期耐久验证。
