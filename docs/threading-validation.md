# 前端线程预算与回环任务验证

本记录对应 2026-11-24 的两项实施计划。
[前端 OpenMP](superpowers/plans/2026-11-24-frontend-openmp.md) ·
[回环候选依赖任务](superpowers/plans/2026-11-24-parallel-loop-candidates.md)

## 配置与实现

`LIVO_ATLAS_USE_OPENMP` 默认 `AUTO`，按目标工具链探测 C++ OpenMP。
`ON` 要求提供能力，`OFF` 关闭项目引入的相关依赖和并行区。旧 ON/OFF 缓存和
TRUE/FALSE、YES/NO、1/0 等常见布尔值继续接受。内部私有能力宏
`LIVO_ATLAS_HAS_OPENMP` 由探测结果设置，不向 SDK 消费者导出。
即使外部编译参数带 `-fopenmp`，项目选择 OFF 时前端仍串行运行。

四份传感器 YAML 增加 `frontend.openmp_threads`，默认 2，必须为正整数。
SDK 对应 `FrontendOptions::openmp_threads`。每个实例的 LIO/VIO 并行区使用自己的预算，
不修改 OpenMP 默认设置；无 OpenMP 时接受相同参数，通过 GLOG 记录串行回退。
运行库的线程上限和动态调度仍可缩小实际团队。

LIO 使用独立字节标记写入逐点结果，再按输入顺序归并。VIO 将逐点误差和计数写入独立槽，
按观测顺序汇总，保持串行 EKF 的终止判定。后者增加 O(视觉观测数) 的临时存储。
初次保留浮点 reduction 的完整回放出现了约 2.27 厘米的前端位置差，并被后端放大；
固定汇总顺序后，本机各线程预算的前端输出逐值一致。

后台仍由 `LivoSystem` 内的 `PoseGraph` 管理。默认 4 个线程，可用
`pose_graph.num_background_threads` 设置 6。候选任务复用同一线程池，
用 Cartographer `Task` 依赖连接准备、验证、汇总和私有试算，正式接纳走串行 `WorkQueue`。
完整任务图在派发时注册，工作线程不等待或追加任务。

池大小为 N、几何驻留预算为 R 时，每批最多 `min(max(1, N-1), R-1)` 个候选。
默认 R=3，包含源块，所以 4/6 线程池均最多并行验证两个候选。可通过
`loop_closure.geometry.max_resident_submaps` 调整。源快照在批次间驻留，
上一批释放目标快照后，下一批才读取几何。

## 构建和生命周期检查

验证环境为本机 x86_64、原生 Noetic 和 Docker 内的 Humble。ARM 尚未实测。

| 检查 | 结果 |
| --- | --- |
| 独立 SDK | 76 个 CTest 目标通过 |
| Noetic | 9 个核心 CTest 目标及 40 个 ROS 用例通过 |
| Humble | 10 个相关 CTest 目标通过 |
| 安装后 SDK | 前端、位姿图、回环及接纳的 4 个消费者通过 |
| 生命周期重复 | 回环后端、查询任务和线程池各重复 10 次通过 |
| 构建能力 | AUTO/ON/OFF、缺失能力、旧布尔值、同目录重配置和导出依赖共 9 项通过 |
| OFF 与外部 `-fopenmp` | 三种前端模式通过，实际保持串行 |
| 多实例预算 | 两个同时存活的前端交错处理，实际 LIO 团队分别为 1 和 4 |

候选测试用可控调度器反转完成顺序，并覆盖 1/4/6 线程、驻留预算 2/3/6、空候选、
资源上限、读取失败和线程池析构。对五候选依赖图的 12 次提交逐个注入失败，
已提交任务能够排空，失败提案不重复交付。

独立复核发现未提交 Task 析构可能留下悬空依赖。AddressSanitizer 探针先复现了
`heap-use-after-free`，修复后通过原探针和 4 线程下 10000 次取消依赖的压力检查，
同时启用了泄漏检测。NEW Task 暂存前驱所有权，析构时撤销订阅；成功提交后由线程池
持有到完成。查询内部错误与派发失败的结果分开保存，避免并发读写同一个异常字段。

工程测试建议在独立构建目录执行。

```bash
cmake -S src/livo-atlas -B build-core \
  -DLIVO_ATLAS_ROS_VERSION=0 -DLIVO_ATLAS_USE_OPENMP=AUTO \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build-core -- -j2
(cd build-core && OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=4 \
  ctest --output-on-failure)
```

非标准依赖路径按 [SDK 构建文档](core-build.md) 提供。Noetic 执行 `catkin_make tests`
后再运行测试，Humble 构建测试目标后执行 CTest。修改公共数据结构后需重建消费者，
避免旧测试程序与新共享库混用。参数测试需要各自的 ROS 环境；ROS1 需启动测试 master。

## 完整 bag 对照

以下历史延迟包含线程审计探针开销。后续采样发现探针对 FLANN 查询有显著影响，
性能评估需另做不带该探针的回放；线程数、数值与归档检查仍有效。
见 [采样与重复对照](gicp-cpu-profile.md)。

使用工作区的 `Retail_Street.bag`，每次新建输出目录，以 1 倍速完整回放。
五组配置共用 CPU 集合 `2,4,6,8,10,12`，固定其他参数；构建任务使用其他 CPU 核。
外部 OpenMP 限额为 4，探针确认 LIO/VIO 实际使用请求的 1/2/4 个线程。

| 前端预算 | 后台线程 | 实际候选峰值并发 | 本次查询最长交付延迟 |
| --- | --- | --- | --- |
| 1 | 1 | 1 | 6.22 s |
| 1 | 4 | 2 | 7.38 s |
| 1 | 6 | 2 | 6.96 s |
| 2 | 4 | 2 | 5.73 s |
| 4 | 4 | 2 | 7.02 s |

每组均完成 1352 次 LIO、1351 次 VIO、172 个关键帧和 13 次查询；验证 20 个候选，
接纳 2 次回环，最终位姿版本为 2。全部归档检查返回 consistent。
轨迹、原始关键帧、成员修订及语义输出与串行基线字节一致；去除耗时后的候选诊断一致。
最终位姿对旧基线的最大矩阵元素差为 2.16e-15。

复核修复后的最终程序另做了一次前端 2、后台 4 的完整回放，保持相同计数、诊断和资产，
最终位姿最大差为 1.72e-15，实际候选并发为 2。

这些样本证明了并发和数值回归，端到端延迟尚未显示稳定收益。下一步补齐几何读取、构建、
验证、试算及交付的分段计时，再依据数据选择优化对象。当前 bag 约 135 秒，不代表
长距离运行效果。

工作区证据在 `validation/frontend-openmp-20261124/` 和
`validation/parallel-loop-candidates-20261124/`，包括构建日志、探针、回放与 comparison.json。
另一台电脑可按 [ROS 回放文档](ros-builds.md) 使用 `tools/run_baseline.py`，分别修改
传感器 YAML 和 backend YAML 中的预算，用新的输出目录完成上述配置矩阵。

## 后续测试覆盖

将两个同时存活实例的交错团队验证加入长期 GTest 回归仍是后续项。
本次保持每个会话由单一所有者推进，不扩展多个前端同时处理的线程安全保证；
原有全局平面 ID 计数器需要在支持该用法时单独审计。
