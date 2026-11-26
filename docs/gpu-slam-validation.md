# 当前版本 GPU 与双 ROS 全链路验证

本轮 C++ 验证基于 `bba0c8b`，另包含接收工具修复 `f2b94b0`，检查邮箱竞争修复后的
实际 CUDA worker、完整 SLAM 与联合渲染。
不新增算法功能；每个会话使用独立 socket、输出目录和 worker，顺序运行。

## 执行与验收

- [x] 真实 worker：历史资产渲染与修复、空间非相邻选块、预览写入故障后三项用例，均实际运行通过，没有跳过。
- [x] Noetic：完整 `Retail_Street.bag`，开启回环、位姿图优化、持久高斯、空间联合渲染、ROS 预览及 Finalize。
- [x] Humble：复用相同 worker 源码与 CUDA 环境，验证相同功能及 ROS2 消息交付。
- [x] 核对输入/输出计数、回环归档、训练证据、最终位姿版本、资产与封口结果、worker 和节点退出。
- [x] 汇总训练与渲染延迟、显存和 RSS，记录实际覆盖及限制；保存可复现命令与摘要。

## 资源与证据

证据目录为工作区 `validation/gpu-slam-20261126/`。复用现有 Release 构建，不复制 bag；
保留输入文件和既有验证数据。只有本轮成功验收后的新关键帧图像/点云允许精简，删除前
保存逐文件 SHA256，保留元数据、轨迹、日志、高斯资产及摘要。失败会话保留现场。
首轮失败会话的原始载荷与高斯资产以无损压缩归档保留，核对归档内全部文件的 SHA256
后移除解压副本；没有丢弃失败数据。

本机可用 GPU 为 RTX 4060 Laptop 8 GiB，worker 使用现有 Python 3.10 / Torch 2.3.0+cu121 /
gsplat 1.5.3 环境。GPU 用例与回放顺序执行。跨 ROS 的数值差异按
[既有诊断](cross-ros-numerics.md)解释，不要求不同依赖版本逐位相同。

## 结果

两端均以1倍速覆盖完整135.47秒输入，前端 OpenMP 预算为2，后端线程池为4；节点使用
逻辑 CPU 2、4、6、8、10、12，worker 的 OMP/BLAS 预算为1，采样策略为 `uniform`。
Humble 列采用修复接收工具后的完整重跑结果。

| 项目 | Noetic | Humble |
| --- | ---: | ---: |
| IMU / LiDAR / 图像输入 | 27447 / 1355 / 1355 | 相同 |
| LIO / VIO 更新 | 1352 / 1351 | 相同 |
| 轨迹行数 / 关键帧数 | 1352 / 172 | 1352 / 169 |
| 回环完成 / 接纳 / 最终位姿版本 | 13 / 2 / 2 | 相同 |
| 最终 current / eligible 子地图 | 13 / 13 | 相同 |
| 已接纳联合渲染 / 双块请求 | 67 / 55 | 62 / 52 |
| 过期渲染 / 渲染失败 | 35 / 0 | 36 / 0 |
| 输入截止前实际收到的 READY | 65 | 56 |
| 内部训练调用数（含重建） | 339 | 332 |
| 训练调用 P50 / P95（ms） | 147.63 / 179.03 | 139.54 / 182.68 |
| 已接纳渲染派发至 owner 接纳 P50 / P95（ms） | 188.60 / 275.75 | 179.19 / 250.00 |
| PyTorch 峰值 allocated / reserved（MiB） | 96.53 / 114.00 | 96.53 / 114.00 |
| 节点采样 RSS 峰值（MiB） | 890.58 | 930.04 |
| worker RSS 峰值（MiB） | 1195.63 | 1199.86 |
| Finalize（秒） | 17.70 | 16.36 |

输入计数、输出有限性、序列化载荷（Humble）及正常退出检查通过，回环归档均为
`consistent`。两端逐份读取13份最终状态，验证 manifest 与载荷 SHA256、数组、保留
视图、来源和 Gaussian 数量。最终均无 missing、needs_rebuild 或 dirty，pending 与
in-flight 归零，峰值各为1。实际订阅图像与保留 PNG、空间选块相机及 owner 来源身份核对通过。

Noetic 的 `trajectory.tum`、`mat_pre.txt`、`mat_out.txt`、`imu.txt` 与既有同环境完整
参考逐字节一致；Humble 的同四份文件及 `odometry.txt` 也与其既有参考逐字节一致。
参考路径及逐项结论保存在 `frontend-reference-check.json`，跨 ROS 不作逐位一致承诺。

修复后 SDK CTest 为78/78，两端相关 Python 测试各82项通过。独立只读复核无待修问题。
训练调用统计包含重建内部调用及随后可能被拒绝的候选；渲染延迟只汇总 owner 接纳样本，
过期结果单列，双块请求数不等同于每次都产生可见贡献。这些是各一次运行的实测，不能
作为修复前后性能收益、在线画质或长距离精度结论。

两端订阅者均未收到退出时的 `STOPPED`，发布统计各记录1次 unavailable；这是仍待收尾
的正常退出通知交付问题。封口、worker 停止和节点正常退出已独立通过，不把这些结果
表述为终止消息已可靠送达。
该限制随后已在[双 ROS 正常退出验证](graceful-shutdown-validation.md)中修复；本节保留
当时完整回放的原始观察，不回写历史结果。

原始 ROS1 bag 与冻结 Noetic 源码/运行库校验通过，ROS2 转换数据与此前基准的三份文件
校验和一致。成功会话分别精简344、338份关键帧载荷，共437436233字节；清单与 SHA256
先于删除保存。首轮失败会话797份文件、946437493字节无损归档为311093075字节，逐文件
校验通过。原输入和既有参考数据均未删除。

## 实测发现与修复

首轮 Humble 已完成1352行轨迹、2次回环和13块高斯封口，worker 正常退出，但预览接收器
报告 `Invalid Gaussian camera`，因此整轮回放仍按失败保留。实际生成的 ROS2 消息中，
固定长度 `k` 数组的元素类型是 `numpy.float64`；接收器直接复制列表后，原有 JSON 数字
校验拒绝了该类型。修复只在消息转换入口显式转为 Python `float`，不放宽记录校验，
不改变节点或算法数据。先用非原生数值标量和实际 Humble 生成消息复现，再验证修复，
最后重跑完整 Humble 链路；失败首轮不覆盖。

## 跨机器复现

沿用[工程回归工具](regression-tools.md)的构建方式，先准备相应 ROS 环境和已构建节点，
使用新的私有 socket 目录（属于当前用户，权限0700）。三项 GPU 用例必须分别启动
独立 worker，每个 worker 只服务一个会话。测试二进制通过 `LIVO_REAL_GAUSSIAN_SOCKET`
和 `LIVO_REAL_GAUSSIAN_OUTPUT` 接收路径，逐项选择：

```text
OnlineGaussianRenderTest.RealWorkerRendersAssetsDuringHistoryRepair
OnlineGaussianRenderTest.RealSpatialWorkerSelectsNonadjacentAssets
OnlineGaussianRenderTest.RealWorkerContinuesAfterPreviewRenameFailure
```

worker 启动命令为：

```bash
"$LIVO_PYTHON" "$LIVO_REPO/tools/3dgs_probe/online_worker.py" \
  --config "$LIVO_REPO/config/gaussian.yaml" --socket "$LIVO_SOCKET_DIR/worker.sock"
```

完整回放使用已有 `tools/run_baseline.py` 或 `tools/run_ros2_baseline.py`，共同关键选项为
`--duration 136 --rate 1 --image-queue-size 32 --submap-mode online --loop-closure
--keyframe-trajectory --stage-timing --gaussian-persist-submaps --gaussian-finalize-on-shutdown
--gaussian-joint-render --joint-render-policy spatial --gaussian-publish-preview`，并提供独立的
`--gaussian-socket`、`--gaussian-output` 和回放 `--output`。Noetic 需要独立源码/节点快照
工作区；Humble 需要转换后的 ROS2 bag 和匹配生成消息。Humble 的本轮调用另带
`--default-threads`，允许节点按配置使用前端线程预算2。

本机 Humble 容器没有 GPU 设备，worker 在宿主机运行；节点与 worker 以相同 UID 使用
已有共享挂载的相同绝对路径。本机复用 `/tmp/.X11-unix/` 下独立的0700子目录，结束后
移入本轮证据目录。其他机器建议挂载专用共享目录，并在两侧使用同一绝对路径；只共享
socket 而没有共享高斯状态目录是不够的。

`run_real_cases.py`、`run_noetic.py`、`run_humble.py`、`run_humble_retry.py` 和实际参数、
worker 命令、错误/通过日志保存在证据目录。`inspect_run.py` 汇总状态恢复、来源、接收
证明与资源数据；`noetic-inspection.json`、`humble-retry-inspection.json` 和 `summary.json`
保存最终结果。失败首轮位于 `humble-first-artifacts.tar.gz`，归档逐文件清单与校验结果也保留。
