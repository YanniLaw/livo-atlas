# Retail_Street 整包 Noetic 分层补点对照

本轮比较 `flat` 与 `stratified_additions` 在完整在线回放中的处理和建图表现。
两侧使用同一节点构建和当前 Python worker，各自从空模型启动，开启回环、位姿图、
空间联合渲染、在线留出和持久化封口。唯一训练设置差异是补入点分布，两侧均使用
`coverage / replace / 0.25`。前一轮的冻结前缀和 10 秒回放见[分层补点实验](stratified-additions.md)。

## 控制条件

- 原始 Retail_Street.bag，1 倍速播放 136 秒，覆盖全部约 135.47 秒输入。图像队列 32，
  前端 OpenMP 2、后端线程池 4，节点和加载器沿用逻辑 CPU 2、4、6、8、10、12。
  worker 单独启动，OMP 和 BLAS 均为 1。
- 复用原整包验证的 Noetic 二进制和运行库，核对 294 个源码、构建和配置文件，准备独立工作区。
  两侧运行参数和共享库均绑定摘要，记录回放前 GPU 温度和可用显存。worker 使用 Python 3.10.22、
  PyTorch 2.3.0+cu121 和 gsplat 1.5.3。
- 整包运行先按 flat、additions 串行执行。若全前端、LIO、VIO 或训练 P95 的绝对差落在
  相同运行方式的历史重复差异内，则反序再跑一对。重复门槛在启动前从四份历史记录读出，
  依次为 0.258、0.646、0.272 和 3.270 ms。
- 两侧使用同一冻结 split，172 个关键帧中 22 个留出，预选 21 个查询。在线留出任务各有自己的
  `as_of`，按成功、取消和失败记录。所有回放完成后才对持久封口的模型做统一最终评分，
  复核 21 个单块、19 个联合视角的参考图像、相机与选块。
- 全图覆盖使用独立图像分母。共同区域在所有本次成功模型中按 alpha≥0.05 取交集，
  弱梯度沿用参考阈值 0.02，SSIM 使用完整 11×11 窗口。共同深度按交集内有效深度标签
  累计 L1 和 RMSE。覆盖周转始终用同组 flat 为参照。

## 回放与封口

首对 VIO P95 差值为 −0.214 ms，触及事前 0.272 ms 重复门槛，随后反序完成第二对。
每轮接收 27447 条 IMU、1355 帧 LiDAR 和 1355 帧图像，完成 1352 次 LIO 和 1351 次 VIO 更新。
172 个关键帧共提交 13 次回环，接纳 2 次，最终位姿版本为 2。每轮以 13 个当前子地图
正常封口，全部持久化并收到了 worker 退出确认。在线留出各成功 20 次，键 `169` 在停机时
取消，失败和待处理均为 0。四轮的轨迹、前端与 IMU 状态文件、语义输出、回环接纳记录
以及最终物化位姿一致。

| 执行顺序 | LIO P95 (ms) | VIO P95 (ms) | 前端整体 P95 (ms) | 全部训练 P95 (ms) | 截止前请求内训练 P95 (ms) |
| --- | ---: | ---: | ---: | ---: | ---: |
| `flat-1` | 49.332 | 18.587 | 47.004 | 179.117 | 181.226 |
| `additions-1` | 47.882 | 18.373 | 45.934 | 187.319 | 190.531 |
| `additions-2` | 48.545 | 19.014 | 46.131 | 183.816 | 189.133 |
| `flat-2` | 48.308 | 18.767 | 46.510 | 185.110 | 186.559 |

候选侧前端整体 P95 分别低 1.070 和 0.379 ms。LIO、VIO 和全部训练 P95 的差值在两对中
方向相反。仅统计在输入截止前已发起的 worker 请求所含训练，候选侧 P95 分别高 9.305 和
2.574 ms。一次 worker 请求可以包含多次训练，这一口径以请求开始判定。每次训练 20 步，计时包含重新训练。

| 运行 | 训练调用（截止前请求内 / 全部） | 合并 / 过期 | 封口 (s) | 其中修复 (s) | ROS 停机 (s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| `flat-1` | 190 / 281 | 26 / 19 | 18.499 | 17.870 | 19.337 |
| `additions-1` | 192 / 283 | 25 / 20 | 19.400 | 18.824 | 20.313 |
| `additions-2` | 180 / 273 | 26 / 19 | 19.422 | 18.805 | 20.345 |
| `flat-2` | 180 / 270 | 27 / 22 | 18.534 | 17.961 | 19.287 |

高斯队列待处理和执行中峰值各为 1。两对候选侧封口均约多 0.9 s，修复耗时占据这段差异的大部分。
四轮节点和 worker 观测到的峰值 RSS 分别为 864–873 MiB 与 1179–1205 MiB，CUDA 分配器峰值
约分配 96.5 MiB、保留 114 MiB。回放起始 GPU 温度依次为 48、55、58、58 ℃。
实际派发和合并的训练负载会随调度改变，以上尾延迟是本机两对端到端观察值。

## 最终留出画质

每轮 21 个单块、19 个联合视角均完成评分。四轮的参考图像、相机和选块一致，训练收据与
封口状态中的训练谱系均通过留出隔离验证。下表为同对 `stratified_additions − flat`。

| 配对 | 视图 | 全图覆盖 (百分点) | 全图 SSIM | 全图 PSNR (dB) | 共同弱梯度 SSIM | 共同深度 L1 (mm) | 共同深度 RMSE (mm) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 单块 | +0.862 | +0.00785 | +0.132 | +0.01115 | −3.550 | −3.237 |
| 1 | 联合 | +0.397 | +0.00734 | +0.169 | +0.00934 | −3.467 | −0.313 |
| 2 | 单块 | +1.283 | +0.00502 | +0.128 | +0.00505 | −0.017 | +0.578 |
| 2 | 联合 | +0.385 | +0.00533 | +0.132 | +0.00692 | −0.239 | +5.521 |

四份地图交集在单块和联合视角上分别有 5,116,513 和 5,363,911 个像素，
2,016,380 和 2,425,956 个弱梯度完整窗口，以及 230,706 和 231,200 个有效深度标签。
联合渲染新增覆盖为 2.258 和 2.381 个百分点，同时丢失 1.860 和 1.996 个百分点。
单块新增覆盖为 4.325 和 4.636 个百分点，同时丢失 3.464 和 3.353 个百分点。

两对都观察到净覆盖和共同纹理的改善。共同深度 L1 的降幅在反序对缩小，RMSE 则升高。
实时链路完成了本机这段数据的 1 倍速输入，两对候选侧都观察到略长的封口时间。
这些结果的适用范围是这一条约 135 秒的路线和本机 Noetic 调度条件。

## 复现入口

使用[既有 Noetic 整包验证的构建和冻结 split](spatial-heldout-comparison.md#复现入口)，
先在独立的运行工作区准备节点。复制同一配置文件两份，分别选择两个分布。
配置中令 `gaussian.joint_render_policy` 为 `spatial`，开启 `gaussian.evaluation.enabled`
和 `gaussian.evaluation.online_enabled`，填写冻结 `split_file`。评估输出与剩余磁盘各按
1024 MiB 设置边界。以下是候选配置的 worker 段，对照将 `seed_distribution` 设为 `flat`。

```yaml
gaussian:
  worker:
    seed_policy: coverage
    seed_distribution: stratified_additions
    coverage_sampling: replace
    coverage_uncovered_ratio: 0.25
```

每轮用新输出目录和当前用户拥有的私有 socket 目录。在安装了 CUDA 依赖的 Python 环境中
启动 worker。实际测量用同一入口的 `worker-measure.py` 包装器记录训练耗时、种子配额和资源峰值。
路径和端口按本机替换，线程配置和 CPU 绑定在同一配对内固定。

```bash
mkdir -m 700 /data/private
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 \
  python tools/3dgs_probe/online_worker.py \
  --config /data/candidate.yaml --socket /data/private/worker.sock
```

保持 worker 运行，在另一个终端执行回放命令。

```bash
source /opt/ros/noetic/setup.bash
OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=4 OMP_DYNAMIC=FALSE \
  taskset -c 2,4,6,8,10,12 /usr/bin/python3 tools/run_baseline.py \
  --workspace /data/noetic-snapshot --bag /data/Retail_Street.bag --output /data/run/slam \
  --port 11881 --duration 136 --rate 1 --image-queue-size 32 \
  --submap-mode online --loop-closure --keyframe-trajectory --semantic-output --stage-timing \
  --gaussian-socket /data/private/worker.sock --gaussian-output /data/run/gaussian \
  --gaussian-persist-submaps --gaussian-finalize-on-shutdown --gaussian-finalize-timeout-ms 120000 \
  --gaussian-evaluation-split /data/split.json --gaussian-online-evaluation \
  --gaussian-evaluation-max-output-mib 1024 --gaussian-joint-render --joint-render-policy spatial
```

所有回放结束后，在 CUDA 环境中依次运行各轮最终快照的评分命令。

```bash
python tools/3dgs_probe/evaluate_heldout_snapshot.py \
  --map-directory /data/run/gaussian --keyframes /data/run/slam/keyframes \
  --prepare-executable /sdk/bin/livo_atlas_prepare_3dgs_views \
  --selector-executable /sdk/bin/livo_atlas_select_gaussian_neighbor \
  --config /data/candidate.yaml --output /data/run/final-scores
```

按配对顺序运行，每次只启动一个测量中的 GPU worker。本机归档位于工作区的
`validation/noetic-additions-20261127/`，约 6.4 GiB。`protocol.json` 绑定运行输入，
`run-receipts.json` 绑定四轮回放与最终评分，`audit.json` 保存计时、质量指标和文件摘要。
`audit_results.py --verify` 可重新核对运行结果和归档收据。

[设计](superpowers/specs/2026-11-27-noetic-additions-design.md) ·
[执行计划](superpowers/plans/2026-11-27-noetic-additions-validation.md)
