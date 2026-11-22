# P6-03c4c 在线覆盖优先采样

持久子地图 worker 启动时选择 `uniform` 或 `coverage`，默认使用 `uniform`。普通新帧和回环后
重建的各帧使用同一个策略。重用执行零步训练，拒绝候选后从确认态恢复。
[设计](superpowers/specs/2026-11-22-p6-03c4c-online-seeding-design.md) ·
[实施计划](superpowers/plans/2026-11-22-p6-03c4c-online-seeding.md)

## 启用

使用新的私有 socket 目录，并让 owner 指向同一 socket。`coverage` 需要启用
`gaussian.persist_submaps`。ROS1 和 ROS2 使用同一个 worker 命令。
[持久化和预览参数](p6-03c3c2b-ros-preview.md#开启)

```bash
mkdir -m 700 "$LIVO_SOCKET_DIR"
"$LIVO_PYTHON" "$LIVO_REPO/tools/3dgs_probe/online_worker.py" \
  --socket "$LIVO_SOCKET_DIR/mapper.sock" --seed-policy coverage
```

## Retail_Street 在线对照

本机使用 RTX 4060 Laptop 和 Noetic，固定 bag、二进制、依赖、节点 CPU 0、图像队列 32
和 1 倍回放，OMP/BLAS 线程为 1。每次使用独立 worker 与输出目录。两侧都启用持久化、
空间联合渲染、ROS 预览和 Finalize。

10 秒短段的两侧各确认了 5 次训练，各有 4 条截止前 READY，唯一子地图成功封口。覆盖优先
执行了 4 次预渲染，P95 为 2.32 ms。前端数组、原始关键帧文件和已有话题语义严格一致。

完整回放两侧均为 172 帧、2 次回环接纳、最终位姿版本 2。轨迹、前端数值数组、517 个原始
关键帧文件和已有话题语义严格一致。13 块子地图均成功封口并持久化。下面数据来自各一次回放。

| 完整回放 | `uniform` | `coverage` |
| --- | ---: | ---: |
| 确认的 view / rebuild / reuse | 163 / 20 / 11 | 163 / 20 / 11 |
| 合并 / stale | 9 / 21 | 9 / 21 |
| 截止前收到的 READY | 84 | 84 |
| 截止前有双块贡献的 owner 接纳 | 73 | 71 |
| 首条截止前 READY | 2.650 s | 2.791 s |
| 162 个配对暖机 view 的训练中位数 | 78.21 ms | 79.05 ms |
| 整项 rebuild 耗时 P95 | 0.937 s | 0.914 s |
| PyTorch 峰值 allocated / reserved | 96.53 / 116.00 MiB | 96.50 / 114.00 MiB |
| worker 峰值 RSS | 1,192.07 MiB | 1,203.11 MiB |
| Finalize 耗时 | 7.163 s | 6.850 s |

每侧有 163 次常规训练调用、207 次 rebuild 内部训练调用，包含一项被拒绝的 rebuild。
两侧 195 项 worker 操作的顺序、确认决定以及内部训练帧和窗口一致，见 `worker-history-check.json`。
覆盖优先共执行 336 次预渲染，P50 为 1.41 ms，P95 为 1.90 ms，累计 0.472 s。
共预留 165,698 个种子位置。配对视角逐帧耗时差的 P50 为 +1.18 ms，P95 为 +7.13 ms。
两侧 pending 与 in-flight 峰值均为 1，worker 干净退出。

## 在另一台电脑复现

把 `LIVO_REPO`、`LIVO_REPLAY_WS`、`LIVO_PYTHON`、`LIVO_BAG` 和 `LIVO_PHASE` 设置为
本机绝对路径。`LIVO_REPLAY_WS` 使用独立 Noetic 工作区，包含启用 Gaussian、GTSAM 和回环的
Release 节点及对应源码快照。CUDA Python 环境可复用 C4b，验证时还需要 PyYAML。
[Noetic 构建与回放](p6-03c3c2b-validation.md#证据位置与复现) ·
[Python 环境](p6-03c4a-validation.md#在另一台电脑复现)

从验证归档复制 `run-replay.sh`、`worker-measure.py` 和 `compare-online-seeding.py` 到新的
`LIVO_PHASE` 目录，再运行下列命令。短段为 10 秒，完整配置覆盖约 135.47 秒的原始 bag。

```bash
export PYTHONPATH="$LIVO_REPO/tools${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1
export CUDA_VISIBLE_DEVICES=0 PYTHONNOUSERSITE=1
for length in short full; do
  for policy in uniform coverage; do
    "$LIVO_PHASE/run-replay.sh" "$length" "$policy" || exit 1
  done
  "$LIVO_PYTHON" "$LIVO_PHASE/compare-online-seeding.py" \
    --phase "$LIVO_PHASE" --length "$length" --uniform "$length-uniform" \
    --coverage "$length-coverage" --output "$LIVO_PHASE/$length-comparison.json" || exit 1
done
```

`*-worker.json` 用会话、job 和训练序号关联实际采样策略、候选及确认。`*-comparison.json`
核验原始输出、训练证据、最终资产和订阅接收。训练时长围绕模型调用测量，整项 worker 耗时还
包含候选处理。首条 READY 来自独立订阅者。匹配视角的延迟只汇总相机、保留窗口和高斯数相同、
配对键唯一的暖机后已确认 `view`。

本机证据在 `validation/P6-03c4c-20261122`。在线预览使用训练视角，留出画质另见 C4b。
[C4b 留出对照](p6-03c4b-validation.md)

## 验证记录

四次回放使用 worker 源码 `7c80adb` 和已核验的 C3c2b Noetic Release 二进制。原始代码、
输入、二进制和依赖版本随报告归档。最终通过 220 项 Python/CUDA 测试和 125 项工具测试。

独立审阅发现复现脚本的路径判断可能覆盖旧日志。修复后用已有普通文件和断链复现验证，
9 个路径场景均在启动 worker 前退出。实测使用的原脚本保存在 `measured-run-replay.sh`。
修复涉及回放前的路径检查，两份最终配对报告基于已留存产物重新核验，结果逐字节一致。
