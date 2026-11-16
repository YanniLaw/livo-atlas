# P6-03c2b 在线 Gaussian 子地图验证

本阶段实现可选的历史工作集、确认回退、刚性重绑、窗口重建、持久目录和有限调度。
使用方式见[在线说明](online-gaussian.md)，契约见[设计](superpowers/specs/2026-11-17-p6-03c2b-online-gaussian-submaps-design.md)
与[计划](superpowers/plans/2026-11-17-p6-03c2b-online-gaussian-submaps.md)。

本文先记录候选源码的实际证据；独立审阅后，最后一节补充最终源码身份及复验结果。
候选数据不冒充修复后的二进制验证。所有产物位于包外
`validation/P6-03c2b-20261117/`，迁移机器需另行保存或按命令重新生成。

## 环境与来源

Noetic 主机为 Ubuntu 20.04、GCC 9.4、CMake 3.16；Humble 容器为 Ubuntu 22.04、GCC 11.4、
CMake 3.22。使用既有 Sophus、GTSAM 4.2 和独立 Python 3.10.22/NumPy 1.26.4/PyTorch
2.3.0+cu121/gsplat 1.5.3 环境，预编译 gsplat.csrc。GPU 为 RTX 4060 Laptop，驱动
535.183.01，起跑检查约 7300 MiB 空闲、8188 MiB 总量。未升级依赖、驱动或系统环境。

CPU 资产检查另外在本机 Python 3.8/NumPy 1.22.3 上通过。推迟类型注解求值，并在已有
4096 字节 NPY 头部、dtype、shape 和 payload 检查后使用兼容的 NumPy 读取参数；
`allow_pickle=False` 与 C2a schema 1 保持。

官方 Retail_Street.bag 的 SHA-256：

```text
1745bee1d570749679bad7a0392100477ab62d50aa77f7f629bc57f2a72e4427
```

输入约 135 秒。候选节点由 `candidate-source.tar` 构建到独立 `noetic-ws`，源码归档、节点及
全部本包共享库分别记录 SHA-256；每次回放结束复核 bag 和共享库，配对比较再核对源码文件、
节点二进制及所有共享输入配置。CPU 0、1 倍速、图像队列 32、OpenMP/BLAS 单线程；其他
机器应选允许的相同 CPU。正式回放期间不并行编译或执行其他 GPU 验证。

## 自动检查与构建

- 普通核心 Gaussian/GTSAM/回环开启构建的 66 个 CTest 目标通过；Gaussian map writer
  8 项、持久 mapper 20 项（18 项受控 socket，2 项真实 GPU 单独运行）、原 C1 mapper
  23 项（21 项受控，2 项真实 GPU 单独运行）。跳过的 GPU 用例不计为 GPU 通过。
- Python/CUDA 107 项通过；比较器 21 项、runner 18 项通过。新增检查拒绝零步重用冒充训练、
  错会话/绑定、错误或缺失资产、超预算、volatile 冒充持久化、组内变形误报 current、
  incomplete 停机；允许明确标记的历史重建输入。
- 普通 shared/static Gaussian OFF/ON 安装消费通过；OFF 不导出可选目标，ON 可只链接
  `livo_atlas::online_gaussian`。动态依赖检查未引入 ROS、torch、CUDA 或 vikit 到普通 SDK。
- Noetic 11 项、Humble 13 项参数用例通过。两端实际共用入口在暂停仿真时钟、零传感器输入下
  完成 C1 reset 或 C2b 空目录持久提交，正常结束四类消费者。编译 OFF 的两端在创建数据集
  和 Gaussian 输出前拒绝启用请求；Humble ament 安装消费者也链接/运行通过。
- 所有普通公开头独立编译。最终源码改动对应的矩阵复验见末节；Humble 的 GPU 覆盖仍为
  受控 peer，真实 CUDA 仅在 Noetic 主机运行。

实际失败先复现再修正：空激活后原 resident 标记未清；停止回执 generation 未校验；
舍入标定的独立 C_B 未按既有重建路径投影；独立 repair latency 起点误设在确认处理时；
最大元数据预算被缩进耗尽。均有回归覆盖。32 块、512 个来源成员的目录现为 324,320 字节，
保留 JSON 字符串/转义与换行，符合 512 KiB 并可由 OpenCV 读取。

验证准备还遇到测试时序/环境问题：首个 checkpoint 用例在新块尚未确认时调用 Finish，
与“拒绝未决候选”的契约冲突，改为等待确认；迟到 commit 用例等待真实 decision 接收而非
固定轮询次数。Humble 的定向构建漏生成运行时动态加载的 FastRTPS/introspection 库，补齐
类型支持后真实入口通过。一次 Noetic 冒烟目录过长导致 AF_UNIX 超过 107 字节，改用新的
短路径。直接 CMake 安装采用显式前缀消费，不假设存在 colcon 工作空间 setup.bash。

## 候选源码官方回放

候选源码先完成 10 秒 OFF/持久 ON，再完成整 bag OFF/持久 ON，并单独进行 C1 短段。
对应 `final-short-*`、`final-full-*`、`final-c1-*` 及比较 JSON。`final-*` 是当时脚本的目录名；
该轮在 repair 计时修正前，最终源码复验使用新的输出路径。

| 对象 | 10 秒配对 | 完整 bag 配对 |
| --- | --- | --- |
| LIO / VIO | 98 / 97 | 1352 / 1351 |
| 四组数组 | 逐值一致 | 逐值一致 |
| 图像/点云发布语义 | 一致 | 一致 |
| 原始关键帧文件 | 16 个 SHA-256 一致 | 517 个 SHA-256 一致 |
| 回环后端 | 5 帧、版本 0、0 接纳 | 172 帧、版本 2、2 接纳 |
| Gaussian 资产 | 1 个，29,824,607 字节 | 13 个，703,846,956 字节 |
| volatile / 无引用资产 / incomplete | 0 / 0 / 0 | 0 / 0 / 0 |

完整输入收到 27,447 条 IMU、1,355 帧 LiDAR、1,355 帧图像，计数无缺失。Gaussian 观察
172 帧，175 次确认、15 次合并、31 次 checkpoint、17 次 load、8 次 rebind、10 次 rebuild、
18 次 prune，无失败或容量命中，pending/in-flight 峰值各 1。8 对保留预览中 5 对训练、
3 对重用；比较器将两者分开计数。节点采样峰值 RSS 为 OFF 826.84 MiB、ON 835.07 MiB；
这是该次采样结果，不是资源保证。

末次回环后 5 块 current、8 块 needs_rebuild。正常 Finish 的完整性指已确认内容和引用已
持久提交，不承诺把全部历史修复队列训练完。8 块保留旧 canonical 及实际窗口，明确禁止
当作当前优化地图；比较器以最终绝对位姿逐块验证该分类。该阶段不能被称为完整优化后的
稠密 Gaussian 地图，也不支持用这些文件直接自动恢复整套 SLAM。

## 独立冷恢复与受控真实观测

停止 worker 后，另起 GPU 进程依次读取 13 个资产并创建新模型：参数及 origin IDs 逐位
一致，渲染有限，2 个仍有对应预览的资产最大 RGB8 差为 0。没有借用原进程的模型实例。

验证工具接着从两个历史资产的真实 RGB/深度/K/曝光/独立 C_B 重新构建新会话，使用人工
控制的目标序列验证模型生命周期；源资产不改写：

- 拒绝新视图和拒绝跨块激活，均恢复此前 resident 的参数；RGB/alpha/z 最大差均为 0。
- 刚性复用 0 步，GPU 参数地址、逐位参数和资产 SHA-256 不变。
- 修改未保留成员 28，并将 pose version 从 1 跳到 4，9 个保留视图重建 180 步。
- 新确认写入、持久引用确认后清理旧资产；目录同步故障发生在 manifest 可见后，旧、新
  资产 1/3/4 均保留且可检查，失败会话禁止继续 prune。
- 32/33 槽位使用**同一个真实观测重复、人工分配 frame/submap IDs**；不称作 32 段真实街景。
  32 个资产落盘，第 33 个请求被拒绝，已有参数和资产保持。

该进程 PyTorch allocated/reserved 峰值为 101,128,704 / 117,440,512 字节，RSS 峰值
1,278,533,632 字节；不含整个系统/驱动的显存上界。报告与实际文件在 `real-controls/`。

在新电脑上，先运行官方 bag 得到至少两个多视图资产，再使用独立环境运行：

```bash
OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1 \
CUDA_VISIBLE_DEVICES=0 PYTHONNOUSERSITE=1 "$LIVO_PYTHON" \
  tools/3dgs_probe/validate_submap_lifecycle.py \
  --map-directory /path/to/finished-gaussian-output \
  --output /path/to/new-separate-validation-output
```

该工具的修正、版本跳变、故障和容量分组是受控输入；真实回放的异步接纳、保存期间修正、
状态发布公平性和队列边界还由 C++ socket 用例与正式 bag 分别覆盖，不互相冒充。

## 最终审阅与复验

独立整范围审阅、修复后的源码身份和最后一次受影响构建/回放结果待本轮收尾填入。
本节未完成前，前面的候选数据只证明对应源码与环境；不将未运行项计为通过。
