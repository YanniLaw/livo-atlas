# P6-03c2a 模型资产验证记录

2026-11-17，当前记录保存/释放/恢复一个 Gaussian 工作集的验证，不评价地图质量。
使用和恢复语义见 [Gaussian 工作集说明](gaussian-state.md)。

## 源码、环境和输入

- 最终已验证实现提交为 `8543bf0ea181c384ab8e039f24fb545535cfb80c`。
  源码归档 `source-final-code.tar` SHA-256：
  `fe08e3b2a96c05270054dec1ffb2fead61ee4f4f0d1b1a693394894086d93f8c`。
  独立审阅后的修复已提交，再以这份源码完成真实来源 GPU 复验；后续提交只更新文档。
- 主机为现有 Ubuntu 20.04 / Noetic 环境；Python 3.10.22、NumPy 1.26.4、
  PyTorch 2.3.0+cu121、CUDA 12.1、gsplat 1.5.3，使用已预编译扩展。
  GPU 为 RTX 4060 Laptop，驱动 535.183.01，最终起跑检查空闲/总显存 7,308 / 8,188 MiB。
- 原始数据来自 C1 最终源码完成的官方 Retail_Street 完整回放：
  `validation/P6-03c1-20261116/final-full-off/keyframes/`，共 172 个关键帧。
  将 `dataset.yml` 与全部原始帧文件 517 项列入相对路径摘要清单，准备和运行前后均核验。
  本轮没有重新读取或播放 bag，未将新验证说成新的在线多块运行。
- 来源文件清单 SHA-256：
  `66c5465ee5b5b273f7c3d9bcc4f3b411f16eb2033134ed465cb11cc0028f1b99`。
  新观测 bundle 清单 SHA-256：
  `2c57a52a34f95f99283c18c8587690ad3e8c2bf06507fad01e2b550051bc6eef`。
  选择 ID 0–11，训练 `0,1,2,4,5,6,8,9,10,11`，留出 `3,7`，显式标为 offline probe。

验证产物在包外 `validation/P6-03c2a-20261117/`，迁移电脑时单独复制或重新生成。
准备器来自新的串行 C++ 构建，使用已安装普通 SDK；GPU 进程清除 ROS `PYTHONPATH`，
设置单线程 OpenMP/BLAS 并使用外部进程时限。没有升级驱动、Python 环境或启动 ROS2 容器。

## 测试与故障路径

- 现有相关 Python/GPU 基线 45 项通过；加入 body 来源后 48 项通过；最终相关
  **80 项全部通过**，包括审阅修复的 3 项回归。普通核心现有构建的 **62 个 CTest 目标
  全部通过**，没有重新构建未改变的 ROS 节点。
- body 元数据在 C++→Python 的实际往返中验证；额外使用 Rz90 和非零平移的 body 锚点，
  检查 camera/body 关系与 RGB、深度、内参保持一致。旧 bundle 可读，坏可选姿态拒绝。
- 资产测试覆盖未知字段、dtype/形状、掩码、来源、摘要、截断、压缩/重名 ZIP、巨大 NPY
  头/shape、object dtype、目录覆盖和符号链接。ZIP 中央目录在 ZipInfo 分配前检查；
  CPU 读写和 inspect 在阻止 torch 导入的独立子进程中通过。
- 注入数组写入、文件 close/fsync、清单 rename 失败时，没有最终清单；注入提交后的
  资产/父目录 fsync 失败时，收到 CommitUncertainError，资产仍可读，CLI 非零报告
  `commit_uncertain`。这不等于验证真实掉电或所有文件系统的持久性。
- 最大合法 payload 实验包含 10 个 1,000×1,000 视图及 10,000 个高斯，观察数组未压缩
  payload 为 180,000,000 字节。最终 25 项格式/bundle CPU 测试峰值 RSS 410,720 KiB；
  该值受 Python 和临时缓冲影响，不能把 192 MiB 文件上限称作整个进程内存上限。
- 实际 CUDA 单元测试验证未接纳候选禁止捕获、捕获不借用原数据、独立相机关系、旧前沿
  兼容和身份拒绝、删除旧模型后恢复，以及第 11 个视图的淘汰与 20 步继续训练。
  保存/恢复后的参数逐位一致，RGB、alpha、z 深度最大差均为 0。
- 真实来源下隐藏 CUDA 设备返回 1，错误 `CUDA is required`，没有创建输出目录；
  截断真实资产副本的参数文件后 inspect 返回 1，错误 `Array file size or digest mismatch`。
  原始输入和正式资产未被故障注入修改。

## 真实来源 GPU 往返

先确认 9 个训练视图，保存 9,000 个高斯，再删除原模型。资产记录 12 个 probe 主成员的
局部 body 位姿、9 个保留视图，捕获前沿为 172。资产文件大小分别为：

| 文件 | 字节数 |
| --- | ---: |
| `manifest.json` | 12,373 |
| `gaussians.npz` | 577,526 |
| `observations.npz` | 53,093,290 |

模型释放前 PyTorch allocated 为 54,742,528 字节，删除后为 **0**；新模型恢复后为
53,661,184 字节。这里检查的是原模型张量已释放，不要求销毁 CUDA context 或 allocator
缓存。CPU 资产、读取结果以及恢复后参数/观测逐位一致，RGB/alpha/z 浮点最大差和 RGB8
最大差均为 **0**。

随后用最后一个训练帧 11 继续 20 步，得到 10,000 个高斯，检查到非零有限梯度与参数更新，
累计有效深度监督 239,269 次。Adam 按新 append 重建，没有声称恢复了旧动量或相同训练轨迹。
保留视图顺序为 `0,1,2,4,5,6,8,9,10,11`。

最终 GPU 往返过程记录约 3.79 秒，allocated/reserved 峰值分别为
101,071,360 / 117,440,512 字节，进程峰值 RSS 1,292,300,288 字节。
CPU inspect 约 0.090 秒、峰值 RSS 100,278,272 字节。上述是本机该次试验采样，
不是在线吞吐、跨硬件时限或显存总量保证。

实际查看的 `restored.png` 仍有明显点状空洞；本阶段证明资产恢复一致性，不证明稠密覆盖
或渲染质量改善。保存前后的同视图结果也不能充当独立新视角质量评价。

## 审阅与后续范围

已完成一次独立整范围审阅（设计起点 `b7f2684` 至文档提交 `1b3d587`），没有 Critical。
按使用影响归级后，三项发现均在同一次修复中关闭，每项先观察回归失败再修复通过：

1. 磁盘 uint8 掩码过早转 bool，导致 255/2 被归一为 True 并绕过域检查；现在先验证
   原始值，再生成独立 bool 数组。回归实际改写观察归档并更新摘要，覆盖两种掩码。
2. 输入 ID 的 `list(range(first,last+1))` 在前沿上限检查前分配；现在在小清单中检查
   ID、共同前沿和连续性，再读取图像，连续性只遍历已有条目。回归安全拦截旧 eager
   reader，不实际执行巨大分配，覆盖 10 亿 ID、超限/不一致前沿、断裂和重复成员。
3. `inspect` 缺少计划中的 schema、位姿覆盖、来源与预算摘要。审阅原评 Minor，执行者
   按跨电脑资产核对的实际缺失升为 Important；现在输出这些信息，来源缺省时不捏造摘要。

修复后的 80 项 Python/CUDA 测试、62 项普通核心 CTest、最终源码真实往返和失败路径复验
全部通过。无延期 Minor。C++ 变更按 Google 风格核验；5 份相关 Markdown 的本地链接和
代码围栏检查通过。独立准备工具的动态链接没有 ROS、vikit、CUDA 或 torch 依赖；
本阶段没有修改核心/ROS 的源码、公开头或主构建入口。

在线多块驻留、全局目录、实际闭环修正、worker 自动重连和完整 SLAM session 恢复属于后续
C2b 及其独立验收；C2a 不放宽既有 512 帧前沿与工作集预算。
完整历史主成员仍由调用方保证；资产提交后须保持只读。压缩/带注释的非规范 NPZ 被拒绝，
跨 GPU/软件渲染一致性与稠密/留出画质需要后续独立验证。CLI 携带来源清单摘要，原始文件
核验由外部验收命令完成。这些责任与局限保持原设计，不宣称已由本次往返解决。

## 本机证据位置

以下路径均相对于 `validation/P6-03c2a-20261117/`，不构成产品的绝对路径依赖：

- `roundtrip-final/report.json`、`roundtrip-final/state/`、`roundtrip-final/restored.png`：
  最终模型、冷恢复/继续训练指标和实际预览。
- `roundtrip-final.log`、`inspect-final.log`、`no-gpu-final.log`、`corrupt-final.log`：
  最终源码的成功与非零退出证据。
- `source.sha256`、`source-final-before-check.log`、`source-final-after-check.log`：
  517 个来源文件前后核验，清单及 bundle 摘要见上文。
- `review-mask-*`、`review-frontier-*`、`review-inspect-*`：审阅回归的 RED/GREEN 日志；
  `review-full-green.log`、`core-ctest-final.log`、`cpu-format-final.*`：完整测试与 CPU 用量。
- `source-final-code.tar`、`final-source-commit.txt`：对应实现源码；
  `execution-review-record/`：逐项执行台账、审阅范围和发现处置记录。
