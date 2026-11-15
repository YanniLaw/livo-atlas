# P6-03c1 Online Gaussian Mapper Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans for the previously chosen inline execution. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让可选的独立 gsplat 进程在官方 bag 输入期间，对一个有界子地图增量训练并输出带版本的滚动预览。

**Architecture:** 单一 `LivoSystem` 回调分发到已有消费者和普通 C++ Gaussian 桥接。所有者线程维护最新状态、非阻塞交接；C++ 后台线程准备观测并通过有界 Unix socket 与独立 Python/GPU 进程通信，候选经版本核对和确认后才可用。任一位姿版本变化保守清空模型并从最新观测重建；多子地图持久化留待后续阶段。

**Tech Stack:** C++17、OpenCV/PCL、Linux Unix socket、Python 3.10、NumPy、PyTorch 2.3.0+cu121、gsplat 1.5.3、ROS1 Noetic、ROS2 Humble。

**Spec:** [在线 Gaussian 建图与位姿版本设计](../specs/2026-11-16-p6-03c-online-gaussian-design.md)。

## Global Constraints

- **当前计划尚未执行。** 正式编码前通知用户以便切换模型；只修改 livo-atlas，C++ Google 风格、函数间空行，直接组合，避免 Pimpl 和插件框架。
- 默认关闭；普通核心、公共 ROS 入口和未启用的构建不依赖 Python、CUDA、PyTorch、gsplat 或 vikit。可选桥接仅支持 Linux；Humble 仍使用同一 `src/app/livo_main.cc`。
- 输入来自已有 `OnlineSubmapUpdate`，不得替换既有回环、轨迹和几何消费者；拥有数据的关键帧只读，后台调用 `MakeReconstructionView`，不在回调中做 I/O、去畸变或 GPU。
- `Observe`/`Poll` 运行于系统所有者线程，不等待工作线程、socket、GPU 或磁盘；背景线程的 mailbox 非阻塞交接失败时，所有者保留最新待办而不是丢掉位姿状态。
- 视图最多 1,000,000 像素、100,000 云点；前沿最多 512 帧；每个会话一个在途任务、一个最新待处理关键帧和一个最近已选关键帧。队列合并不得影响原始数据保存或后端。
- 只选主成员；一个当前子地图，封存后保留到下一个子地图或结束。至多 10 个已选视图、每视图 1,000 个种子、总共 10,000 高斯；每新视图 20 步（当前 10、其余按 ID 循环 10；无其余时全用当前）。超过 10 帧逐出最旧视图及其来源高斯，并重置 Adam 动量。
- 任何新位姿版本或子地图切换递增 generation，过期模型及优化器必须清空；同版本只追加前沿不令既有任务失效。`RESET` / `RESET_OK` 独立于新视图；接纳确认前再次检查，历史预览不得称为当前。
- GPU 仅一个，启动时空闲至少 4 GiB，PyTorch 总显存比例 40%；缺失预编译 `gsplat.csrc` 禁止 JIT。观察结果不等于实时性或长距离 SLAM 成功。
- 传输为一个可信本机 Unix stream；帧为 `<u32 header bytes><u64 body bytes><JSON><body>`，均小端，头 ≤64 KiB，body ≤16 MiB。连接/握手 10 s、单次传输 5 s、单任务 30 s、停止等待 3 s，均用单调时间；失败只停 mapper，不阻止前端。
- 新输出目录不得存在或位于源关键帧目录内；滚动保留最近 8 份带元数据预览和最多一份在途临时预览。`status.json` 是可能落后的异步采样，所有者内存中的就绪检查才是当前权威。
- 安装或编译前检查目标分区至少 10 GiB 可用；构建、GPU 测试和 bag 回放串行，编译 `nice -n 10`、`-j1`，`OMP_NUM_THREADS=1`、`OPENBLAS_NUM_THREADS=1`。不改系统驱动或 Docker 配置。

## Execution Paths

以下路径为本机示例，换电脑时按实际工作区、可用 bag、SDK 依赖及 P6-03b 环境替换。
正式执行前新建包外实验目录，不覆盖已有结果：

```bash
set -euo pipefail
export LIVO_REPO=/home/yanni/livo-atlas_ws/src/livo-atlas
export LIVO_BAG=/home/yanni/livo-atlas_ws/Retail_Street.bag
export LIVO_PHASE=/home/yanni/livo-atlas_ws/validation/P6-03c1-20261116
export LIVO_CORE_BUILD="$LIVO_PHASE/core"
export LIVO_NOETIC_WS="$LIVO_PHASE/noetic-ws"
export LIVO_PYTHON=/home/yanni/livo-atlas_ws/validation/P6-03b-20261116/env/bin/python
export LIVO_SOPHUS_DIR=/home/yanni/3rd-party/Sophus/build
export LIVO_GTSAM_DIR=/home/yanni/livo-atlas_ws/validation/P6-02b-20261111/gtsam-install-noetic/lib/cmake/GTSAM
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1
test ! -e "$LIVO_PHASE" && test ! -L "$LIVO_PHASE" || exit 1
mkdir -p "$LIVO_PHASE"
df -h "$LIVO_PHASE"
test "$(df -Pk "$LIVO_PHASE" | awk 'NR==2 {print $4}')" -ge 10485760
nvidia-smi
cd "$LIVO_REPO"
cmake -S . -B "$LIVO_CORE_BUILD" -DCMAKE_BUILD_TYPE=Release \
  -DLIVO_ATLAS_ROS_VERSION=0 -DLIVO_ATLAS_WITH_GTSAM=ON \
  -DLIVO_ATLAS_WITH_LOOP_CLOSURE=ON -DLIVO_ATLAS_WITH_ONLINE_GAUSSIAN=ON \
  -DSophus_DIR="$LIVO_SOPHUS_DIR" -DGTSAM_DIR="$LIVO_GTSAM_DIR" -DBUILD_TESTING=ON
```

此命令只在实际实现了 Task 1 的选项与测试脚手架之后执行。Noetic 源副本、ROS master、
socket 和预览目录只在各自新建的 `$LIVO_PHASE` 子目录中建立；迁移时不要复用其他机器的
构建目录。Python 环境缺失时按 [P6-03b 实验说明](../../p6-03b-gpu-probe.md)重建并先测试。

## File Structure

- `include/livo_atlas/core/online_gaussian_options.h`、`src/core/online_gaussian_options.cc`：普通参数及校验；`LivoConfig` 共用 ROS 参数键。
- `include/livo_atlas/gaussian/online_gaussian_mapper.h`、`src/gaussian/online_gaussian_mapper.cc`：所有者线程状态、有限 mailbox、worker 生命周期、预览和统计；无 Pimpl。
- `src/gaussian/wire_protocol.h/.cc`、`src/gaussian/unix_socket.h/.cc`：私有固定消息、受限长度和非阻塞 Linux 传输；不作为通用 IPC 框架。
- `tools/3dgs_probe/online_protocol.py`、`online_model.py`、`online_worker.py`：相同协议、固定规模增量模型、单客户端进程；复用已有 `rasterizer.py` 和训练计算。
- `tests/gaussian/` 与 `tests/3dgs_probe/`：协议、真 socket 假 worker、并发边界、CUDA 合成测试；`tests/ros1/node_options_test.cc`、`tests/ros2/node_options_test.cc` 覆盖两端共用配置。
- `CMakeLists.txt`、`cmake/online_gaussian.cmake`、`cmake/node.cmake`、`cmake/tests.cmake`：默认关闭的桥接目标、可选安装与测试；`src/app/livo_main.cc` 接入现有分发/轮询/关闭顺序。
- `tools/run_baseline.py` 与其测试：新增显式成对的 socket/输出参数用于 Noetic 验证；文档新增 `docs/online-gaussian.md`、`docs/p6-03c1-validation.md`，更新 `docs/README.md`。

## Review Focus

- **帧合法但尺寸、长度或掩码不一致：** Task 1 用跨语言 8×6 真实 socket 往返及截断、溢出和深度掩码错误测试拒绝。
- **位姿在接纳和确认之间变化，且再无新帧：** Task 3 测试不发布旧候选、独立 RESET 到达并清空原地模型及 Adam。
- **单次通知先追加后封存，或旧封存与新建同时到达：** Task 3 测试最后封存块仍是当前块，直到新主成员出现。
- **worker 卡住、半包、关闭或没有 GPU：** Task 1/3/4 用真实 socket 及隔离进程验证超时、SIGPIPE、非阻塞失败和前端继续；GPU 测试不能把 skip 算通过。
- **滚动输出与异步状态发生竞争：** Task 3 阻塞文件写入并注入位姿更新，验证内存预览立即失效、状态最终追上、最近 8 份上限与未覆盖已有目录。

## Shared Interfaces and Wire Contract

所有 JSON 消息含 `schema: 1`、`type` 和 32 个小写十六进制字符的 `session_id`（128 位随机值）。
以下 ID、版本、修订和前沿字段均为无前导零的 uint64 十进制字符串；父系是 `odom` 或 `map`。
`reset` 和 `reset_ok` 携带 `generation`、`pose_version`；有子地图时携带 `submap_id`，
尚无子地图时省略该字段。OpenCV 4.2 的 JSON 解析器不支持 null，故不使用 null。
消息中的非 ASCII 文本直接采用 UTF-8；OpenCV 4.2 不支持 JSON `\u` 转义，两端均拒绝
实际的 `\u` 转义序列，不能因此编码的控制字符也不能用于消息文本。
`view`、`candidate`、`decision`、`commit_ok`、`discard_ok` 携带 session、`job_id`、
`generation`、`submap_id`、`pose_version`；任务/候选还携带 `membership_revision`、
`frame_count`、`optimized_frame_count`。这些字段每次核对，不接受错误类型、重复任务或倒退。

`view` 还携带 `keyframe_id`、`calibration_id`、三个有限时间、有限逆曝光估计、正整数
`width/height`、有限 `fx/fy/cx/cy`、行优先 16 元 `camera_from_submap` 和
`parent_from_submap`。body 为 BGR、图像掩码、深度掩码、小端 float32 深度的逐行数据。
`candidate` 带所用关键帧 ID 数组、20 步、有限 RGB/深度指标、深度监督数量、有限高斯数量及
`width/height`，body 为对应图像大小的 RGB uint8 预览；有状态接收端核对其与任务的尺寸。
`decision` 的 `action` 为 `accept` 或 `discard`；
每个 accept 产生 `commit_ok`，每个 discard 清空模型/Adam 并产生 `discard_ok`。
`hello` / `hello_ok` 确认协议及会话；成功握手记录软件、设备和起跑可用显存。
`hello_ok` 的固定字段为字符串 `python`、`torch`、`gsplat`、`cuda`、`device`，以及
十进制字符串 `gpu_free_bytes`、`gpu_total_bytes`。
候选的固定指标字段为 `frame_ids`、`steps`、`rgb_l1`、`depth_l1_m`、
`supervised_depth_pixels`、`gaussian_count`；计数及步数用非负 JSON 整数。
握手后无法使用 GPU 或预编译扩展时 `error` 返回 `gpu_unavailable` 或 `extension_unavailable`；
其他协议/工作错误分别为 `protocol_error` / `worker_error`，无 body，随后关闭连接。
无法完成握手时记录 EOF/启动错误。`stop` 终止会话。没有自动重连；坏长度、截断、
不匹配身份和错误顺序必须失败。`reset_ok` 确认模型、优化器和观测窗口均清空。

供各任务复用的接口：

```cpp
// src/gaussian/wire_protocol.h, private to the optional module.
struct GaussianMessage {
  std::string header_json;
  std::vector<std::uint8_t> body;
};
std::vector<std::uint8_t> EncodeGaussianMessage(const GaussianMessage& message);
GaussianMessage ReadGaussianMessage(int fd, std::chrono::milliseconds timeout);
void WriteGaussianMessage(int fd, const GaussianMessage& message,
                          std::chrono::milliseconds timeout);

// Public optional bridge; Observe and Poll are called only by the system owner.
class OnlineGaussianMapper {
 public:
  OnlineGaussianMapper(LivoSystem& system, CameraConfig camera,
                       OnlineGaussianOptions options);
  ~OnlineGaussianMapper();
  void Observe(const OnlineSubmapUpdate& update) noexcept;
  void Poll() noexcept;
  void Finish() noexcept;
  std::shared_ptr<const GaussianPreview> GetLatestPreview() const;
  OnlineGaussianStats GetStats() const;
};
```

`GaussianPreview` 的身份和 RGB 像素自有且只读，`GetLatestPreview()` 在当前 generation
无确认时返回空；外部长期持有预览不计入 mapper 内部上限。`OnlineGaussianOptions` 有
`enabled=false`、socket 路径、全新输出目录及四个上述超时；固定帧、视图和 GPU 预算不额外
变成配置项。`camera` 由调用方从同一系统配置的 `frontend.calibration.camera` 显式传入；
c1 只支持当前数据集的 calibration ID 0，不能从私有 `LivoSystem` 状态推断标定。
`OnlineGaussianStats` 至少含设计所列计数、峰值、最新版本/代数与失败信息。

```python
# tools/3dgs_probe/online_protocol.py
def encode_message(header: dict, body: bytes) -> bytes: ...
def read_message(sock, timeout: float) -> tuple[dict, bytes]: ...
def write_message(sock, header: dict, body: bytes, timeout: float) -> None: ...
def decode_view(header: dict, body: bytes) -> dict: ...

# tools/3dgs_probe/online_model.py
class OnlineGaussianModel:
    def reset(self, generation: int) -> None: ...
    def train_candidate(self, view: dict) -> tuple[np.ndarray, dict]: ...
    def decide(self, accept: bool) -> None: ...
```

模型同一时刻至多一个候选；`train_candidate` 原地更新，`decide(False)` 必须清除整块模型、
窗口和优化器；`decide(True)` 保留结果。重置之后要先收到与所有者最新 generation 匹配的
`reset_ok` 才能训练。模型的 20 步轮询及采样复用 P6-03b 的数学约定，不改变该离线 CLI。
`decode_view` 返回拥有存储的 RGB float32、两个 bool 掩码、米制 float32 深度、OpenCV K、
`camera_from_parent`（这里即 `camera_from_submap`）、关键帧 ID 和 `split="train"`，与
已有 P6-03b 初始化/损失函数对接；原始位姿、时间与身份从头保留。

---

### Task 1: 受限跨语言消息和 Linux 传输

**Files:** 创建 `src/gaussian/wire_protocol.h/.cc`、`src/gaussian/unix_socket.h/.cc`、
`tools/3dgs_probe/online_protocol.py`、`tests/gaussian/wire_protocol_test.cc`、
`tests/gaussian/wire_peer.cc`（用于真实跨语言 socket 往返的测试程序）、
`tests/3dgs_probe/test_online_protocol.py`、`cmake/online_gaussian.cmake`；修改 `CMakeLists.txt`、
`cmake/tests.cmake`。

**Interfaces:** 产生 Shared Interfaces 的消息读写函数及可选 `livo_atlas::online_gaussian`；
Python 和 C++ 使用相同 JSON 字段与长度，后续任务不得另造另一套格式。

- [x] **Step 1: 写 RED 测试。** 用 socketpair 和 8×6 样本验证 C++→Python、Python→C++ 的
  字节/身份往返：BGR `(10,20,30)`、深度 4 m、小端、非单位相机变换、掩码 `0/1` 和大于
  2³¹ 的十进制 ID。分别注入半包、EOF、头/body 超长、乘法溢出、非法掩码、非有限深度、
  非法 session 字符串和对端关闭；断言拒绝且不会触发 SIGPIPE 终止进程。错误会话和
  重复任务跨消息顺序由 Task 3 的有状态调度器验证，帧读取器不持有会话历史。
  同时增加 cmake 测试目标，保留实现缺失。
- [x] **Step 2: 运行 RED。** 以 `-DLIVO_ATLAS_ROS_VERSION=0 -DLIVO_ATLAS_WITH_ONLINE_GAUSSIAN=ON`
  配置新 `$LIVO_CORE_BUILD`，单任务构建 `gaussian_wire_protocol_test`；以 `$LIVO_PYTHON`
  运行 `PYTHONPATH="$LIVO_REPO/tools/3dgs_probe" "$LIVO_PYTHON" -m unittest discover -s tests/3dgs_probe -p test_online_protocol.py -v`。
  Expected: C++ 和 Python 因接口缺失失败，已有 SDK 依赖可正常配置，不能把缺依赖当 RED。
- [x] **Step 3: 实现。** 完成固定协议与非阻塞读写，使用已有 OpenCV 内存 JSON 支持，严格
  检查字段、尺寸和数值；所有读取前先核验长度。`send` 防止 SIGPIPE；时限用单调时间，
  停止时通过关闭/唤醒 socket 结束等待。Linux 外启用选项时报明确配置错误，关闭不受影响。
- [x] **Step 4: 运行 GREEN。** 构建目标，运行
  `cmake -E chdir "$LIVO_CORE_BUILD" ctest -R '^gaussian_wire_protocol_test$' --output-on-failure -j1`
  及 Step 2 的 Python 测试。Expected: 两语言实 socket 测试非零且全部通过；头文件和 SDK
  目标不引入 CUDA/Python，构建选项 OFF 的原核心仍可配置。
- [x] **Step 5: 提交。** `git add` 本任务文件，`git diff --cached --check`，提交
  `feat: add bounded Gaussian observation transport`。

### Task 2: 增量高斯工作集与真实 GPU

**Files:** 创建 `tools/3dgs_probe/online_model.py`、`tests/3dgs_probe/test_online_model.py`；
仅在确有共用需要时修改 `tools/3dgs_probe/train.py` 与其已有测试。

**Interfaces:** 消费 Task 1 解码后的 `view`；提供 Shared Interfaces 的 `reset`、
`train_candidate`、`decide`，返回 RGB uint8 预览和可序列化有限指标。

- [ ] **Step 1: 写 RED 测试。** 两帧非单位 SE(3) 合成数据检查 `T_camera_S`、BGR→RGB、
  深度掩码和 20 步调度；先训练 11 个有深度视图，断言最多保留最近 10 个及其来源高斯，
  每帧 ≤1,000、总数 ≤10,000，添加/逐出后 Adam 动量重置。断言无深度帧不插入，纯黑白
  参数和损失有限；拒绝候选后所有模型、窗口和优化器为空，再以新 generation 从头训练。
  真 CUDA 合成测试要求前向、有限非零梯度和参数更新；设置 `LIVO_REQUIRE_GPU=1` 后缺
  GPU 必须失败，不能作为跳过通过。
- [ ] **Step 2: 运行 RED。** `PYTHONPATH="$LIVO_REPO/tools/3dgs_probe" "$LIVO_PYTHON" -m unittest discover -s tests/3dgs_probe -p test_online_model.py -v`。
  Expected: 因 `online_model` 尚未实现而失败，torch/gsplat 已可导入。
- [ ] **Step 3: 实现。** 复用 P6-03b 的反投影、gsplat 主点转换、有限性和掩码损失；
  建立固定数量和来源 ID 的活动高斯、20 步顺序、淘汰及重置，不引入 densification。
  运行前核验空闲显存、设置 40% 分配比例，并调用已有预编译扩展检查，拒绝隐式 JIT。
- [ ] **Step 4: 运行 GREEN。** 执行 Step 2、已有 `test_train.py` 与
  `LIVO_REQUIRE_GPU=1` 的 `test_rasterizer.py`；Expected: 所有 CPU 和实际 CUDA 项通过，
  记录非零深度监督、实际步数及 PyTorch 峰值，不把同步 GPU 耗时当在线吞吐。
- [ ] **Step 5: 提交。** 提交本任务源码与测试，示例 `feat: add bounded incremental Gaussian model`。

### Task 3: 在线调度、过期提交和滚动预览

**Files:** 创建 `include/livo_atlas/core/online_gaussian_options.h`、
`src/core/online_gaussian_options.cc`、`include/livo_atlas/gaussian/online_gaussian_mapper.h`、
`src/gaussian/online_gaussian_mapper.cc`、`tools/3dgs_probe/online_worker.py`、
`tests/gaussian/online_gaussian_mapper_test.cc`、`tests/3dgs_probe/test_online_worker.py`；
修改 `cmake/core.cmake` 和可选 CMake 文件，为默认核心编译不含 socket/GPU 的选项校验。

**Interfaces:** 消费 Task 1 的消息接口和 Task 2 的模型；实现 Shared Interfaces 的
`OnlineGaussianMapper`。worker 由用户先以 `online_worker.py --socket PATH` 显式启动。

- [ ] **Step 1: 写 RED 测试。** 用实际本机 socket 的受控假 worker 和拥有数据的关键帧：
  A 在途时依次到达 B、C，断言 B 被合并、C 保留，单槽峰值为 1；阻塞 worker 期间
  `Observe`/`Poll` 在对端释放前返回。测试相同版本新前沿保留 A 的接纳资格并标记实际
  覆盖；一次事件中追加后封存、旧封存与新建均按主成员切分，最后封存块直到切换仍有效。
  在候选产生前连续注入两个 pose 版本，及 accept 后/confirm 前再注入版本；均不得输出旧
  预览，必须发 `reset` 并收到最新 `reset_ok`，假 worker 中被污染的模型和 Adam 清空。
  在不再有新关键帧时重复后一场景。堵住 status 写入后连续更新，再解除堵塞，断言内存
  查询立即空、最终 status 最新；连续接纳 10 次只留最近 8 份预览，已有输出目录不变。
  另测 EOF、错误 session、重复任务、坏 ID、超时、后台异常和关闭，前端与其他观察者继续。
- [ ] **Step 2: 运行 RED。** 构建 `online_gaussian_mapper_test`，运行
  `ctest --test-dir "$LIVO_CORE_BUILD" -R '^online_gaussian_mapper_test$' --output-on-failure -j1`，
  并运行 `$LIVO_PYTHON -m unittest discover -s tests/3dgs_probe -p test_online_worker.py -v`。
  Expected: 因 mapper 或 worker 接口缺失而失败，不访问真实 bag。
- [ ] **Step 3: 实现。** 保持 `Observe`/`Poll`/`Finish` 的所有者线程模型；后台最多一个
  in-flight、一个 pending 和一个最近快照，非阻塞尝试交换 mailbox，交接失败保留最新状态。
  使用实际绝对 pose/generation 处理跳过版本；区分 `candidate → decision → commit_ok` 和
  独立 `reset → reset_ok`。变更发生于任何确认边界时禁止对外发布旧结果，清除 worker
  原地模型；写滚动 PNG、同名原子 JSON 和可能滞后的状态采样。所有故障记录在 mapper 内。
- [ ] **Step 4: 运行 GREEN。** 运行 Step 2 和 Task 1/2 全部测试；启动真实 Python GPU
  worker，对两帧合成数据经 C++ 桥接实际完成至少一次 20 步确认、位姿重置和新确认。
  Expected: 不仅协议互通，还观察非零有限梯度/深度和可读 RGB 预览，旧模型及优化器未复用；
  控制器、worker 和文件输出的资源与超时记录齐全。
- [ ] **Step 5: 提交。** 提交本任务源码、测试；示例 `feat: stream versioned online Gaussian views`。

### Task 4: Noetic/Humble 共用入口与配置

**Files:** 修改 `include/livo_atlas/core/livo_config.h`、
`src/core/livo_config.cc`、`src/app/livo_main.cc`、`cmake/core.cmake`、`cmake/node.cmake`、
`cmake/tests.cmake`、两端 `node_options_test.cc`、`tools/run_baseline.py` 及其参数测试。

**Interfaces:** 配置键为 `gaussian.enabled`、`gaussian.socket_path`、
`gaussian.output_directory`（ROS1 为斜线形式）；默认关闭。开启要求已开启 keyframes 与
online submaps、可选目标已构建、路径和超时合法。Task 3 的 `Observe/Poll/Finish` 直接接入。

- [ ] **Step 1: 写 RED 测试。** Noetic/Humble 配置默认关闭，禁用时忽略 socket/输出值；
  开启但 keyframes/submaps/online、路径或编译选项缺失应在创建输出前拒绝。真实统一 main
  连接假 worker，并同时启用回环、轨迹、几何三个消费者，断言没有覆盖回调；暂停 `/clock`
  时仍完成轮询。验证后端 `Finish()` 期间的最终 pose 仍传给 mapper，然后才结束 mapper。
  baseline 工具只接受成对 socket/输出参数，缺一或不合法时不创建输出。
- [ ] **Step 2: 运行 RED。** 分别运行 Noetic 和 Humble 的节点选项测试、统一入口假 worker
  集成测试，以及 baseline 参数测试。Expected: 新配置和接入尚不存在而失败。
- [ ] **Step 3: 实现。** 公共参数一次解析，两端保持已有 Node 适配；主入口在既有回调、
  `after_process` 和 Noetic wall-rate 条件中加入 mapper。后端关闭、最后 pose 通知和 mapper
  关闭顺序与设计一致，失败只记录并退化 mapper。默认关闭时不加载可选目标或启动 worker。
- [ ] **Step 4: 运行 GREEN。** 普通 SDK 的桥接 ON/OFF 各编译测试和安装消费；Noetic 与
  Humble 各编译并运行受影响测试。Expected: 两端共用入口和假 worker 行为通过，OFF 的
  默认 SDK/节点 `ldd` 无 Python/CUDA/torch/vikit，保留原有参数与输出。
- [ ] **Step 5: 提交。** 提交配置、接入、回放参数和测试；示例
  `feat: connect optional Gaussian mapper to shared ROS entry`。

### Task 5: 官方 bag 在线验收与跨电脑说明

**Files:** 创建 `docs/online-gaussian.md`、`docs/p6-03c1-validation.md`；修改 `docs/README.md`。

**Interfaces:** 使用 Task 4 同一入口和 Task 3 `online_worker.py --socket PATH`；
`run_baseline.py` 的 `--gaussian-socket PATH --gaussian-output NEW_DIR` 同时启用 bridge。

- [ ] **Step 1: 建立可复验环境。** 使用全新包外 `$LIVO_PHASE`，确认磁盘 ≥10 GiB，
  SHA-256 记录 bag、源码提交和环境；从当前提交导出独立 Noetic 源副本，不能把节点运行
  产物写入开发源码。复用 P6-03b 的 Python 环境及其锁定文件；清除继承的 ROS `PYTHONPATH`。
  Expected: 空闲 GPU ≥4 GiB，扩展可加载，源副本与提交一致，输出目录及 socket 均不存在。

  ```bash
  mkdir -p "$LIVO_NOETIC_WS/src/livo-atlas"
  git archive HEAD | tar -xf - -C "$LIVO_NOETIC_WS/src/livo-atlas"
  ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake "$LIVO_NOETIC_WS/src/CMakeLists.txt"
  sha256sum "$LIVO_BAG" > "$LIVO_PHASE/bag.sha256"
  git rev-parse HEAD > "$LIVO_PHASE/source-commit.txt"
  df -h "$LIVO_PHASE"
  test "$(df -Pk "$LIVO_PHASE" | awk 'NR==2 {print $4}')" -ge 10485760
  source /opt/ros/noetic/setup.bash
  nice -n 10 catkin_make -C "$LIVO_NOETIC_WS" -j1 \
    -DLIVO_ATLAS_ROS_VERSION=1 -DLIVO_ATLAS_WITH_GTSAM=ON \
    -DLIVO_ATLAS_WITH_LOOP_CLOSURE=ON -DLIVO_ATLAS_WITH_ONLINE_GAUSSIAN=ON \
    -DSophus_DIR="$LIVO_SOPHUS_DIR" -DGTSAM_DIR="$LIVO_GTSAM_DIR"
  ```
- [ ] **Step 2: 短段成对回放。** 按 [core-build.md](../../core-build.md) 单任务构建可选
  Noetic 目标；使用 `tools/run_baseline.py --workspace "$LIVO_NOETIC_WS" --bag "$LIVO_BAG"`
  配合 `--submap-mode online --loop-closure --image-queue-size 32 --rate 1 --duration 10`
  各跑一次关闭和开启。开启时预先启动独立 worker 并提供上述两个 Gaussian 参数；输出、
  ROS master 端口和日志各用新路径。Expected: 两次正常退出，输入计数、前端数组和已保存
  的原始观测逐项一致，mapper 在 bag 输入未结束前完成一次真实 GPU 训练/确认。

  ```bash
  export OMP_THREAD_LIMIT=1
  python3 tools/run_baseline.py --workspace "$LIVO_NOETIC_WS" --bag "$LIVO_BAG" \
    --output "$LIVO_PHASE/short-off" --port 11341 --cpu-affinity 0 \
    --submap-mode online --loop-closure --image-queue-size 32 --rate 1 --duration 10
  mkdir -m 700 "$LIVO_PHASE/short-worker"
  env -u PYTHONPATH CUDA_VISIBLE_DEVICES=0 PYTHONNOUSERSITE=1 \
    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 nice -n 10 \
    "$LIVO_PYTHON" tools/3dgs_probe/online_worker.py \
    --socket "$LIVO_PHASE/short-worker/mapper.sock" > "$LIVO_PHASE/short-worker.log" 2>&1 &
  LIVO_WORKER_PID=$!
  trap 'kill "$LIVO_WORKER_PID" 2>/dev/null || true' EXIT
  LIVO_WORKER_READY=0
  for ((attempt=0; attempt<100; ++attempt)); do
    if test -S "$LIVO_PHASE/short-worker/mapper.sock"; then
      LIVO_WORKER_READY=1
      break
    fi
    kill -0 "$LIVO_WORKER_PID"
    sleep 0.1
  done
  test "$LIVO_WORKER_READY" = 1
  python3 tools/run_baseline.py --workspace "$LIVO_NOETIC_WS" --bag "$LIVO_BAG" \
    --output "$LIVO_PHASE/short-on" --port 11342 --cpu-affinity 0 \
    --submap-mode online --loop-closure --image-queue-size 32 --rate 1 --duration 10 \
    --gaussian-socket "$LIVO_PHASE/short-worker/mapper.sock" \
    --gaussian-output "$LIVO_PHASE/short-previews"
  wait "$LIVO_WORKER_PID"
  trap - EXIT
  ```

  `--cpu-affinity 0` 为本机示例；其他机器应先确认允许的 CPU 并在成对运行中保持一致。
  `--gaussian-*` 是 Task 4 需要实现并测试的参数，当前版本尚无此参数。
- [ ] **Step 3: 完整成对回放。** 使用相同条件各跑一次 `--duration 136 --rate 1`，完成后
  比较 `trajectory.tum`、`mat_pre.txt`、`mat_out.txt`、`imu.txt` 的数组形状、时间和数值；
  比对两次原始关键帧文件哈希与既有回环/输出统计。记录所有已选/提交/合并/过期/重置、
  在线结果时间、端到端延迟分布、前端耗时、CPU RSS、GPU allocated/reserved 与系统
  使用量。Expected: 原前端和观测严格一致；至少一个真实训练结果在输入期间被接纳，
  不丢报状态、没有越过上限。出现差异或资源超限须排查并原样记录，不能调门限掩盖。
- [ ] **Step 4: 失败与平台边界。** 用无 GPU、缺预编译扩展、worker 崩溃和连接卡住的独立
  实验检查 mapper 明确失败且前端继续；用确定性人工位姿更新确认丢弃与 RESET。Humble
  至少运行共用入口和真 socket 假 worker；若容器没有 GPU，只报告未覆盖真实 GPU，不配置
  容器运行时。重新运行 `sha256sum --check "$LIVO_PHASE/bag.sha256"` 并核对所选原始数据哈希。
- [ ] **Step 5: 文档、回归和审阅。** 写跨电脑版本、构建、启动 worker/节点、预览与状态
  时效、限制、停止顺序和可重复命令；真实数值与失败证据写验证报告。运行完整 CPU CTest、
  全部相关 Python 和 GPU 合成测试，`git diff --check`、头文件及动态依赖检查。提交后按
  `executing-plans` 对整个 c1 范围独立审阅一次，修复重要发现并记录复验；不宣称在线画质、
  真实长距离闭环或多子地图持久化已验证。

## Self-review

Task 1 固定有界协议；Task 2 固定 GPU 数学及丢弃语义；Task 3 将两者接入有限的单线程所有者
状态，明确 idle reset、confirm 竞态和封存；Task 4 在同一 ROS 入口复用；Task 5 验证真实
时间与前端兼容。多子地图存储、精细修正、自动恢复和高斯反馈均不属于 c1。
