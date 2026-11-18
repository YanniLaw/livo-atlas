# P6-03c3b Snapshot Joint Rendering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement task-by-task. Steps use checkbox (`- [ ]`) syntax. 用户已选择当前会话原地直接实施，沿用一次最终独立整范围审阅。

**Goal:** 从完成 Finalize 的只读快照渲染显式选定的至多两块 Gaussian，以正确最终相机和跨块遮挡交付图像、覆盖及资源证据。

**Architecture:** 复用完整 CPU 目录/资产校验，流式捕获一份查询观测和最终 poses；精简参数在相机相对坐标中组合，单次调用既有 gsplat renderer。独立 CLI 发布图像/NPZ 后最后写结果标记，不接训练或 ROS 调度。

**Tech Stack:** Python 3.10、NumPy 1.26.4、PyTorch 2.3.0+cu121、预编译 gsplat 1.5.3、Pillow/PyYAML，现有 CPU 严格读者。

**Spec:** [已确认设计](../specs/2026-11-18-p6-03c3b-snapshot-joint-rendering-design.md)，用户在模型切换通知后回复“好的 继续”。

## Global Constraints

- 基线979b28f；仅 livo-atlas，保留 framework_completion，无新 worktree/merge/push/tag。
- 完整确认内容必须 current、durable、冻结 E、正常 stop；逻辑 missing 单列，选中 missing 拒绝。
- 沿用512帧/分组、32有内容块、既有文件大小/SHA/NPZ字段检查；每次1..2块/至多20,000Gaussian，不裁剪。
- 单个保留训练观测作相机；查询帧可来自选择集合之外；必须包含最终投影尾部，不要求资产capture版本等于binding版本。
- CPU float64相对变换，proper SO(3)投影统一作用均值/协方差；wxyz Hamilton乘积；GPUfloat32、相机单位viewmat、K只+0.5一次。
- 单相机至多1,000,000像素；一个完整CPUstate+至多两块精简参数+一个查询；不把训练窗口/Adam/梯度放GPU。
- 不升级环境；沿用4GiB free/40%分配比例/预编译扩展；错误不自动降块/降分辨率/换CPU。
- 输入封口且不可并发修改；保留验证时poses，检查可见文件变化，不声称事务隔离或离线报告自证原fsync。
- report schema1/kind=livo_atlas.gaussian_render ≤64KiB，最后rename才标记成功；失败非零，无成功report。
- 不改C++/ROS/IPC/资产格式；无Pimpl、通用框架、自动可见性、去重、训练、自由相机或定位反馈。
- 新产物只用 validation/P6-03c3b-20261118，目标≤1GiB；只读复用C3a最终13块，保护所有既有数据/依赖。

## Review Focus

1. 查询来自未选中的第三块：基准不污染联合输入，GPU及完整CPUstate及时释放，报告真实覆盖。
2. 旧capture但刚性current、投影尾部及大世界原点：相机/绑定来自最终版本且只变换一次，形状旋转一致。
3. 验证后来源文件或分组目录发生可见变化：重读不能接受另一个manifest，输出不能宣告混合快照成功。
4. 合法但完全无可见贡献/零深度监督：有限背景、null误差/PSNR、训练视图标签真实，不能用可读PNG冒充遮挡正确。
5. GPU/文件发布中途失败：旧输入及已有输出不受损，不创建成功report或悄悄回退；报告不超过64KiB。

---

## 文件与接口

- 修改 tools/3dgs_probe/gaussian_map.py：内部 `_inspect_gaussian_map(..., query_id=None)` 返回 `(summary, document, poses, query)`，public inspect_gaussian_map 原返回保持。query仅克隆匹配的一份观测及owner ID，不缓存全部state。
- 新 tools/3dgs_probe/joint_renderer.py：CPU快照/相机/几何及lazy CUDA推理；普通dataclass RenderSnapshot，直接数据成员。
- 新 tools/3dgs_probe/render_gaussian_map.py：CLI、指标、独立输出及有界结果发布。
- 新 tests/3dgs_probe/joint_fixtures.py：真实小型map/state/keyframe/报告文件，无GPU假模型。
- 新 tests/3dgs_probe/test_joint_snapshot.py、test_joint_renderer.py、test_render_gaussian_map.py：CPU、真实CUDA、CLI契约。
- docs/online-gaussian.md、docs/README.md、新docs/p6-03c3b-validation.md：使用、来源、实测及边界。

RenderSnapshot成员固定为 directory/keyframes:Path、document/finalization/map_report:dict、poses:list[np.ndarray]、
query:dict（RGB/masks/depth/K/camera_from_body/时间与calibration ID，去除含混的旧camera_from_parent字段）、
query_submap_id:int、selected_ids:tuple[int,...]、camera_from_parent:np.ndarray（最终全局P→C）、
fingerprint:dict（受控路径→stat身份与顶层/pose摘要）。

函数：
`load_snapshot(directory:Path, keyframes:Path, keyframe_id:int, submap_ids:list[int]) -> RenderSnapshot`；
`verify_snapshot(snapshot:RenderSnapshot) -> None`；
`load_block(snapshot:RenderSnapshot, submap_id:int) -> tuple[int, dict[str,np.ndarray], np.ndarray]`（id、拥有参数、最终camera_from_submap）；
`transform_parameters(parameters:dict, camera_from_submap:np.ndarray) -> dict`（拥有raw参数，相机坐标）；
`render_blocks(blocks:list[tuple], K:np.ndarray, width:int, height:int, device) -> dict`（CPU RGB/alpha/depth数组、counts和阶段/GPU耗时）；
`image_metrics(result:dict, query:dict) -> dict`；`run(args) -> dict`；`main(argv=None) -> int`。
GPU设备只在CPU preflight后用既有checked_cuda_device选择；render_blocks不创建OnlineGaussianModel。

## 验证约定

仓库 /home/yanni/livo-atlas_ws/src/livo-atlas；phase=/home/yanni/livo-atlas_ws/validation/P6-03c3b-20261118。
使用C3a run-python.sh中的现有GPU环境/fixture/wirepeer路径，新包装只添加已保留tool-deps的PyYAML前缀，
不安装依赖。run-python.sh可执行完整GPU套件及root工具套件。
整套gate：phase/run-all.sh，顺序 unittest discover -s tests/3dgs_probe -v、root tests/test_*.py。
GPU gate LIVO_REQUIRE_GPU=1，不计skip为成功；独立正式资产实验不与套件/构建并行。

### Task 1: 完整只读快照与查询相机

**Files:** gaussian_map.py、joint_renderer.py；joint_fixtures.py、test_joint_snapshot.py。
**Interfaces:** 产生RenderSnapshot/load_snapshot/verify_snapshot/load_block，供后续精简读者与CLI使用。

- [x] **Step 1: RED tests。** 真实文件fixture创建至少三块各2帧、sealed/current、map+finalization及raw poses；测试选择[1,0]归一(0,1)，query来自2的帧5，固定最终相机；旧capture刚性current和优化前缀+投影尾部合法。拒绝duplicate/3块/bool/负/超uint64/缺块/未保留帧/坏资产SHA/历史或dirty/失败报告/错误session与frontier；验证只保留一个query、来源改动被拒绝，import不需要torch。每个预期姿态独立手算。
- [x] **Step 2: Run RED。** 新suite因joint_renderer/load_snapshot缺失失败；fixture自身用现有inspect_gaussian_map和inspect_gaussian_finalization通过，不能以坏fixture充当RED。
- [x] **Step 3: 实现。** 内部流式读者返回已验证poses并在当前state释放前只克隆查询；旧public返回保持。RenderSnapshot固定文件身份（inode/dev/mode/size/mtime_ns/ctime_ns，目录存在性）及小文件摘要。load_block前后核对身份及map引用manifest SHA、返回参数后释放views；最终相机由对应camera_from_body和完整poses构造。
- [x] **Step 4: GREEN。** 新CPU tests、旧map比较器和全部Python/CUDA+root工具通过，无GPU依赖混入CPU预检。
- [x] **Step 5: Commit/task-done。** `feat: load verified final snapshots for joint rendering`；gate run-all.sh。

### Task 2: 相对几何和真实 CUDA 联合光栅化

**Files:** joint_renderer.py、test_joint_renderer.py。
**Interfaces:** 消费Task1 block/query；产生transform_parameters/render_blocks。

- [x] **Step 1: RED tests。** 独立期望：Rz90平移中心；Rx/Rz非交换旋转、anisotropic covariance的全矩阵；q与−q；全局共同刚性变换及1e6原点取消；坏刚体/数组/计数拒绝；输入字节不变。真实GPU单块vs旧render路径，绝对容差RGB/alpha1e-5、depth1e-4m。两块含深度2/8与4/6的交错Gaussian，以四个单Gaussian的真实alpha和解析front-to-back公式独立算RGB/alpha/ED，不能用被测拼接函数生成期望；改变顺序和前后关系验证遮挡。requires_grad全部false。
- [x] **Step 2: Run RED。** 缺少新增函数的import/API失败；不以CUDA不可用跳过冒充RED。
- [x] **Step 3: 实现。** 验证刚体后SVD投影并统一变换中心与wxyz，复制其它raw参数；推理时激活scale/color/opacity，按blockid规范次序拼接，一次render，单位viewmat。device由调用者选定且必须CUDA0；lazy imports，no_grad，输出立即转CPU、释放GPU临时张量。记录实际CUDA同步耗时与allocator峰值。
- [x] **Step 4: GREEN。** CPU+真实CUDA新suite和全部suite；基准可用旧render输入，但期望不能调用被测几何函数。
- [x] **Step 5: Commit/task-done。** `feat: render bounded Gaussian blocks in one camera`；gate run-all.sh。

### Task 3: CLI、真实指标及失败发布

**Files:** render_gaussian_map.py、test_render_gaussian_map.py。
**Interfaces:** 消费Task1/2；产生命令入口、image_metrics、report schema。

- [x] **Step 1: RED tests。** 真CLI解析非法IDs/路径/输入输出包含/已有sentinel拒绝且不导入torch；GPU真实fixture成功输出5文件，query属于第三块仍联合两块。指标用独立小数组手算RGB L1/MSE/coverage/z，零深度样本和完美MSE为null；输出有限且training_view=true/omitted/missing明确。测试输入SHA不变、验证后source变动、OOM/缺扩展/写文件或最后rename失败不发布成功report；注入只在测试侧外部运行边界，不增加产品hooks。
- [x] **Step 2: Run RED。** 缺CLI/functions导致非预期成功的测试失败，确认不是fixture/模块路径错误。
- [x] **Step 3: 实现。** CPU全量验证先于CUDA/输出，路径新建0700；单块reference先渲染后释放，再按selected渲染，第三块不得加进联合。保存joint/reference/single PNG与render.npz，report含所有设计字段但不复制full primary/所有观测，≤64KiB、无NaN/Infinity；最后独占临时report并rename。失败清理本次未完成report、非零及有界failure.json；不覆盖已有输出。来源验证前后；GPU错误不回退。
- [x] **Step 4: GREEN。** CLI CPU/真实GPU/故障与完整suite通过；文件发布故障用真实FS测试并保留输入，mock仅对外部故障点。
- [x] **Step 5: Commit/task-done。** `feat: expose read-only joint rendering diagnostics`；gate run-all.sh。

### Task 4: 实际资产、整范围审阅及最终记录

**Files:** docs/online-gaussian.md、docs/README.md、docs/p6-03c3b-validation.md、spec/plan状态；验证脚本/产物只在phase。
**Interfaces:** 前三项完整CLI与报告；输入只读C3a g-final-full-finalize与final-full-finalize/keyframes。

- [x] **Step 1: 候选实测。** 固定源码/依赖与源文件SHA；首/中/末三组相邻块各两侧retained查询共至少6例，必要时在12组边界内寻找共视。真实参数20k上限/版本/非参考块贡献、输入SHA前后不变；保存可比较图像、数值与资源，不要求画质必提高。查询含投影尾部和未选中owner的实际正例；来源低覆盖明确失败而非放宽。
- [x] **Step 2: 文档与候选提交。** 记录命令、pins/source、实际选块/coverage/metrics/timings、局限，说明C++/ROS未改；普通核心旧可执行67CTest可补验，但不把旧二进制当新构建。新磁盘≤1GiB，只清本阶段重复产物并保留摘要/最终组。
- [x] **Step 3: 一次独立整范围审阅。** most capable fresh reviewer，给Review Focus/设计/ledger；按影响评级，Important/Critical各先RED后统一一次修复，整套绿；Minor及所有declined-to-judge记录，不追加复审。
- [x] **Step 4: 最终重跑。** 固定修正后源码，重跑至少同6实际正例+第三块owner/尾部+全Python/CUDA/root tools，核对所有来源/输出hash和报告，不能沿用修复前结果。
- [x] **Step 5: Commit/task-done/归档。** `docs: record verified snapshot joint rendering`；gate完整suite+最终报告/身份检查。勾选真实完成状态，保留分支；字节/哈希核对归档本计划ledger/rawreview，仅删除此计划scratch，不碰同级。

## Self-Review

Task1→2/3 的字段/所有权明确；Task2→3使用同一render结果dict；Task3→4固定CLI和report，
四项均可独立验证。ReviewFocus1/2分别由Task1+3/Task2承接，3由1+3，4/5由2+3。
实现只需三个集中Python文件（含一个既有读者），没有新训练模型/框架；资源和图像指标不夸大。
用户已明确原地native执行和自动验证，设计及编码前模型通知后的“好的 继续”沿用，不追加已授权范围的逐任务确认。
