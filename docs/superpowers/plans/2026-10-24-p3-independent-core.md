# P3 独立核心执行计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 livo-atlas 内替换实际使用的 vikit 功能，构建可安装、无 ROS/vikit 的真实核心，并保持 Noetic 行为。

**Architecture:** Camera 小接口/两模型与两项图像操作；Frontend 直接使用普通库；CMake core/ROS1 分支和共享测试清单；ROS 层保留整体调用计时。

**Tech Stack:** C++17、Eigen、PCL、OpenCV、兼容版 Sophus、可选 OpenMP、CMake 3.16、Noetic、GTest。

**Spec:** `docs/superpowers/specs/2026-10-24-p3-independent-core-design.md`。

## Global Constraints

- 既有 refactor-fast_livo2 分支，基线 752a3be256b4560ce537a09077f2b419b0b584b1；按用户持续授权实施与验证，不合并/推送。
- 仅 livo-atlas；rpg_vikit 固定 6c886c8e5d83997806e00294826d528cea3581dd，不修改。
- Google C++17；不改估计数学、同步、单位、标定方向；既有相机特殊分支先保持，数值例外须单独裁定。
- 新证据 validation/P3-noetic-20261024；P2-03 原始证据/工作区只读核验，不能重跑或改写。
- 构建 nice -n 10/-j1，回放与构建串行，无 RViz；不加 native ISA / fast-math。
- 真正的行为 RED → GREEN，编译/链接错误不算 RED；一次新上下文整阶段评审，Critical/Important 一批修复，Minor 留档。

## Review Focus

1. native vikit 到普通编译的浮点契约、float OpenCV K/D、畸变开关和光轴特殊值不能被无声改变；Task 1/2 golden 与 Task 5 数组/消息/导出逐项对照。
2. 相机缩放不能重复；非法尺寸必须在转换整数/分配前校验；灰度采样须支持 ROI 且四邻域合法；Task 2 手算、坏配置和非连续图像验证。
3. 所谓独立核心不能偷偷借用 ROS/vikit 或构建树；Task 3 在排除两者的环境编译 shared/static，移动安装前缀后单独消费并推进实际 LO/LIO/VIO。
4. Writer 失败应报具体文件且无法再成功 Finish，两个 writer 的 PCD/图像间隔不能共享；Task 4 真实故障和交错输出文件验证。
5. 移走计时不得多写/少写更新，Mapper 高频游标不得跨实例耦合；Task 3 处理结果测试，Task 4 双 Mapper 真实初始化，Task 5 回放计数与 gdb/生命周期检查。

### Task 1: 固定参考和数值样本

**Files:** 包外 `validation/P3-noetic-20261024/verify_reference.py`、`capture_legacy.cc`、参考清单；新增 `tests/camera/data/legacy_camera.csv`、`tests/image/data/legacy_image.csv` 及说明。

**Interfaces:** CSV 使用配置编号、运算类型、输入和输出；17 位十进制保留 double，float 值往返精确；非有限结果明确标记。Golden 由固定版真实 vikit 产生，不用新实现算期望。

- [ ] Step 1: 核验 P2-03 候选源码等于 git archive 基线、运行库/环境/bag/短完整导出摘要；写本阶段只读参考索引。Expected: verified，不修改旧文件。
- [ ] Step 2: 独立编译参考采集器，只链接冻结的旧相机/视觉库；Pinhole/Equidistant 的无畸变、正常畸变、首系数零、阈值及光轴，原图去畸变；连续/ROI 灰度插值及角点评分，保留输入种子/算法/平台/程序/库摘要。Expected: 样本有限性与特殊值均可审计。
- [ ] Step 3: 保存可移植小型 golden 数据与生成说明，现有 13 Python 通过；提交 `test: pin legacy camera and image behavior for P3`；`verify_reference.py` 通过。

### Task 2: 相机和图像实现及接线

**Files:** 新 `include/livo_atlas/camera/{camera,pinhole_camera,equidistant_camera,camera_factory}.h`、`src/camera/*.cc`、`include/livo_atlas/image/image_ops.h`、`src/image/image_ops.cc`；替换旧 vikit factory，更新 Frame/VIO/Frontend/OutputProcessor 和相机/VIO测试。

**Interfaces:** Spec 的 Camera/两模型/CreateCamera、InterpolateGray8/ShiTomasiScore；保留 frame 的过渡旧名称，仅替换相机类型和使用方法。

- [ ] Step 1: 最小可编译空实现与真实测试：Pinhole 手算 (fx=4,cx=8,z=1,x=.125 → u=8.5)，两模型 golden 投影/反投影，float 去畸变图摘要、单次缩放1281×1025/.5→640×512，边界、非法配置。灰度梯度 ROI 插值及评分对 golden、邻域越界拒绝。Expected: 行为断言 RED，保存 XML。
- [ ] Step 2: 按原表达式顺序和类型实现模型/图像操作；不带原库的头/符号，不补未使用接口。Expected: 新测试 GREEN；有浮点差异先定位原生编译算子再裁定，禁止随意松容差。
- [ ] Step 3: 替换全部实际调用，删除无调用的金字塔和未用依赖，Frame/VisualPoint 删除拷贝；调整现有测试使用新模型。Expected: Noetic 全量直接测试通过，包内产品代码无 vikit 引用；提交 `refactor: replace vikit camera and image operations`。

### Task 3: 独立构建、安装与时钟边界

**Files:** 根 CMake、`cmake/{core,ros1,tests,sophus_target}.cmake`、Config 模板、package.xml；IMU/VIO/map 计时与 Mapper 输出标签；`tests/installed_consumer/{CMakeLists.txt,main.cc}` 和普通 Frontend 连续视觉用例。

**Interfaces:** `LIVO_ATLAS_ROS_VERSION=0|1`；`livo_atlas::core` 聚合 frontend/synchronization/preprocess，camera/output/io 独立可链接；Config 找实际依赖并创建旧版 Sophus 兼容目标。Mode 0 不调用 catkin 或查找 vikit。

- [ ] Step 1: 无 ROS/vikit 的 full Frontend 测试/外部消费先编译运行于未拆分入口，保存实际不独立失败；新增纯 C++ 连续 LO/LIO/VIO 与 publisher 缺席用例。Expected: 依赖/安装入口不可用被明确记录；功能样例在原实现可作为特征化（编译失败不计功能 RED）。
- [ ] Step 2: 拆分目标、前缀和依赖，不依赖全局 include/link；给普通数学目标维持原数值编译选项/PIC；package.xml 删除 vikit。把时间/输出表格移出核心，移除仅用于它们的字段，比较器留实现文件，OpenMP 头按实际并行能力条件包含。Expected: 同一普通套件在 Mode 0/1 均通过，VIO/LIO 标签由 ROS 层各打印一次。
- [ ] Step 3: Mode 0 shared/static 构建、无 OpenMP 配置、安装后移动 prefix，再用独立工程 find_package 并运行实际 LO/LIO/VIO；检查 compile_commands/ldd/导出目标无 ROS/vikit/源码构建树路径。Expected: 全部通过且普通头自包含。
- [ ] Step 4: Noetic 构建/普通测试/实际 ROS/启动检查，记录实际计数；提交 `build: expose standalone core and isolate ROS1 integration`。

### Task 4: 评审后续项和实例游标

**Files:** `src/io/result_writer.cc`、`tests/io/result_writer_test.cc`；`LIVMapper.h/.cpp`、真实 ROS 接线测试。

**Interfaces:** ResultWriter 的公开 API/成功文件字节不变；私有 TextFile 记录具体路径和流状态，检查每次写/关闭。Mapper 传播游标为实例成员，原公式不变。

- [ ] Step 1: Linux 真实写失败要求消息含 mat_out.txt；关闭本测试文件描述符后 Finish 要报告具体文件且后续仍失败。Expected: 旧包装只给目录，2 项 RED。
- [ ] Step 2: 用最小私有文本流数据和状态检查补上下文，实际图像/PCD 错误也保留路径；Expected: GREEN，全部写入器用例通过。
- [ ] Step 3: 两个 writer 采用不同目录与 PCD/image 间隔2/3，交错3组LIO/VIO；断言 A 的 PCD时间100.5、图像100.5625，B 的 PCD100.75、图像100.8125，各1份，尾批仍不落盘。Expected: 当前正确实例计数的特征化通过；不冒称既有实现有 bug。
- [ ] Step 4: 实际初始化两个 Mapper，提供相同高频传播状态/IMU/时间，两者应得到相同非零传播；先观察静态游标 RED，改为成员后 GREEN。全套普通/ROS/启动通过；提交 `fix: report output files and isolate propagation cursors`。

### Task 5: 全量回归与交付

**Files:** `docs/p3-validation.md`、README/架构进度，包外验证脚本/清单/原始记录、完整历史 bundle。

**Interfaces:** Task 1 参考只读；当前候选无 rpg_vikit 源码/链接；实际 golden 和新 CMake 消费产物是独立验证输入。

- [ ] Step 1: 所有构建串行完成后，生命周期对照/实际诊断、3秒 gdb、30秒/完整136秒控制回放、默认线程45秒、3秒产物场景；先严格比较四组数组、消息有效字段和57导出产物。Expected: 原门槛通过；任何数值例外有独立原因、界限和裁定。
- [ ] Step 2: 核验原始 XML、实际测试数、公开头、目标/系统运行依赖、安装消费、Noetic参数/输出目录、phase-base diff --check、原始 vikit 干净；生成可审阅证据。
- [ ] Step 3: 一次最有能力模型的新上下文整阶段评审；逐项重评级并裁定所有排除项，Critical/Important 一批 RED→GREEN 修复及受影响全验证，Minor 留档。
- [ ] Step 4: 文档登记结果/限制/跨电脑重建命令；提交，生成完整历史 bundle 并在空仓库校验恢复 HEAD；归档本计划执行记录，保留开发分支。
