# P6-03c3b：最终快照的有界跨子地图联合渲染

2026-11-18，基于 `cee41c3`。本轮交付设计，产品代码尚未实现。
用户已选择“先做最终快照的联合渲染，再接在线”；本文的两块上限、查询方式和输出格式
是执行者根据现有代码提出的具体选择，尚不代表用户逐项批准。
正式编码前仍按用户要求通知切换模型。设计确认后另写实施计划。

## 目标与当前基础

[C3a 最终验收](../../p6-03c3a-validation.md)已交付同一最终位姿下 current、durable 的
13 块确认内容。现有预览只渲染一个活动模型，不能验证不同局部坐标下的模型能否正确
出现在同一视图、形成跨块遮挡。本阶段先交付一个可重复、只读的局部联合渲染工具，
给未来在线可见工作集和显示接口提供可验证的渲染基础。

以下事实已核对当前源码及保留的最终资产：

- 每块最多 10,000 个 Gaussian，保存 means、wxyz quats、log_scales、color_logits、
  opacity_logits、origin_ids 及至多 10 个完整观测。当前是固定 RGB 的 SH0 表示。
- 目录 binding 是最新放置位姿；asset_source 和资产 capture 位姿是来源基线，可能较旧。
  已通过 canonical primary 检查的刚性重用资产合法，不能强制 capture_version 等于最新版本。
- 最终 172 帧的优化前缀为 169，末尾还有投影位姿，不能只用前 169 帧构造查询相机。
- 实际观测为 640×512；一块的参数 NPZ 约 0.64 MB，十个观测约 59 MB。
  直接用 OnlineGaussianModel.from_state 会把训练观测也传到 GPU，并创建可训练参数。
- rasterizer.py 已封装同一相机下的 RGB、alpha 和期望相机 z 深度，保留 OpenCV 像素中心
  到 gsplat 的 +0.5 转换。渲染器输入支持一次提交多个 Gaussian。

因此复用已有验证与 rasterizer，新增只读推理组合；不借恢复训练模型来承载这项功能。

## 路线选择

| 路线 | 收益 | 代价与结论 |
| --- | --- | --- |
| 最终快照中选择少量块，同次联合光栅化 | 版本稳定，遮挡可测试，计算范围明确 | 只描述选定块；用户已选择先做快照路线，本阶段推荐此实现 |
| 全部块常驻 GPU，直接渲染完整目录 | 单张图输入范围更大 | 引入全图常驻和新的资源预算；暂不采用 |
| 在训练 worker 中立即增加在线联合显示 | 可以边训练边看 | 还需要一致快照、候选/确认隔离、可见集合和调度契约；放在后续在线阶段 |

分别渲染两张块图再叠图不作为实现方案：不同块的 Gaussian 深度可能交错，块级图片和
平均深度不能恢复全部逐 Gaussian 的前后关系。联合路径必须在同一次光栅化中处理选定内容。

## 本阶段范围

- 读取已停止并完成 Finalize 的固定输入；首版只接受状态一致、全部确认内容 current 且
  durable 的目录。允许逻辑 missing 明确保留，但不会把它们装成有效内容。
- 一次请求显式选择 1..2 个已有块，联合 Gaussian 总数最多 20,000，不截断、不降采样。
- 查询相机来自某个已经保留的训练观测，使用其 K、尺寸、camera_from_body 和对应最终位姿。
- 输出联合 RGB/alpha/z、参考图及参考所属单块的诊断基准、来源和资源报告。
- 仅增加现有可选 Python/CUDA 工具和测试；普通 C++、ROS1/ROS2 共用入口及已有资产格式不变。

本阶段不做自动可见性选块、自由相机漫游、全图常驻、跨块去重、重训练、增密、外观修正、
高斯约束反馈定位或 session 恢复。当前选块完整性和画质会如实报告，不承诺完整场景覆盖。
新工具没有接管或恢复在线 session 的权限。

## 输入、只读前提与 CLI

拟新增 `tools/3dgs_probe/render_gaussian_map.py`，最小入口为：

```bash
python tools/3dgs_probe/render_gaussian_map.py \
  --map-directory /path/to/g-final-full-finalize \
  --keyframes /path/to/final-full-finalize/keyframes \
  --keyframe-id 23 \
  --submap-ids 0 1 \
  --output /path/to/new-render-output
```

这里的帧/块 ID 仅说明调用方式，不是实测中已确定的共视正例。

1. CLI ID 使用规范非负 uint64 十进制；keyframe_id 必须小于最终 frame_count。块列表长度
   1..2，重复拒绝，不同输入顺序归一为升序；不靠连续 ID 假设分配任意大的数组。
2. 用现有 inspect_gaussian_map 检查完整目录、原始/优化位姿、分组、逐资产 SHA/数组和
   canonical currentness，再用 inspect_gaussian_finalization 核对 E、前沿、引用及停止事实。
   timed_out、未冻结 E、dirty、历史内容、未引用资产或坏引用均拒绝。
3. 离线工具只能验证当前文件与报告事实，不能由文件存在证明原进程曾成功 fsync 或正常退出。
   C3a 回放验收中的 report_persisted/正常退出证据仍独立保留，不在此重新伪造。
4. 请求的块必须有 current 内容及匹配 current 资产。未选中的 missing 只在覆盖报告中列出；
   选中 missing 则失败。空地图/没有可用内容不能产出一张“成功空图”。
5. 在全部已验证资产的 retained views 中寻找查询帧，必须唯一。查询不必属于选中块；
   若帧未保留则明确失败，不把原始未去畸变图像或另一帧相机偷偷当成查询输入。
6. 输出必须是新目录，父目录已存在，且不得与 map 或 keyframes 相等、互相包含；先完成
   参数、文件和查询验证，再创建输出或初始化 CUDA。输入文件绝不改写，也不更改原预览。

输入要求已经封口且在整个调用期间保持不变。保存验证时的 map/finalization/关键帧目录
身份、使用的最终位姿及资产摘要；重读选中资产时再次核对目录引用的 manifest SHA。
固定已验证位姿对象，不在渲染中重新取另一版位姿。输出报告前复核顶层摘要及输入文件
身份/大小/mtime 的可见变化，发现变化则失败。该检测不提供对并发写入者的事务隔离，
也不支持以一个持续变动的在线目录作为输入。

## 坐标、相机和高斯形状

约定 `T_A_B` 将 B 坐标转换到 A。P 是目录 parent_frame（map 或 odom），S_i 为第 i 块
首个 primary body 的局部坐标，B_k 为查询帧机体，C 为查询相机。

```text
T_C_P  = T_C_Bk(saved observation) * inverse(T_P_Bk(final complete poses))
T_C_Si = T_C_P * T_P_Si(latest map binding)
mu_C   = R_C_Si * mu_Si + t_C_Si
Sigma_C = R_C_Si * Sigma_Si * R_C_Si.transpose()
q_C    = q(R_C_Si) ⊗ q_Si           # Hamilton product, wxyz, normalize
```

在 CPU float64 中先组合相对变换，再把选中参数转到查询相机坐标，上传为 CUDA float32；
联合调用的 view matrix 为单位阵。这样不把巨大的全局原点直接灌入 float32 Gaussian 坐标。
组合变换的旋转沿现有刚体容差验证；以 SVD 投影到最近的 proper SO(3)，中心与 quaternion
统一使用该旋转，避免均值和协方差应用不同的近似。超出既有刚体容差的输入直接拒绝。
这些只作用于临时渲染参数，来源资产和目录位姿保持原样。

scale=exp(log_scales)，opacity=sigmoid(opacity_logits)，RGB=sigmoid(color_logits)。
刚性变换不改变 scale、opacity 或当前 SH0 颜色。不能只平移/旋转中心而遗漏 anisotropic
Gaussian 的方向；也不能在相机和参数两边重复施加同一位姿。新工具只接受当前表示，
未来加入方向相关 SH 时需另行约定外观方向，不能直接沿用 SH0 结论。

相机 K 使用保存观测的 OpenCV 像素约定，进入既有 to_gsplat_camera 时只加一次 +0.5。
不二次去畸变、不自行改变尺寸、不拿全局初始外参替换对应观测的 camera_from_body。
曝光估计仅作为来源记录，不在此增加另一套亮度补偿。深度沿用 alpha-normalized camera z，
不是射线量程或最前表面的硬深度；低 alpha 区域用 mask 标明，不称作有效几何测量。

## 联合光栅化与覆盖语义

- 将选定块按 ID 升序、各块 Gaussian 按保存顺序拼成同一组临时输入，调用一次既有 render。
  中心、旋转、尺度、颜色、不透明度逐项对齐；origin_ids 仅保留来源，不用于物理点去重。
- 单块诊断基准额外渲染查询所属块，使用同样的最终相机。它不替代联合调用，也不因块数
  上限把不在选择集合内的参考块悄悄加入联合输入。基准和联合图顺序运行并释放临时缓冲。
- 不把同源 keyframe ID、相近位置或相似颜色当作同一 Gaussian。当前不同块可能描述同一
  表面，透明度叠加、重影和边界缝隙如实保留，属于后续去重/外观研究。
- 报告列出全部有内容块、missing、选中和省略的块，以及各块实际 retained IDs 和参数数目。
  即使所有输入块都 current，选择两块也仍是局部视图，不宣告整张地图已渲染。

## 资源预算及实现边界

| 项目 | 首版选择 |
| --- | --- |
| 来源目录 | 沿用 512 帧/分组、至多 32 个有内容块及既有资产字节上限 |
| 每次联合请求 | 1..2 块、1..20,000 Gaussian，不静默裁剪 |
| 相机 | 一个已保留观测，width×height ≤ 1,000,000，不批量驻留相机 |
| CPU | 一次最多一个完整 GaussianState，另有至多两块的精简参数及一份查询观测 |
| GPU | 一个联合推理缓冲；不加载训练窗口、Adam、梯度或训练模型 |
| CUDA | 沿用已固定环境、预编译 gsplat、至少 4 GiB free、PyTorch 40% 分配比例 |
| 新验证产物 | 目标 ≤1 GiB；复用 C3a 最终资产，不重新录制/复制完整 bag 数据集 |

先完整 CPU 验证；加载选中块时仍复用 read_state，每次释放完整 state 后再读下一块，
只把实际参数和相对位姿用于推理。查询观测只单独保留所需的一帧和相机数据，不长期保留
其所属块的完整训练窗口；构建联合缓冲时的有界临时张量也计入实测峰值。不新写一个跳过 SHA/NPZ 域检查的“快速读取器”。
现有完整 map 验证会流式检查所有资产，启动耗时单列，不能算作已实现实时渲染。

用 torch.no_grad/inference_mode，所有派生张量均不启用梯度，不创建 OnlineGaussianModel、
优化器、回退副本或新 worker，不修改 mapper 单 GPU/单确认工作集的既有契约。
推理进程在在线训练结束后独立运行。图像和 tile 中间量仍会影响实际显存，固定参数数目
不是显存/时延的硬保证。OOM/设备/扩展错误明确失败，不自动换 CPU、减块或降分辨率。

建议只新增 joint_renderer.py 的 CPU 几何/推理函数和上述 CLI，复用 gaussian_map.py、
gaussian_state.py、rasterizer.py。若为了返回已验证 poses/metadata 需要提取小型内部 helper，
保留原 inspect_gaussian_map/inspect_gaussian_finalization 的返回语义和全部旧检查。
不增加 C++ DTO、ROS 参数、IPC 命令、通用插件框架或 Pimpl。未来在线调用可复用数值函数，
其快照交付/选块/时限和消息接口留待下一阶段设计。

## 输出、指标与失败

成功输出拟为：joint.png、reference.png、single-reference.png、render.npz、report.json。
NPZ 保存 float32 RGB/alpha/depth_m 与 alpha≥0.05 的有效 mask；未覆盖深度保持渲染器的
有限背景值，必须与 mask 一起解释。参考图来自保存的 rectified RGB，不另拷训练窗口。

report.json（独立 schema 1，kind=livo_atlas.gaussian_render，上限 64 KiB）至少包含：
源 session/map_sequence/pose_version/frame_count/optimized_frame_count/parent_frame、
map/finalization/数据集及资产摘要；查询帧/相机/K/尺寸和时间来源；当前绑定与来源基线的
版本；选中/省略/missing 块与实际训练覆盖；各块和总 Gaussian 数；联合和单块的 RGB L1、
有效图像 mask 内 PSNR、alpha 覆盖、真实稀疏深度 mask 内的 z 误差及有效像素数；
读取/验证、变换/传输、首帧和渲染、总墙钟时间，GPU peak allocated/reserved 与进程 RSS。
零有效深度像素时对应误差为 null，不把零样本写成零误差；完美 MSE 的 PSNR 不写 Infinity。

查询来自训练保留帧，报告必须标明 training_view，不能冒称 held-out 或新视角泛化成绩。
同时记录相对于参考单块图的变化像素；数值改变证明输入参与，不代表质量改善。
不承诺联合图的 PSNR 一定提高；重复表面可能降低指标。

先关闭全部图像/NPZ文件，最后写 report.tmp.json 并 rename 为 report.json 作为完成标记。
它是离线实验结果，不新增类似 map_committed 的持久提交/fsync 承诺。已创建输出后的异常
返回非零、只留有界 failure.json 和诊断产物，不发布成功 report；不删除未知输入或旧输出。

## 验证设计与实施切分建议

正式计划按以下职责拆分，并在编码前写可失败的行为测试：

1. CPU 快照/请求验证及相机：复用完整 canonical 检查；拒绝未收敛、坏 SHA/引用、缺块、
   错前沿、超预算/重复 ID、未保留查询帧、输入输出重叠；错误不得创建输出或启动 CUDA。
   验证 capture 旧但刚性 current 的合法资产，以及最终投影尾部查询。
2. CPU 变换：独立手算平移、非交换旋转、wxyz 乘法、anisotropic covariance、共同世界
   刚性变换不变性、大原点下的相对计算，以及只做一次像素中心转换；输入数组字节不变。
3. 真实 CUDA 联合渲染：单块与既有 render 路径在固定容差内一致；两块分别含前后交错
   的 Gaussian，用独立逐 Gaussian alpha/深度解析组合验证遮挡、RGB、alpha、ED。
   调换块列表后归一输入不改变语义；改变前后关系应改变颜色，不以图片可读当正确。
4. CLI/输出：当前真实 gsplat 扩展执行，完整来源、有限输出、错误状态、来源只读和预算
   记录通过；保留全部 C1/C2a/C2b/C3a 既有 Python/工具回归。
5. 实际最终快照：复用 C3a 的 13 块和对应 keyframes，首/中/末至少三组相邻块各选两侧
   可用的 retained 查询帧，验证多块输入、最终版本和非参考块的可见贡献，记录不同程度
   遮挡和边界；必要时在全部 12 组相邻块中寻找有效共视正例。若数据无法给出足够正例，
   明确报告覆盖不足，不通过放宽定义伪造成功。新 GPU 进程、来源 SHA 前后保持。

复用 C3a 资产的实际源位姿/窗口变化要进入报告，不能把合成几何测试称为真实道路质量。
这是 Python 可选工具阶段，未改 C++/ROS 时无需重跑整套 bag 矩阵；若实施扩大到普通核心、
公共头或 ROS 接入，必须同步补对应构建与实际入口验证，不能沿用旧证据冒充新实测。
实施中的 C++ 如确有必要仍遵循 Google C++17、函数定义间空行和适当注释。

## 参考与设计选择归属

现有基础见 [gaussian_state.py](../../../tools/3dgs_probe/gaussian_state.py)、
[gaussian_map.py](../../../tools/3dgs_probe/gaussian_map.py)、
[rasterizer.py](../../../tools/3dgs_probe/rasterizer.py) 和
[既有子地图调研](../../online-3dgs-submap-research.md)。

gsplat 接口已核对本机固定 1.5.3 源码与[官方 v1.5.3 rendering.py](https://github.com/nerfstudio-project/gsplat/blob/v1.5.3/gsplat/rendering.py)
（Git blob 2054fed2e33fdffa8965beef0871a52a2227965c）：wxyz、world-to-camera、直接颜色和
RGB+ED 约定均与现有封装一致。本阶段不升级训练/渲染依赖，也不声称调研了更新版本性能。

用户选择的是“最终快照先行，之后在线”；每次两块、先用保留相机、相机相对坐标、独立
只读推理进程和 ≤1 GiB 验证目标均为执行者选择。代价分别是局部覆盖、没有自由漫游/
held-out 泛化验收、需要处理相对旋转，以及暂时没有在线显示；它们避免把版本和训练调度
重新混进首个联合渲染验证。具体私有 helper/测试文件拆分由构建者决定。

## 可实施性检查

2026-11-18，一次独立只读 builder check 返回“nothing important left”。检查阅读设计及
必要源码，未修改产品、运行构建/GPU/bag，也不代替实施后的代码审阅和实际渲染验收。
检查记录在包外 `validation/P6-03c3b-design-20261118/`。
当前仅选择了推进路线，完整设计仍待确认；尚无产品实现、渲染性能或画质验收结论。
