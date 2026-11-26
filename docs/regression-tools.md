# 可复现工程回归

按已批准的方向复用现有 CMake/CTest、安装消费者和两套 ROS 回放工具。入口包含正式几何
基准、构建记录与日志分析，不修改 SLAM 算法、参数格式、线程调度或公开 SDK。

## 设计范围

选择普通 Python 脚本顺序调度，避免引入 CI 服务、测试插件框架或自动安装依赖。
原有 run_baseline.py、run_ros2_baseline.py 继续负责 ROS 进程、消息和回放检查；统一入口
记录命令、日志、实际覆盖范围和退出状态。缺少显式请求的环境应报告 unavailable，不能
当作测试通过；没有请求的阶段标记 not_requested。

统一入口 tools/run_regression.py 使用已配置的 --sdk-build、--ros-build 路径，先构建再
执行 CTest。ROS 构建从 CMakeCache 检查版本，并要求当前终端已 source 对应 Noetic 或
Humble。不自动启动 Docker、切换 ROS 环境或猜测宿主路径。--ctest-regex 可以明确选择
子集；报告保留实际测试数。Noetic 的未注册 rostest/transport 程序不属于 CTest 覆盖。

SDK 成功后安装到本次输出下的 stages/consumer/sdk-install，再构建并运行已有 installed_consumer，
从 SDK 的 CMakeCache 继承已配置的编译器及依赖查找目录；不覆盖用户已有安装。

tools/analyze_timing.py 独立分析 GLOG，标准库逐行读取并汇总次数、累计、P50/P95/P99、
最大值、前端两类耗时和缓存计数，不固定任何回放次数。精确分位数会保留数值样本。
父子计时不相加；少量样本不能
代表尾部分布。无记录或非法时长明确失败。

## 正式几何基准

安装的 livo_atlas_benchmark_geometry 接受 --dataset、--pose-version（缺省0）、--rounds，
以及几何阈值与资源参数。通过公共只读接口读取封口且完整索引的数据集，不固定13块、
172帧或版本2。预加载原始点云有可配置的存储上限；不把加载、结果比对、指纹计算纳入
计时。每轮逐块交替公开事务参考与完整构建顺序，不保留整份参考地图或输出大体积载荷。

逐字段检查结果，包括 double 的位表示；同时输出两条路径的 FNV-1a 64位指纹、样本与
工作量。指纹仅用于便携回归提示，不作为密码学哈希或不同平台数值一致性的承诺。
程序只向 stdout 输出 JSON，诊断经 GLOG 到 stderr。实现位于 tools 私有库，公共 SDK 不变。

统一入口可通过 --dataset 请求该基准，--geometry-config 读取现有 config/backend.yaml
中的 loop_closure.geometry，传给基准既有字段；配置缺省仍由 GeometryOptions 定义。
没有 SDK 安装阶段时可显式给 --benchmark-executable。

## 回放与产物

--replay ros1/ros2 调用既有工具，--replay-args 后传入其原有参数；统一入口控制 --output。
若生成回环数据集，使用 --loop-checker 或本次 SDK/ROS 构建中的检查工具确认归档后，
才允许精简输出。ROS1 仍遵守既有回放工具对独立源码快照 workspace 的要求。

--output 必须是不存在的新目录。既有目录、文件、符号链接（含悬空链接）均拒绝；
悬空父链接也拒绝，指向既有目录的父链接可用于创建新输出。路径按分量依次检查，
拒绝 `missing/..` 这种先经过不存在前缀再折返的写法，防止路径归一化掩盖既有链接。
--retain compact 为缺省，只在所有请求阶段成功后，
对本次 replay 中生成的点云/图像保存逐文件 SHA256 清单，再删除这些载荷；元数据、
结果摘要、日志、轨迹和编译/安装产物保留。--retain all 保留全部载荷，失败或未完成
验证时始终保留。成功验证后的精简若被中断，可能已经删除部分载荷，报告会明确标记
清理中断；删除前生成的文件清单仍保留。不会清理输入数据、既有输出、构建目录或跟随
符号链接删除外部文件。
精简后的回放目录不是完整可恢复数据集，需要 --retain all 才能继续作为基准输入。

## 验收

先测非固定大小/版本的基准、坏参数、资源上限和输入只读，再测零用例、CTest/GTest失败、
缺少环境、回放失败和保留策略。最后运行实际 SDK、Noetic、Humble 测试、安装后消费、
真实数据集基准与原生 Noetic 回放。Humble 完整 bag 回放留给后续步骤，不能用单元测试
结果替代。

## 在另一台电脑运行

先按 [构建说明](core-build.md) 准备依赖及 CMake 构建目录。测试构建需要 GTest、
Python 3.8+；Python 调度和日志分析只用标准库，`--geometry-config` 额外需要 PyYAML。
回放所需 ROS、NumPy、bag 消息依赖沿用原回放器。脚本从源码树运行，不需要 pip 安装。
以下命令在 livo-atlas 包根目录执行，各输出目录必须尚不存在。

```bash
cmake -S . -B build-core -DLIVO_ATLAS_ROS_VERSION=0 \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
python3 tools/run_regression.py --sdk-build build-core --jobs 2 \
  --output ../regression/sdk

# 在已 source 的 Noetic 或 Humble 终端传入对应构建目录。
python3 tools/run_regression.py --ros-build /path/to/configured/build \
  --jobs 2 --output ../regression/ros

# 可选子集；报告记录名称、数量和正则，不代表完整测试覆盖。
python3 tools/run_regression.py --ros-build /path/to/configured/build \
  --ctest-regex 'geometry_.*test' --output ../regression/geometry-tests
```

SDK 阶段包含构建、CTest、临时安装和独立消费者。构建会更新给定的构建目录，
不自动重设其编译选项；非标准依赖需在首次 CMake 配置时提供对应 `*_DIR`。
CTest 在构建目录中运行，保留 XML，并额外检查本轮新写入的 GTest XML，避免 catkin
包装命令返回0却遗漏失败。catkin 的结果目录以缓存中的 `CATKIN_TEST_RESULTS_DIR` 为准；
CTest/GTest 失败时同样保存本轮 XML。仅通过消费者后才提供其安装前缀供后续阶段复用。

单独运行真实数据几何基准：

```bash
python3 tools/run_regression.py --dataset /path/to/keyframes \
  --benchmark-executable /path/to/install/bin/livo_atlas_benchmark_geometry \
  --pose-version 2 --rounds 5 --max-preload-mb 512 \
  --geometry-config config/backend.yaml --output ../regression/benchmark
```

版本号必须是数据集中实际保存的版本；不提供时明确使用0，不能理解为最新版本。
`stages/benchmark/geometry.json` 包含有效几何参数、点云预加载字节数、工作量、指纹和每轮耗时。
公开参考路径分别计 `AddKeyframe` 和 `Snapshot`；mapper 总时间包含读取回调、建图、
快照及缓存管理，另列内部阶段。原始点云已经预加载，不能用该结果估算磁盘读取性能。
预加载限制只约束点数组容量，不是进程总内存上限；还需要位姿、mapper 驻留块和当前
一块参考结果与比较缓冲区。先 warmup 一轮，再按轮交替两条路径的运行顺序。

Noetic 回放示例沿用原工具的独立源码快照工作区：

```bash
source /opt/ros/noetic/setup.bash
python3 tools/run_regression.py --replay ros1 \
  --loop-checker /path/to/build/livo_atlas_check_loops \
  --retain compact --output ../regression/replay \
  --replay-args --workspace /path/to/isolated/snapshot \
  --bag /path/to/Retail_Street.bag --duration 136 --rate 1 \
  --loop-closure --semantic-output --stage-timing
```

`--replay-args` 必须放最后，其余参数直接交给旧回放器。Humble 用 `--replay ros2`，
通过其 `--executable`、`--config`、`--camera` 等参数提供当前环境输入。
回放目录由统一入口创建，不能指向既有数据。成功回放的检查工具必须与数据格式匹配。
统一入口拒绝 `--legacy-output`，因为该模式可覆盖源码快照中的历史输出。
`--gaussian-output` 必须位于本轮输出的 `replay/` 内。回环请求或回放摘要声明启用回环时，
缺少归档也会失败并保留产物。Humble 的 stdout 和 GLOG stderr 分开读取，合并统计；
精简同时覆盖两端使用的 `node_output` 和 `node-output` 产品目录。

日志分析也可单独运行，输出文件不可已存在：

```bash
python3 tools/analyze_timing.py --log /path/to/mapping.log \
  --output ../regression/timing.json
```

总报告为 `summary.json`；阶段日志在 `stages/`，回放日志、检查摘要和计时报告在输出根目录。
状态为 passed/failed/incomplete/interrupted，退出码分别为0/1/2/130；阶段含 `requested`
标记，请求后先为 pending，未请求为 not_requested，缺环境为 unavailable，依赖前置失败
为 blocked。SDK/安装失败会阻塞依赖该安装的基准；显式 `--benchmark-executable` 仍独立
运行。中断时，正在执行和尚未执行的请求阶段均为 interrupted，并给出不同原因。
报告记录来源、缓存指纹、
工具指纹和实际测试范围。Git 不可用的源码快照明确标记，不伪造提交号。
保留失败日志再重跑时请选择新输出目录。Ctrl+C 停止后续阶段，验证未完成时不启动精简。

## 构建版本与线程记录

CMake 配置时在包的构建目录生成 `livo_atlas_build_info.txt`，安装到
`share/livo_atlas/livo_atlas_build_info.txt`。记录编译器 ID/版本、目标系统/处理器、ROS
版本、构建类型、Eigen/PCL/OpenCV/GTSAM 版本，以及已解析的 OpenMP 编译能力。
版本来自本次 CMake 依赖解析，使用非标准安装时也不改读系统头文件。catkin_make 的
顶层构建与包构建目录不同时，从缓存的 `livo_atlas_BINARY_DIR` 定位相应记录。

SDK/ROS 阶段保存原始记录和 `build-info.json`；成功时同时写入阶段摘要的 `build_info`。
构建前后各采集一次，失败的重构建也刷新记录并保留原始命令失败原因。刷新后的记录
非法时，保存 invalid 状态和原文，不保留旧版本的 recorded JSON。测试失败时仍保留记录。
旧构建没有记录时明确为 unavailable；
版本为空时为 JSON null，不猜测版本。坏 schema 或非法能力值会失败，并保留原记录。
这些字段描述构建配置，不能替代运行时实际加载库路径的核验。

日志分析和回放阶段分别在 `replay-timing.json`、阶段摘要中保存 `thread_configuration`。
它收集节点声明的前端 OpenMP 预算/可用性和后端线程预算；多个实例的不同配置均保留，
旧日志没有配置时为空列表。例子：

```json
{
  "frontend": [{"openmp_threads": 2, "openmp_available": true}],
  "pose_graph": [{"background_threads": 4}]
}
```

总报告 `thread_environment` 只记录回归进程的 OMP_NUM_THREADS、OMP_THREAD_LIMIT、
OMP_DYNAMIC、OPENBLAS_NUM_THREADS、MKL_NUM_THREADS 中已设置的值。回放器可能为节点
调整环境；这里不推测子进程环境或实际线程团队大小。仅请求 replay 时，没有构建阶段
版本记录；可同时提供 `--ros-build` 获取相应配置记录。

数值回归按 [差异诊断结论](cross-ros-numerics.md) 区分同环境精确对照与跨环境数值记录，
不根据单个 bag 制定通用容差。

## 本轮验证记录（2026-11-26）

| 验证 | 结果 |
| --- | --- |
| SDK Release 静态构建、CTest | 78/78 通过，包含3个 C++ 基准用例和21个 Python 工具用例 |
| 安装后独立消费者 | 配置、编译、执行通过；安装的基准可执行 |
| 原生 Noetic 相关 CTest 子集 | 8/8 通过，报告保留正则和实际测试名 |
| Humble 容器相关 CTest 子集 | 9/9 通过，容器已恢复停止 |
| 真实几何数据版本2 | 172帧、13块、169030体素、57136有效法向；逐字段逐位一致 |
| 几何指纹 | 参考与 mapper 同为 `4dda8d3ca8584c2e`；3轮样本一致 |
| 显式版本0、不同规模版本0/1小数据集 | 通过，不依赖固定块数或最新版本推断 |
| 官方 Retail_Street 完整 Noetic 回放 | 1352次 LIO、1351次 VIO、13次回环查询、2次接受、最终位姿版本2 |
| 归档与产物精简 | 先检查 consistent 和保存750项文件清单，再精简344份载荷，共220691953字节（约210.47 MiB） |

完整回放沿用既有几何构建优化版本的隔离源码与节点快照；本轮未改变生产算法。
精简前载荷的 SHA256、消息语义摘要、最终位姿与参考回放一致。原 bag 和既有关键帧
数据共723个文件的前后 SHA256、大小及文件集合均未变，元数据、轨迹、日志和归档
检查结果仍保留。独立复核发现的问题已补失败测试后修复，包括旧输出路径逃逸、
Humble stderr、缺失回环归档、自定义 catkin XML、失真的基准报告和 Humble 双份产品。

验证资料位于工作区 `validation/regression-tools-20261126/`：`sdk-final/`、
`humble-final/`、`noetic-replay-final/` 保留实际报告；`verification.json` 汇总输入保护
和精简后的核验。另用已有 Humble stderr 日志确认可读取98次 LIO、97次 VIO及15类计时。
以上属于回归工具整理阶段。后续已完成两次 Humble 完整 bag 回放，结果与跨 ROS
数值差异见 [Humble 完整回放验证](humble-full-validation.md)；ARM 实机仍待完成。

## 工程收尾记录（2026-11-26）

输出入口、父链接与阶段状态边界已补复现测试并修正。工具自测43项通过；SDK完整
CTest复跑78/78及安装消费者通过，Noetic相关子集8/8、Humble相关子集9/9通过。
两端3秒回放的输入、输出及回环归档检查通过，线程记录均为前端2、后端4。
本机与 Humble 记录分别为 GNU9.4 / Eigen3.3.7 / PCL1.10 / OpenCV4.2，和
GNU11.4 / Eigen3.4.0 / PCL1.12.1 / OpenCV4.5.4；两端 GTSAM 均为4.2.2。

独立复核另发现并补齐路径别名、嵌套 catkin、失败重构建记录和报告写入中断四项边界。
JSON 使用临时文件和原子替换保存；元数据采集、阶段快照和最终快照被中断时，恢复
可解析的报告，并协调 pending/running 阶段状态。

首轮 SDK 中既有 `TwoContentAttemptsYieldOnceThenTrainAgain` 用例曾出现一次
视图请求数预期2、实际3的失败。保留该失败 XML 和复跑日志；后续调度诊断见下一节。

资料保存在工作区 `validation/regression-hardening-20261126/`。本轮只验收工具边界、
构建记录和短回放日志接入；生产 SLAM 算法未变，没有重复135秒算法回归。

## 2026-11-26：渲染派发竞争与布尔词尾

自然重复原失败用例550次未再次触发失败；不能据此排除调度缺陷。随后用测试访问类在
真实邮箱 mutex 被另一线程持有时触发 owner 派发，分别覆盖渲染前地图同步与渲染命令
入队。修复前两个受控用例均稳定得到相同事件次序：第三次 `view/decision` 先于首次
`joint_render`，并检测到渲染轮次中提前预留的内容任务。

原因是 `DispatchRender()` 将非阻塞入队失败当作可继续选择内容任务，后续训练预留遮住了
渲染边界。现在已具备资格的渲染轮次在短暂邮箱竞争时保留优先级，由下次 `Poll()` 重试；
只有成功提交的地图同步才计入同步尝试。真实同步确认过期时仍按原规则归还内容调度。
没有增加队列、生产等待或工作线程。原随机失败未被自然捕获，此处证明的是可受控复现
同类失败的具体路径，不声称穷尽其他时序问题。

独立复核另发现测试辅助代码可能二次接收邮箱，绕过所要施加竞争的边界。已增加受控
迟到的地图确认，先观察到绕过边界的失败，再让每轮只接收一次，并排除仍有高优先级
地图确认待发的情况；真正的锁竞争仍调用生产 `Poll()`，不以私有调度调用代替被测入队。

`openmp_available` 只接受完整的 `true/false`，允许行尾、空白或逗号作为分隔符。
`trueish`、`falsehood`、`true!` 和 `false=0` 的失败测试先于解析修复执行。
另保留一次 Python 全套测试失败现场：合成 CMake 源码修改与生成文件时间戳完全相同，
未触发重配置。仅测试夹具给合成修改设置明确较新的时间戳，避免依赖墙钟等待。

SDK 完整 CTest 78/78；Noetic 和 Humble 各运行联合渲染用例32项，其中29项通过，
3项真实 GPU worker 用例因未配置 worker 而跳过。Python 工具自测在两端各45项通过。
复核修正后的四个关键调度用例重复100轮，共400次通过；CMake 失败重建夹具单独重复
100次通过。修复前失败、复核发现的测试失败和各轮通过日志分别保留。
本轮只修改 CPU 调度边界和日志解析，没有重复 bag 回放或验证 GPU 训练画质。
记录位于工作区 `validation/gaussian-scheduling-20261126/`。

后续已用真实 worker 跑实此前跳过的三项 GPU 用例，并完成双 ROS 的完整 bag 验证，
见 [GPU 与双 ROS 全链路记录](gpu-slam-validation.md)。该轮发现并修复 Humble 消息固定
数组的 NumPy 标量被接收工具误判为非法相机的问题；失败原始记录、修复前后复现及完整
重跑结果分别保留。
