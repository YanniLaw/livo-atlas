# 可复现工程回归

按已批准的方向复用现有 CMake/CTest、安装消费者和两套 ROS 回放工具。本轮只增加验证
入口与正式几何基准，不修改 SLAM 算法、参数格式、线程或公开 SDK。

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

--output 必须是不存在的新目录。--retain compact 为缺省，只在所有请求阶段成功后，
对本次 replay 中生成的点云/图像保存逐文件 SHA256 清单，再删除这些载荷；元数据、
结果摘要、日志、轨迹和编译/安装产物保留。--retain all 保留全部载荷，失败或未完成
验证时始终保留。不会清理输入数据、既有输出、构建目录或跟随符号链接删除外部文件。
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
状态为 passed/failed/incomplete/interrupted，退出码分别为0/1/2/130；未请求阶段为
not_requested，缺环境为 unavailable，依赖前置失败为 blocked。报告记录来源、缓存指纹、
工具指纹和实际测试范围。Git 不可用的源码快照明确标记，不伪造提交号。
保留失败日志再重跑时请选择新输出目录。Ctrl+C 停止后续阶段并保留本轮产物。
