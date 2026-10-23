# P2-01：独立传感器同步器

本阶段把原 `LIVMapper::sync_packages` 的窗口选择、IMU 分组、点云切分和 LIO/VIO 交替迁入普通 C++ 库。目标和约束见[设计](superpowers/specs/2026-10-18-p2-01-synchronizer-design.md)及[执行计划](superpowers/plans/2026-10-18-p2-01-synchronizer.md)。参考提交为 `1bca9a3d706b2c25afd50a6d197db51e4474d567`。

主工程与独立候选均通过 98 项单元测试，5 项 ROS 集成、40 项启动、10 项 Python 及三个独立构建入口均通过。30 秒和完整 136 秒回放分别得到 296/1351 点，输入/更新计数、时间戳、轨迹与状态导出值均与参考零差异。默认并行 45 秒回放得到 446 点并正常退出。独立审阅结果在收尾时记录。

## 代码边界

| 文件/类型 | 职责 |
| --- | --- |
| `frontend/synchronized_measurements.h` | 唯一的模式、更新类型和同步输出；`common_lib.h` 只转发原名称。 |
| `sensor/synchronizer.h` | 成对传感器缓冲、同步返回值和普通 C++ 调用入口。 |
| `sensor/synchronizer.cc` | 同步与切分；没有 ROS、vikit、线程、时钟和文件输出。 |
| `LIVMapper` | 接收普通测量，持同一 mutex 入队/调用核心，把状态映射为 ROS 日志和原 bool 返回值。 |

`SensorBuffers` 将点云及时间、图像及时间放在同一测量里。`latest_imu_time` 保存 ONLY_LIO 原来用于覆盖判断的 double，不能改用 ROS 最后一次纳秒舍入后的时间；LIVO 仍使用队尾 IMU 时间。已有 IMU/LIO/VIO 消费者使用原字段名，避免第二份状态及多余转换。公开数据采用 C++17，PCL/OpenCV 仍是核心数据依赖。

调用者需独占修改缓冲和输出，或在整个操作期间持锁。时间为 double 秒，点 curvature 为相对毫秒。`last_lio_update_time` 由估计器在处理后推进；同步器只在首次有效扫描设置初值。输入按照采集顺序排队，点云在转交后不能由其他执行者改写；调用接口不提供时钟回退或 EOF 排空。

## 保留的数值语义

- LIVO 点偏移严格小于截止值才进入当前云，等于截止值进入剩余云；IMU 则消耗到终点，包含等号，并只输出大于上次更新时刻的样本。
- 切分只修改点的副本；点序、float 截止值和混合浮点运算表达式维持原样。
- 剩余云在下一次 LIO 整体复制进当前云，不按下一张图像重新切分。这是本次刻画并保留的原行为，不代表任意相机频率组合都已经验证。
- LIVO 在待执行 VIO 时仍要求三种队列非空。有限数据流最后一次 VIO 可能继续等待，须在后续显式设计 EOF 排空。
- ONLY_LO 不要求 IMU/图像；扫描结束时间取最后一个点的偏移，不排序或改成最大偏移。

新增的输入边界只处理无效云：首/尾空云或空指针移出并返回 `kInvalidCloud`，切分遇到内部无效云跳过；ONLY_LIO 单点帧移出，避免原先永久堵塞。其他模式允许单点。

## 验证方法与证据

本机证据目录沿用 `validation/P2-01-noetic-20261018/`，实际采集时间记在环境/回放记录。构建全部使用 `nice -n 10`、`-j1`，与回放串行，不启用 RViz。

先对原实现运行 78 项单元、10 项 Python、5 项真实 ROS 测试。新增的 3 个真实 Mapper 同步测试固定严格窗口、LIO/VIO 交替、覆盖时间舍入和 ONLY_LO 行为；其源码、程序、本地库及 XML 在 `characterization-artifacts/` 中保存。迁移后只调整这些测试的缓冲接线，数值断言不变。

独立核心用 20 项行为测试覆盖边界、等待、曝光偏移、所有权、多扫描及无效输入。可编译空实现得到 19 项内容/状态断言失败，再实现后 20 项通过；缺输入等待测试从一开始即可通过。编译或链接失败未作为行为 RED。

`reference.json` 固定 P1-03 最终候选作为本阶段参考：逐一校验源码、可执行文件、8 个本地动态库、bag、编译器、依赖及两份记录摘要。有效算法源码与本阶段基线相同，因此复用已采集的参考，不重写 P1-03 工作区。比较时另核对六个估计源文件及其归一化编译参数。

最终回放保存短段 30 秒、完整 136 秒和默认并行 45 秒。控制配对使用 `OMP_THREAD_LIMIT=1` 和 `OMP_DISPLAY_ENV=TRUE`，比较 TUM、mat_pre、mat_out 的时间、shape、有限性和逐元素相等；位置/姿态阈值为零，不做对齐或裁剪。结果只证明现有导出精度下相等，不代表未导出的全部内部状态逐位相同。默认并行用于完整性与正常退出检查。

控制配对实际结果：

| 记录 | 轨迹点 | mat_pre / mat_out | 结果 |
| --- | ---: | --- | --- |
| 30 秒 | 296 | 597×19 / 591×20 | 时间、计数、三份导出数据完全相同。 |
| 完整 136 秒 | 1351 | 2707×19 / 2701×20 | 时间、计数、三份导出数据完全相同。 |
| 默认并行 45 秒 | 446 | 全部有限 | 正常退出，不用于逐值确定性结论。 |

完整输入为 27447 IMU、1355 点云、1355 图像；更新为 1351 LIO、1350 VIO。全部文件保留，没有对齐、裁剪或误差阈值放宽。详细检查见 `short/full-comparison.json`、`short/full-arrays.json`、`build-settings-comparison.json`。

## 在另一台电脑重建

从工作区根目录执行独立构建，无须 ROS 或 vikit：

```bash
nice -n 10 cmake -S src/livo-atlas/tests/frontend -B validation/p201-frontend
nice -n 10 cmake --build validation/p201-frontend -- -j1
(cd validation/p201-frontend && ctest --output-on-failure)
```

该入口查找 Eigen3、PCL common、OpenCV core、GTest，编译实际同步源和两个公共头的独立包含检查。数学/标定、预处理入口仍为 `tests/core` 和 `tests/sensor`。只迁移源码和数据，在目标电脑新建 build/devel；不要复用其他路径或机器的 CMakeCache。

ROS1 按[通用基线说明](validation.md)准备 Noetic 依赖及独立源码副本，源码包仅为 livo-atlas，另行准备固定版本 `6c886c8e5d83997806e00294826d528cea3581dd` 的 rpg_vikit。编译时显式设置候选副本的 `CATKIN_DEVEL_PREFIX`，然后依次构建 `fastlivo_mapping`、`lidar_callback_test`、`run_tests`。ROS 回调测试需要先 source **独立副本**的 devel/setup.bash，再执行：

```bash
rostest livo_atlas lidar_callbacks.test
```

构造函数仍会写旧 ROOT_DIR 日志，ROS 集成、启动和回放均在独立副本执行。实际 ROS 用例为 rosunit XML 中的 5 项，不重复计算 rostest 包装项。Python 检查在包目录运行：

```bash
/usr/bin/python3 -m unittest discover -s tests -p 'test_*.py' -v
```

40 项启动检查使用 `tools/check_calibration_startup.py --workspace <独立副本> --output <新目录> --port <空闲端口>`。回放示例从工作区根目录执行，每次输出必须是新目录：

```bash
OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE /usr/bin/python3 \
  src/livo-atlas/tools/run_baseline.py \
  --workspace validation/p201-candidate --bag Retail_Street.bag \
  --output validation/p201-short --duration 30 --rate 0.5 --port 11392
```

再使用 duration=136 做完整对照，最后取消两个 OMP 环境变量运行 45 秒。参考端必须使用固定提交同环境构建，或通过所有摘要/环境检查后复用既有记录。`tools/compare_trajectories.py` 的两个最大误差设为 0；另外使用 NumPy 对三份导出数据做 `array_equal` 检查。

## 本轮范围与后续

同步核心已隔离，但整个 Frontend 尚未完成隔离。P2-02 将整理单一前端状态和处理调度；P2-03 迁移 ROS 发布与文件输出。Humble 接入、去除 vikit、GTSAM 后端和 3DGS 数据导出继续按总体架构推进。

完整时钟回退 reset、首帧积分时间锚点和原特征平面参数 `disB` 未初始化问题仍需单独数值验证。本 bag 只覆盖 Livox + IMU + 相机的非特征分支；合成边界用例不能替代其他设备数据。没有据此宣称 ARM、Humble、1 倍实时性能或真值定位精度通过。

新代码按 Google C++17 约定人工核对并执行 `git diff --check`；本机没有 clang-format，不声称自动格式化已运行。
