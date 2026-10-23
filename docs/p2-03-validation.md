# P2-03：输出边界与复现

本阶段将 ROS 发布、点云着色和文件写入分开；完整回归和整阶段评审结果在下方登记。实施基线为 `3eb1f8cb72f4b3d9898dbd760aabf70e65db5bb7`，主要实现提交为 `380e42944d898bc3f0a7d67cffd97c4cfabd2335`。

## 实际边界

- `FrontendResult` 提供状态值、点云视图、图像、相机从世界的四元数/平移、IMU 诊断、可选有效点和平面快照。已删除 `visual()` / `local_map()` 借用口。
- `OutputProcessor` 处理着色累积、发布间隔和 COLMAP 图像去畸变，不发布或写文件；暂时私有使用现有 vikit 相机。
- `ResultWriter` 写轨迹、先验/后验、IMU 日志、PCD、原图、调试图和 COLMAP 文件。它有不依赖 ROS/vikit 的独立构建。
- `ros1::ResultPublisher` 拥有话题、TF、Path 和 IMU 高频里程计的消息发布，不处理文件。
- `LIVMapper` 负责 ROS 参数、输入回调、同步和顺序调用；旧 IMU/VIO/地图文件不再读取 ROS 参数、发布 ROS 消息或写文件，源码路径宏 `ROOT_DIR` 已移除。

调用顺序为 Frontend → OutputProcessor → ResultWriter → ResultPublisher。发布可以省略；后端若要异步保留图像或点云，应在下一次 Process 前复制。状态本身是值快照。

本阶段仍是 catkin/Noetic 工程。完整前端的私有相机及构建依赖仍包含 vikit/ROS；旧算法的终端耗时统计暂留。P3 再移除 vikit、整理核心独立构建及剩余计时依赖；ROS2 Humble、GTSAM 后端和 3DGS 训练均尚未实现。

## 输出目录

新增全局 ROS 参数 `output/directory`。默认依次采用 `$ROS_HOME/livo_atlas`、`$HOME/.ros/livo_atlas`、当前目录的 `livo_atlas_output`。例如在启动节点前执行：

```bash
rosparam set /output/directory /tmp/livo_atlas_run01
```

输出保留原相对命名：`mat_pre.txt`、`mat_out.txt`、初始化后的 `imu.txt`、`result/<seq>.txt`、`pcd/`、`image/`、`Colmap/`、`ref_cur_combine/`。按需创建目录，文件写入失败使节点报错退出。不同并发实例使用不同目录。

PCD 正间隔保留原来的部分尾批不导出行为，负间隔在正常退出时导出全量；类型 1 逐帧写 IMU 系点云。同时启用 PCD 和 COLMAP 时，保留原脚本的重建重置行为：清空输出根目录中的 `Colmap/images` 和 `Colmap/sparse/0`，其他目录不清空。独立写入器的计数每实例从头开始，Finish 幂等；失败后的 Finish 继续报告失败。

COLMAP 保留原研究导出格式，包括相机从世界的位姿和下采样彩点；这些文件没有完整的特征观测轨迹，不等同于完成 SfM 重建。相机四元数保留标定舍入误差，不额外归一化。

## 已执行的直接检查

| 检查 | 结果 |
| --- | --- |
| 开发工作区与独立候选的真实 GTest | 各 132 项通过 |
| 实际 ROS 回调、普通结果发布、位姿/marker、配置输出目录 | 11 项通过 |
| 启动配置，包括默认目录、无效间隔、非法路径和不可写目录 | 50 项通过，无源码日志写入 |
| Python 工具 | 13 项通过 |
| 无 ROS 的独立 core / sensor / synchronization / writer | 28 / 13 / 20 / 15 项通过 |
| 普通公共头独立编译 | 18 项通过 |
| 定向 Frontend 生命周期诊断 | 四次构建/实际建图/销毁通过 |
| 真实非空残差 gdb 回放 | 3 秒，26 个轨迹点，正常退出 |

生命周期检查为 Linux 下链接 ASan/LSan 运行库的定向泄漏诊断，不是所有源文件的完整地址插桩。未建图对照保留外部系统分配报告；实际建图用已证明的外部分配位置排除项后退出 0。具体日志、抑制文件摘要和程序摘要保存在 `leak-control/`、`leak-green/`，不据此声称整个系统无任何内存问题。

六个原估计源文件的编译命令只允许工作区路径和移除 `ROOT_DIR` 宏的差异；未添加 native ISA/fast-math。仍保持 estimator DSO 优先于 native vikit，避免混用 Eigen 对齐分配约定；`runtime-dependencies.json`、gdb 和实际回放共同验证这一点。

## Bag 回归登记

证据目录：工作区的 `validation/P2-03-noetic-20261024/`。参考是新建的独立源码/构建副本，P2-02 冻结目录未修改。

`Retail_Street.bag` SHA-256：`1745bee1d570749679bad7a0392100477ab62d50aa77f7f629bc57f2a72e4427`。

参考与候选均已完成 30 秒、完整 136 秒和 3 秒导出场景，分别得到 296、1351、26 个轨迹点；由 `reference.json` 固定参考源码、程序、库、结果和参数，`reference-tools.json` 固定实际采集工具版本。

| 候选场景 | LIO / VIO 更新 | 轨迹点 | 结果 |
| --- | --- | --- | --- |
| 单线程控制，30 秒 | 296 / 295 | 296 | 四组导出数组与点云/图像语义摘要完全一致 |
| 单线程控制，完整 136 秒 | 1351 / 1350 | 1351 | 四组导出数组与点云/图像语义摘要完全一致 |
| 默认线程，45 秒 | 446 / 446 | 446 | 正常完成，输入无丢失；不作为零误差配对 |
| 单线程控制，3 秒导出 | 26 / 25 | 26 | 数组/消息一致，57 个 PCD、PNG、COLMAP/pose 文件逐字节一致 |

完整回放接收 27447 个 IMU、1355 帧 LiDAR、1355 帧图像，发布 2701 帧点云和 1350 帧跟踪图。四组数组形状分别为 TUM `1351×8`、mat_pre `2707×19`、mat_out `2701×20`、IMU `27365×7`，最大绝对差均为 0，全部有限。

语义摘要比较 PointCloud2 的声明字段和图像像素，忽略墙钟、seq 和 PCL 未定义填充；同时检查字段布局、frame_id 和数量。TUM、mat_pre、mat_out、IMU 数组采用零容差。0.5 倍回放用于行为回归，不能据此判断绝对定位精度或实时性能。

## 在另一台电脑复现

只复制源码、同版本 rpg_vikit 和 bag，重新建立 build/devel，不复制 CMakeCache。vikit 固定 `6c886c8e5d83997806e00294826d528cea3581dd`，本阶段没有修改它。采用 C++17、Google 风格；旧兼容入口逐个模块迁移，不做全仓库机械改名。

```bash
source /opt/ros/noetic/setup.bash
nice -n 10 catkin_make -j1
source devel/setup.bash

nice -n 10 cmake -S src/livo-atlas/tests/io -B validation/io-local
nice -n 10 cmake --build validation/io-local -- -j1
(cd validation/io-local && ctest --output-on-failure)
```

Bag 验证继续使用独立 catkin 源码副本，按[通用验证说明](validation.md)配置构建；当前工具默认将节点输出写入本次证据目录内的 `node_output/`：

```bash
OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE /usr/bin/python3 \
  src/livo-atlas/tools/run_baseline.py \
  --workspace validation/candidate_ws --bag Retail_Street.bag \
  --output validation/short-new --duration 30 --rate 0.5 --port 11426 \
  --semantic-output
```

验证旧版节点才加 `--legacy-output`；`--save-products` 同时验证 PCD、图像、COLMAP。完整场景使用 `--duration 136` 和新输出目录/端口；默认线程场景移除两个 OMP 环境变量后运行 45 秒。构建始终 nice -n 10/-j1，不能与 bag 回放并行，无 RViz。

包外脚本 `build_all.py`、`run_validation.py`、`run_replays.py`、`check_builds.py`、`verify_task*.py` 保存本机实际操作；迁移电脑时若要保留旧证据，需要单独复制整个证据目录。中断目录应先改名归档再重跑，不覆盖已完成的参考或复用未通过的结果。当前直接检查与回放验收通过，整阶段评审和源码 bundle 在完成后登记。
