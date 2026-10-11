# P0-01：当前编译与运行入口

本文记录现有代码，非目标架构。只验证增量编译和 AVIA 无数据启动，不代表 bag 标定、建图效果或其他配置已经验证。

## 1. 本地环境与代码

| 项目 | 实际读取结果 |
|---|---|
| 工作区 | `/home/yanni/livo-atlas_ws` |
| 包目录 | `src/livo-atlas` |
| 包名 / CMake project | `livo_atlas` |
| 基准 commit | `ca556e5ad6706ba20dc8dff91010faae546b7528` |
| 系统 | Ubuntu 20.04.6 LTS |
| ROS | Noetic，`/opt/ros/noetic` |
| 编译器 / CMake | GCC 9.4.0 / CMake 3.16.3 |
| C++ 标准 | C++17 |
| 构建工具 | catkin_make，已有 `build/` 与 `devel/` |
| 工作区内依赖 | `src/rpg_vikit` |
| 当前 Sophus 缓存路径 | `/home/yanni/3rd-party/Sophus/build` |
| 当前 OpenCV 缓存路径 | `/usr/lib/x86_64-linux-gnu/cmake/opencv4` |

以上依赖路径来自当前构建缓存，不代表在其他机器可直接复用。P0-01 没有全新安装依赖，也没有清理已有构建目录。

## 2. 构建入口

工作目录为工作区根目录：

```bash
cd /home/yanni/livo-atlas_ws
source /opt/ros/noetic/setup.bash
catkin_make -j2 -l2
source devel/setup.bash
rospack find livo_atlas
```

构建成功时应出现 `Built target fastlivo_mapping`，退出码为 0；包路径应为当前工作区的 `src/livo-atlas`。`-j2 -l2` 是此次验证的资源限制，不是算法线程数。

当前 CMake 创建 `vio`、`lio`、`pre`、`imu_proc`、`laser_mapping` 库和 `fastlivo_mapping` 可执行文件。程序位于 `devel/lib/livo_atlas/fastlivo_mapping`。

CMake 在包内直接设定 Release；缓存中的 `CMAKE_BUILD_TYPE` 显示为空不等于包按 Debug 编译。OpenMP、mimalloc 为条件检测项；Sophus、Eigen、PCL、OpenCV、Boost 和列出的 catkin 组件为构建依赖。当前没有该包的 install 规则，本任务验证 devel 空间启动。

## 3. 启动与配置映射

| launch | 全局算法参数 | 节点相机参数 | 其他节点 |
|---|---|---|---|
| `mapping_avia.launch` | `config/avia.yaml` | `camera_pinhole.yaml` | republish；可选 RViz |
| `mapping_avia_marslvig.launch` | `config/MARS_LVIG.yaml` | `camera_MARS_LVIG.yaml` | republish；可选 RViz |
| `mapping_ouster_ntu.launch` | `config/NTU_VIRAL.yaml` | `camera_NTU_VIRAL.yaml` | 可选 RViz |
| `mapping_hesaixt32_hilti22.launch` | `config/HILTI22.yaml` | `camera_fisheye_HILTI22.yaml` | 可选 RViz |

四个 launch 都提供 `rviz` 参数，默认 `true`；都启动包 `livo_atlas` 中的 `fastlivo_mapping`，节点名 `/laserMapping`。`src/main.cpp` 的 `ros::init` 也使用 `laserMapping`。

入口链路：`main.cpp` → `LIVMapper` 构造（加载参数、初始化文件和组件）→ 初始化订阅发布 → `run()` 循环。

算法 YAML 在全局参数空间加载；相机 YAML 在 `/laserMapping/` 下加载。普通 NodeHandle 读取 `common/*`、`imu/*`、`vio/*` 等配置，相机加载器明确使用 `laserMapping` 命名空间。当前不能把节点任意改名后假定相机参数仍可加载。

现有 AVIA 配置使用 `/left_camera/image`、`/livox/lidar`、`/livox/imu`，与当前 bag 的话题名一致。相机配置为 Pinhole、1280×1024、scale=0.5。完整标定匹配待 P0-02 及后续契约核查，不能只根据话题名判断。

republish 负责 compressed → raw；现有 bag 自带 raw 图像。P0-01 保持 launch 原样，未删除该节点。

## 4. 模式边界

当前分支表达式：`(img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO`。

| 拟使用模式 | 常规参数组合 | 当前核查范围 |
|---|---|---|
| LIVO | img_en=1，lidar_en=1，imu_en=true | 默认 AVIA 无数据启动已验证 |
| LIO | img_en=0，lidar_en=1，imu_en=true | 存在代码分支，未运行验证 |
| LO | img_en=0，lidar_en=1，imu_en=false | 存在代码分支，未运行验证 |

配置键分别位于 `/common/img_en`、`/common/lidar_en`、`/imu/imu_en`。不将其他组合视作已支持模式。即使关闭图像，现有初始化仍会加载相机配置，不应直接移除它。

## 5. 本地启动验收步骤

本任务不播放 bag，不开启保存点云/图像/COLMAP，不开启 RViz。以下命令使用独立 ROS master；执行前确认端口 11321 未被其他程序使用。

现有代码在启动时截断 `Log/mat_pre.txt` 和 `Log/mat_out.txt`。先备份，节点完全退出后恢复。不要与另一个运行中的 livo-atlas 实例同时执行。

终端 A：

```bash
cd /home/yanni/livo-atlas_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
P001_RUN_DIR=$(mktemp -d /home/yanni/livo-atlas_ws/validation/P0-01-user-XXXXXX)
mkdir -p "$P001_RUN_DIR/log-backup" "$P001_RUN_DIR/ros-home"
cp -p src/livo-atlas/Log/mat_pre.txt src/livo-atlas/Log/mat_out.txt "$P001_RUN_DIR/log-backup/"
export ROS_HOME="$P001_RUN_DIR/ros-home"
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=127.0.0.1
echo "$P001_RUN_DIR"
roslaunch -p 11321 livo_atlas mapping_avia.launch rviz:=false
```

终端 B（A 保持运行）：

```bash
cd /home/yanni/livo-atlas_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=127.0.0.1
rosnode list
rosnode info /laserMapping
rosparam get /laserMapping/cam_model
rosparam get /common/lid_topic
```

预期：存在 `/laserMapping`、`/republish`、`/rosout`；相机模型是 `Pinhole`，雷达话题是 `/livox/lidar`；节点持续运行至少 10 秒，无进程异常退出。不输入数据时不要求轨迹或点云产生。

随后在终端 A 按 Ctrl-C，等所有节点退出，再在同一终端恢复：

```bash
cp -p "$P001_RUN_DIR/log-backup/mat_pre.txt" src/livo-atlas/Log/mat_pre.txt
cp -p "$P001_RUN_DIR/log-backup/mat_out.txt" src/livo-atlas/Log/mat_out.txt
cmp "$P001_RUN_DIR/log-backup/mat_pre.txt" src/livo-atlas/Log/mat_pre.txt
cmp "$P001_RUN_DIR/log-backup/mat_out.txt" src/livo-atlas/Log/mat_out.txt
```

`cmp` 无输出且退出码为 0 表示恢复一致。若中途关闭终端，使用上面打印的实际目录恢复备份，不能用新空目录替代。失败时保留 ROS 日志并报告，不执行清理旧结果的脚本。

## 6. 助手实测与待办

- `catkin_make -j2 -l2`：增量构建退出码 0。
- 无 RViz、无 bag 启动 15 秒：节点创建、相机参数加载成功，随后收到计划内 SIGINT 并关闭；timeout 返回 124 是达到预定时长，不是节点崩溃。
- 原日志已从备份恢复，并用 `cmp` 核对一致。
- 首次沙箱内启动因 `netifaces.interfaces()` 权限限制失败；经授权在本机重试成功。
- 未进行干净构建、RViz 显示、bag 回放、轨迹评估、其他 launch 或模式验证。

证据位于工作区 `validation/P0-01/`：`build.log`、`startup.log`（首次限制）、`startup-local.log`（本机成功）和 `ros-home/log/`。

盘点发现并留待后续任务处理：包元数据仍包含旧 LOAM 描述与不同的许可证标记；部分 CMake 依赖未对应声明在 package.xml；两个 AVIA launch 含游离调试文本；日志路径硬编码；Sophus 依赖当前机器缓存路径。本次仅记录，不修改这些行为或元数据。
