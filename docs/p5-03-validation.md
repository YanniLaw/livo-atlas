# P5-03：离线 COLMAP 研究数据导出验证

本阶段以 `refactor2` 的 `55646e1`（P5-02）为基线，增加普通 C++ `ExportColmap` 和
安装后的 `livo_atlas_export_colmap`。复用现有 io 库，不改 ROS main、节点配置、前端算法或数据集 schema。
设计见[设计文档](superpowers/specs/2026-10-30-p5-colmap-export-design.md)，步骤见
[实施计划](superpowers/plans/2026-10-30-p5-colmap-export.md)，使用见[导出说明](colmap-export.md)。

## 环境与证据

本机 Ubuntu20.04/Noetic，已有 ros2 容器 Ubuntu22.04/Humble。
新证据只写 `validation/P5-03-20261030` 和容器 `/tmp/livo_atlas_p5_03`；冻结 P5-02、原始 Log 和 bag 只读。
复用已验证的 Sophus、Livox 消息和 ROS2 bag。vikit 当前缺失，保持缺失。
nice/-j1 串行构建，构建与回放串行，回放使用 OMP_THREAD_LIMIT=1，无 RViz。

官方 Retail_Street.bag 为 1,961,140,106 字节，SHA-256：

```text
1745bee1d570749679bad7a0392100477ab62d50aa77f7f629bc57f2a72e4427
```

两端各与本环境的冻结 P5-02 对比；跨平台读取用于验证数据可移植性。

## 构建、接口和错误路径

| 项目 | 结果 |
| --- | --- |
| 冻结二进制基线 | 22 个二进制 / 205 项，XML/TMPDIR 写入新阶段 |
| 普通核心 | 229 项通过 |
| Noetic 单元 / 真实 ROS | 249 / 8 项通过 |
| Humble C++，含真实 DDS | 265 项通过 |
| 普通公开头独立编译 | 45 个通过 |
| shared / static SDK | 各 229 项，移动安装前缀后实际三模式与原始/优化导出通过 |
| 关闭 OpenMP SDK | 关闭测试构建，移动前缀后实际三模式与导出通过 |
| Humble ament SDK / 实际 colcon 节点 | 迁移消费及实际节点与14个自有运行库检查通过 |
| 两端实际进程启动 | Noetic 50 / Humble 16 项通过 |
| 导出 CLI | shared、static、无OMP、Noetic、ament 各30项通过 |
| Python 工具 | 本机13项通过、1项版本跳过；Humble对应5项通过 |

新增24项普通测试覆盖相机和点的坐标、半像素约定、全部畸变系数、缩放一次、有效边缘、
正深度、遮挡与数量上限、颜色、时间、乱序优化位姿和冻结前缀、无视觉帧、空种子及失败无完成标记。
路径测试覆盖现有目录/文件/符号链接及源目标重叠；新增从源目录内使用相对输出路径的真实回归。
CLI 同时检查返回码、JSON、非法数字、版本及源资产保持。

camera-red.log、exporter-red.log、cli-red.log 保留实现前失败。
外部兼容验证促成三项真实 RED→GREEN 修正：

- convention-red.log：输出使用居中虚拟相机，区分 OpenCV 整数像素中心与 COLMAP 角点坐标。
- relative-red.log：相对输出路径先转绝对路径再 canonicalize，避免源内新路径绕过重叠保护。
- rigid-red.log：先将每个已校验的近似旋转投影到最近 SO(3)，再一致地组合相机、body 和点。

SO(3) 修正仅发生在离线导出；真实数据中单个输入旋转的最大元素变化为 `2.3210220334579645e-7`。
原始图像、矩阵、外参、点云和位姿版本均保持，清单声明 `rotation_projection=nearest_so3`。
居中整流可能损失边缘覆盖，原始资产始终保留。

## 官方读取实现与真实数据

验收直接调用未修改的上游实现：

- COLMAP `3.11.1` 的 [read_write_model.py](https://github.com/colmap/colmap/blob/3.11.1/scripts/python/read_write_model.py)。
- 官方 3DGS 提交 `54c035f7834b564019656c3e3fcc3646292f727d` 的
  [colmap_loader.py](https://github.com/graphdeco-inria/gaussian-splatting/blob/54c035f7834b564019656c3e3fcc3646292f727d/scene/colmap_loader.py)。

读取器、许可、下载地址和 SHA 位于本阶段 external-readers.json；不成为产品依赖。
实际解析 cameras.txt、images.txt、points3D.txt，另逐图检查 PNG、源点成员、颜色、正深度、
相机中心、重投影、双向关联及所有时间/曝光字段。未运行 PyTorch Scene、CUDA 或3DGS训练。

以下六组由移动后的普通 shared SDK CLI 从冻结 P5-02 数据或其独立副本导出：

| 数据 | 版本 | 图像 | LiDAR种子 |
| --- | --- | --- | --- |
| Noetic 3秒 | 0 | 2 | 3,674 |
| Noetic 完整 | 0 | 172 | 328,626 |
| Noetic 完整副本 | 1 | 172 | 328,626 |
| Humble 3秒 | 0 | 2 | 3,674 |
| Humble 完整 | 0 | 172 | 328,610 |
| Humble 完整副本 | 1 | 172 | 328,610 |

全部点/图像/关联通过；最大重投影误差 `7.333e-12` 像素，相机中心误差小于 `5e-14` 米。
独立源点成员检查在 float32 表示下误差为0。全部 PNG 与规定的 CV_32FC1 映射加 INTER_LINEAR 参考逐通道一致。
直接调用 cv::undistort 使用不同内部映射表示，在完整数据的锐利边缘最大相差6个8位通道值；
这项差异单独记录，不用它替代规定整流流程的精确检查。

两份完整副本通过实际 C++ WritePoses 接口倒序写入 `t_x += 10 + 0.01*id` 的版本1。
逐帧修正改变相机和相对几何，所有172个相机中心的修正自检误差小于 `3.1e-14` 米。
优化前后 PNG 摘要与 frames.csv 字节一致，原始资产不变，副本只增加 poses/1.yml。

种子每点只关联本帧的一次真实投影；ERROR=0表示构造关系，不代表多视角三角化或BA精度。
本阶段交付研究输入和格式兼容，实际训练效果需要后续选择场景与训练验证。

## 官方 bag 回归与最终收尾

最终 Noetic / Humble 各短段和完整回放全部通过，与各自冻结 P5-02 配对：

| 环境 / 输入 | 轨迹位姿 | 关键帧 | 原始点数 |
| --- | --- | --- | --- |
| Noetic / 3秒 | 26 | 2 | 40,086 |
| Noetic / 136秒 | 1,351 | 172 | 3,763,228 |
| Humble / 3秒 | 26 | 2 | 40,086 |
| Humble / 136秒 | 1,351 | 172 | 3,762,300 |

四数组逐值精确一致，点云语义字段和图像像素摘要精确一致；短段各57个旧产品逐字节一致。
每个完整数据集的527个原始资产摘要与冻结参考一致，实际安装后的 C++ 读者重读全部帧；Humble 数据由 Noetic SDK 消费。
验收脚本核对当前源码、两端各节点加14个自有库、CLI及原始bag摘要；原配置/launch、31个Log文件、其他分支与标签保持。
最终源码/运行产物绑定见本阶段 acceptance.json，整阶段独立评审及完整历史恢复记录将在收尾时补充。
首次 Noetic 完整回放的数值差异及受控复查保留在 regression-first-failure 与 probe-io-*，不会覆盖失败证据。
相同源码/二进制的四组8秒受控回放及后续完整回放均精确通过，尚未确定首次差异的根因，没有据此修改算法或放宽容差。
另一次 Humble 完整回放恰逢会话服务重启，四数组、全部2,701条点云和527个关键帧资产均精确一致，
但观察端缺失第721个图像样本；该次严格判为失败，保留在 transport-first-failure。恢复后相同源码/二进制重新完整回放通过，
全部1,350条图像观测与参考一致。没有降低消息计数或精确比较要求。

跨电脑按[普通构建](core-build.md)、[双ROS构建](ros-builds.md)和[导出说明](colmap-export.md)操作。
先在目标工具链重新编译，停止数据集写入后导出到新目录，再整体复制导出目录。
普通接口和安装 CLI 不要求运行 ROS、COLMAP 或 CUDA。
