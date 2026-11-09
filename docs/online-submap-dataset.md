# 在线子地图的保存、检查与离线消费

本接口提供成员索引、观测快照和版本化位姿，不包含闭环后端或高斯训练。
设计见[在线子地图契约](superpowers/specs/2026-11-08-online-submaps-design.md)，
已完成的实际验证见[阶段记录](p6-01-validation.md)。

## 启用

默认仍为关闭子地图；启用但未指定模式时，继续使用旧 keyframe_groups 索引。
显式在线配置如下。ROS1 使用斜线参数名，ROS2 使用点分参数名，两端读取同一清单。

```yaml
keyframes:
  enabled: true
submaps:
  enabled: true
  mode: online
  max_keyframes: 20
  online:
    translation_threshold: 5.0
    rotation_threshold_deg: 45.0
    time_threshold: 20.0
    support_keyframes: 2
```

平移单位米、时间单位秒；运动和时间阈值要求有限且大于零。支持帧数量在 0 到
max_keyframes 之间。主成员连续且唯一归属；支持帧引用前组末尾观测，不复制原始资产。
达到运动或时间阈值时当前帧进入新组；达到数量上限时插入后立即封存。
封存停止接收成员，后续位姿仍可更新。正常退出须调用 Finish。

两个回放工具均支持 `--submap-mode online`，自动启用关键帧和子地图保存。
原 `--save-submaps` 保持默认旧分组行为。summary.json 增加 submap_mode 字段。
Noetic 工具要求独立源码快照；Humble 工具读取经核验的 ROS2 bag，不直接读取 ROS1 bag。
完整回放和环境设置见[双版本说明](ros-builds.md)。

## 检查工具

安装后的旧调用方式保持：

```bash
livo_atlas_inspect_submaps /path/to/keyframes
livo_atlas_inspect_submaps /path/to/keyframes 1
```

显式读取在线命名空间：

```bash
livo_atlas_inspect_submaps /path/to/keyframes --online
livo_atlas_inspect_submaps /path/to/keyframes --online 1
```

默认版本 0 使用原始 odom 位姿；正版本使用 map 位姿。版本号必须为规范无符号十进制整数。
成功退出为 0，参数错误为 2，数据或版本错误为 1。工具不自动猜测或混合两套索引。

在线 JSON 的 inspection 固定为 metadata，仅检查索引、引用和位姿，不替代
KeyframeStore::Read 对完整观测及图像/点云资产的检查。

| 字段 | 含义 |
| --- | --- |
| submaps、active_submaps | 已提交子地图数量、仍未封存的数量 |
| primary_frames | 已进入在线索引的主成员数量 |
| support_references | 支持观测引用总数，不是新增帧数 |
| unindexed_frames | 已保存原始帧中，尚未进入在线索引的尾帧数 |
| pose_version | 选择的优化版本，0 为原始位姿 |
| stored_frame_count | 正版本文件冻结时覆盖的帧数；版本 0 无文件，显示 0 |
| frame_count | 本次显式物化覆盖的当前全部已提交原始帧数 |
| optimized_frame_count | 实际参加该版本优化的前缀长度，版本 0 为 0 |

例如冻结文件覆盖 M=3、优化前缀 N=2，后来保存第 4 帧：检查版本 1 显示
stored_frame_count=3、frame_count=4、optimized_frame_count=2。
第 4 帧依据最后优化锚点延续，不能把它描述为已参与优化；冻结文件不被覆盖。

## 存储与异常退出

在线数据保存在 keyframes/online_submaps/：dataset.yml 声明配置和坐标约定，
records/ID/REVISION.yml 保存不可变成员修订，poses/VERSION.yml 保存冻结优化快照。
原始关键帧、旧 submaps/ 和旧 poses/ 的 schema 与用途保持。

修订采用 checked-close 和同目录 rename。原始帧与多个索引文件之间没有跨文件事务，
也没有断电 fsync 保证。读者忽略 pending 文件，报告仍活动的组及未索引尾部；不恢复追加，
不自动补组或封存。成功检查表示已提交前缀一致，不表示上次运行正常结束。
格式可检测中间缺号和现存损坏记录，不能检测整段最高修订被删除的情况。

## 普通 C++ 消费与兼容出口

LivoSystem 的在线回调传递拥有数据的 Keyframe、已提交修订及完整 PoseSnapshot。
应在处理前注册回调，仅登记或入队；不得重入系统或通过 cv::Mat/PCL 别名修改共享资产。
异步消费者保留 shared_ptr 或通知值，不能保留回调参数引用，也不能并发调用串行 Store。
完整快照按需物化，成本为 O(M)；未注册消费者时不逐帧复制完整轨迹。

在停止写入的数据集副本中，可以显式生成旧格式的完整位姿出口：

```cpp
livo_atlas::KeyframeStore frames(dataset);
livo_atlas::OnlineSubmapStore online(dataset / "online_submaps", frames);
const std::uint64_t version = 1;
const auto poses = online.MaterializePoseSnapshot(version, frames.ids().size());
frames.WritePoses(version, poses.poses);
```

只对正版本使用 WritePoses；同名旧版本已存在时拒绝覆盖。原在线快照仍保留优化前缀与
尾部策略，兼容副本不反向驱动在线地图。现有 COLMAP 导出可以读取此完整位姿出口；
旧子地图检查还需要显式生成其独立的旧成员索引。

安装消费示例位于 tests/installed_consumer，通过 find_package(livo_atlas) 和导出目标构建，
无需源码生产头或原构建树。其 `--online-fixtures NEW_DIRECTORY` 模式生成确定的小数据集，
供 tests/tools/online_submap_cli_test.py 验证实际安装的检查工具。
