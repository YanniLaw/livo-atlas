# P1-03：内部测量与 ROS1 转换

本阶段正在实施，最终验收结果在完成回放和审阅后补充。
参考提交为 `10e9222b122c16a4ecb36a2b19e30d113ed78d25`，执行计划见
[P1-03 计划](superpowers/plans/2026-10-18-p1-03-measurements.md)。
仅修改 livo-atlas；vikit 仍为 `6c886c8e5d83997806e00294826d528cea3581dd`。

## 已取得的证据

当前 67 项 C++、10 项 Python、40 项真实启动检查全部通过。
独立入口通过 28 项数学/状态/标定测试、8 项预处理测试，以及 9 个公共头的独立包含检查。
30 秒配对回放均输出 296 个轨迹点；传感器时间、输入/更新数量、轨迹和状态日志导出值零差异。
初次完整配对回放均为 1351 个轨迹点，输入/更新计数、传感器时间、轨迹及状态日志导出值零差异；默认并行 45 秒得到 446 点并正常退出。
检查还发现旧 `last_prop_end_time` 未初始化。本阶段将单独复现和修复，并重新回放最终候选；独立审阅尚未完成。

## 数据边界

| 类型/入口 | 契约 |
| --- | --- |
| `ImuMeasurement` | 传感器时间 double 秒，Eigen 角速度/加速度值；不保留消息指针，不在适配器归一化。 |
| `ImageMeasurement` | 时间 double 秒，拥有独立像素的 BGR8 `cv::Mat`；后续 Mat 复制共享引用计数，顺序前端是唯一修改者。 |
| `LidarScan` | 时间 double 秒，保留原字段精度和顺序的七类型 variant；转换不做筛选、补时或排序。 |
| `PointCloudMeasurement` | 时间和 PCL 原生 Ptr；回调创建独立云后交给顺序同步/估计路径，下一帧不覆盖它。 |
| `ros1::ConvertImu` | 输入已按原回调修正 ROS Time 的消息，复制 header 时间和原始数值。 |
| `ros1::ConvertImage` | cv_bridge 目标仍为 `bgr8`，使用 toCvCopy；支持原路径的 RGB、灰度及其他 cv_bridge 可转换编码，错误不修改输出。 |
| `ros1::ConvertLivox` | 检查 point_num 与向量长度一致后复制实际使用字段；时间只取 header。 |
| `ros1::ConvertPointCloud` | 保留原六种 PointCloud2 布局和 PCL 解码。检查注册字段的名称、类型、count、offset，以及维度、步长和精确数据长度。大端数据明确拒绝；不把未解码字节当成有效点。 |
| `Preprocess::Process` | 接收普通 LidarScan，直接调用对应重载；输出为独立云副本，空输入清空输出。 |

结构与校验是本阶段的变更；原预处理计算保留。输入结构中的原始时间字段不能依据字段名自行统一单位：

| 输入类型 | 原有点内时间计算（最终 curvature 为 ms） |
| --- | --- |
| Livox | `offset_time / float(1000000)`；非特征分支保留首点检查和跳变时 `+0.004166667f`。 |
| Ouster | `t / 1e6`；非特征分支仍按结果排序。 |
| Velodyne | `time / 1000.0`；缺时间时仍按各线 yaw 推算。适配实际驱动时须核对其字段单位。 |
| L515 | 零偏移，RGB 继续存入 normal_x/y/z。 |
| XT32 | 非特征分支 `(timestamp - first.timestamp) * 1000.f`；特征分支保留原 `timestamp / 1000.0` 与原 yaw 回退。 |
| Pandar128 | `(timestamp - first.timestamp) * 1000.f`，byte 强度 `/255.0f`，按偏移排序。 |
| Robosense | `(timestamp - first.timestamp) * 1000.0`，float 强度不缩放，保留 NaN/距离筛选和排序。 |

ROS1 时间修正规则继续留在接入层：IMU 先减偏移并经过 fromSec/toSec，再可选整秒驱动补偿并再次 fromSec；图像加 img_time_offset；标准点云加 lidar_time_offset，Livox 路径仍不应用该参数。高频里程计的 ROS stamp 单独保留，不能经 double 重建。

## 构建与测试

公共数学/标定头的独立入口维持不变。新增 sensor 独立入口允许 Eigen、PCL、OpenCV 和 OpenMP，不查找 ROS 或 vikit；它构建真实 preprocess.cpp 和三个公共头的独立包含检查。

在干净目录中执行，避免复用另一台电脑的 CMakeCache：

```bash
nice -n 10 cmake -S src/livo-atlas/tests/core -B validation/p103-core
nice -n 10 cmake --build validation/p103-core -- -j1
(cd validation/p103-core && ctest --output-on-failure)
nice -n 10 cmake -S src/livo-atlas/tests/sensor -B validation/p103-sensor
nice -n 10 cmake --build validation/p103-sensor -- -j1
(cd validation/p103-sensor && ctest --output-on-failure)
```

Noetic 主工程仍使用 catkin，新增 `measurement_conversions_test`、`sensor_preprocess_test` 和 IMU 现状特征测试。按[通用验证说明](validation.md)配置依赖后，以 `nice -n 10`、`-j1` 构建 `fastlivo_mapping` 和 `run_tests`；运行 `catkin_test_results` 并查看每个 GTest XML 的 tests/failures/errors，避免 catkin 汇总重复计数。Python 测试命令：

```bash
cd src/livo-atlas
/usr/bin/python3 -m unittest discover -s tests -p 'test_*.py' -v
```

本机证据目录为 `validation/P1-03-noetic-20261018/`。转换测试使用真实 ROS 消息、cv_bridge 和 PCL，不使用消息转换 mock。新入口的空实现先得到数值/内容断言失败，再替换成实际实现；编译错误未计为行为 RED。

## 回放方法

按[独立副本方法](validation.md)分别用 git archive 创建参考与候选源目录，每个目录链接同一份未修改的 rpg_vikit，再单独构建。不要在开发源目录直接回放：现有 ROOT_DIR 日志仍在 P2 的迁移范围。

每次输出使用不存在的新目录。配对时两边都设置以下环境并依次运行；不能只设置 OMP_NUM_THREADS，因为原算法会调用 omp_set_num_threads。

```bash
OMP_THREAD_LIMIT=1 OMP_DISPLAY_ENV=TRUE /usr/bin/python3 \
  src/livo-atlas/tools/run_baseline.py \
  --workspace validation/p103-reference \
  --bag Retail_Street.bag --output validation/p103-reference-short \
  --duration 30 --rate 0.5 --port 11381
```

对候选使用同样参数、不同输出/端口；短段通过后把 duration 改为 136 覆盖完整 bag，再运行候选默认并行 45 秒。比较轨迹时把位置/姿态阈值都设为 0；另对 mat_pre/mat_out 做有限值、shape 和 array_equal 检查。保存输入计数、更新计数、源、bag、可执行文件及所有包内动态库摘要。不能只校验主程序，因为算法大多位于共享库。

真实启动检查继续使用 `tools/check_calibration_startup.py` 的全部案例；该检查需启动本机独立 loopback ROS master。构建、启动检查与 bag 回放分别执行。

## 范围

本阶段不能证明整个前端已无 ROS。同步、发布、ROS 参数和旧日志输出仍在 mapper/旧算法实现中；IMU 实现仍使用 ROS 日志，但输入测量不含 ROS 类型。预处理已经可单独无 ROS 构建。相机仍通过已验证的临时工厂使用 vikit。

现有 bag 只覆盖 Livox + 相机 + IMU、非特征点云分支；其他雷达有合成输入直接测试，不等同于实际设备/完整数据集验证。原特征提取和驱动单位约定留待使用对应数据时单独验证，不能由 Livox 回放替代。Humble、ARM、1 倍实时性能和真值定位精度不在本阶段的验证结果中。
