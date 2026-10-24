# P3：移除 vikit 与独立核心

本阶段只修改 livo-atlas，`rpg_vikit` 保持
`6c886c8e5d83997806e00294826d528cea3581dd` 且工作树干净。参考为 P2-03 的
`752a3be256b4560ce537a09077f2b419b0b584b1`；冻结的源码、运行库、依赖版本、
bag 和输出均通过只读摘要核验。当前实现提交为 `e8c1493`。

设计和实施步骤见 [P3 设计](superpowers/specs/2026-10-24-p3-independent-core-design.md)
与[执行计划](superpowers/plans/2026-10-24-p3-independent-core.md)。
跨电脑使用见[独立核心与 Noetic 构建](core-build.md)。

## 实现边界

- 新的 `Camera` 接口及 Pinhole/Equidistant 实现只依赖普通 C++、Eigen、OpenCV。
  替换 Frame、VIO、Frontend 和 OutputProcessor 的调用，移除临时 vikit 工厂。
- 灰度双线性插值和 Shi–Tomasi 评分保留表达式类型与顺序，支持非连续图像。
  未使用的金字塔入口直接删除，没有移植未调用的 SSE/NEON 实现。
- `livo_atlas::core` 聚合前端、同步和预处理；camera/output/io 可独立链接。
  mode 0 不查找 catkin，mode 1 只在外围加载 Noetic 接入。核心编译的头文件路径
  和动态依赖均不含 ROS/vikit；安装配置不引用本包源码或构建目录。
- 算法中的墙上计时、控制台表格和仅用于统计的字段移除，实际 OpenMP 循环保留。
  成功更新的整体 LIO/VIO 计时由 ROS1 Mapper 各打印一次。
- Writer 的真实写入/关闭错误包含具体文件路径，失败后保持失败状态。
  双 writer 的 PCD/图像保存间隔交错测试补齐，原有实例计数行为正确。
- 高频 IMU 传播游标改为 Mapper 成员；两实例同样输入均得到 0.125 m 位移。
  另修复遗留 IMU 雷达类型未初始化：复用深度相机对象的存储后，LO 中间点现在
  正确从 2 修正到 1.9375 m，不再依赖旧内存内容。

没有加入 ROS2、GTSAM 或 3DGS 实现。ROS2 Humble 属于 P4；当前 mode 2 明确报错。

## 实际验证

本机 Ubuntu/ROS1 Noetic、GCC 9.4、CMake 3.16、Eigen 3.3.7、PCL 1.10、OpenCV 4.2，
使用原工程兼容版 Sophus。构建均 `nice -n 10`、`-j1`，与 bag 回放串行，无 RViz。

| 检查 | 实际结果 |
| --- | --- |
| Noetic 普通 GTest | 147 项通过，15 个测试程序 |
| 独立核心 shared / static | 各 127 项通过，使用同一普通测试清单 |
| 公开普通头独立包含 | 23 个通过 |
| shared / static 安装后移动前缀 | 外部工程成功 find_package，连续 LO/LIO/带跟踪点 VIO 通过 |
| 关闭 OpenMP | 全部普通库编译、安装迁移消费及三种前端模式通过；未运行整套 GTest |
| 真实 ROS | 12 项通过，含双 Mapper 传播回归 |
| 启动检查 | 50 项通过 |
| Python | 13 项通过（在包目录运行） |
| gdb 3 秒 | 26 个位姿，正常退出 |

Noetic 模式的 catkin 会向目录注入三个 ROS 编译宏（package/shared/backend 标记），
核心代码不使用它们，也不借用 ROS 头或链接库。mode 0 的编译没有这些宏。

Writer 的两项真实故障和双 Mapper 回归先在旧实现观察到行为失败，再通过修复。
新增 LO 存储复用测试也先得到错误坐标 2，再验证正确值 1.9375。

相机 golden 来自固定旧库：1,140 组投影/反投影，194 组图像操作，及去畸变图像摘要。
旧源码以默认 ISA 编译时与新实现逐值一致，native 关闭 FMA 后也一致；冻结 native
版本的相机最多差 3 double ULP，评分最多差 2 float ULP。因此这些单元样本使用
GTest 的 4 ULP 比较；去畸变图像和 bag/导出比较继续要求精确一致。
详见[样本说明](../tests/camera/data/README.md)与包外 `numeric-control.json`。

生命周期探针实际构造并销毁四个含地图的 Frontend。无地图/有地图无抑制检查均报告
TBB 的 12,312 字节、3 笔分配；仅保留该已确认的 TBB 抑制后，应用检查退出 0。
旧的 VTK/PCL 抑制已删除。这是 ASan/LSan 运行时链接探针，算法 DSO 没有完整插桩，
不把它描述为完整 ASan 检查。

## Bag 对照

`Retail_Street.bag` SHA-256：
`1745bee1d570749679bad7a0392100477ab62d50aa77f7f629bc57f2a72e4427`。

回放参考为冻结的 P2-03 候选，控制组设置 `OMP_THREAD_LIMIT=1`、rate 0.5。
比较 `trajectory.tum`、`mat_pre.txt`、`mat_out.txt`、`imu.txt` 的全部导出值，
以及点云声明字段/图像像素的语义摘要。忽略 ROS 接收时钟、序号和 PCL padding，
不会用浮点单元样本的 ULP 容差放宽这些门槛。

| 场景 | 位姿数 | 当前结果 |
| --- | --- | --- |
| 控制组 30 秒 | 296 | 四组数组、消息有效内容精确一致 |
| 控制组 136 秒 | 1,351 | 四组数组、消息有效内容精确一致，正常退出 |
| 默认线程 45 秒 | 446 | 446 次 LIO、446 次 VIO，输入计数正确，正常退出；不作精确数值声明 |
| 导出场景 3 秒 | 26 | 四组数组、消息精确一致，57 个导出文件逐字节一致 |

136 秒窗口的四组数组分别为 1351×8、2707×19、2701×20、27365×7，全部有限，
与参考的最大绝对差均为 0；点云消息 2,701 条，图像消息 1,350 条。
采样峰值 RSS：控制组完整窗口 817.94 MiB，默认线程窗口 397.32 MiB。
源目录的 31 个 Log 文件在所有回放后摘要不变。

## 证据和复现

包外证据目录为 `validation/P3-noetic-20261024/`，包含原始 XML、日志、采样器源码、
数值对照、安装消费记录、运行依赖及阶段执行记录；迁移时单独复制。
`golden-generator.CMakeLists.txt` 与采样时记录的 SHA-256 一致，保留了初始采集器
构建入口；后续控制实验使用另外的目标。

普通构建、安装和 Noetic 用法已写入 [core-build.md](core-build.md)。
验证入口为包内 `tools/run_baseline.py`；本次串行调度与严格比较脚本在包外
`run_validation.py`、`check_sdk.py`、`audit_builds.py`。不要复用另一台电脑的
build/devel/CMakeCache；在目标平台重新构建依赖与本包。

整阶段独立评审和恢复包校验尚待最终登记；以上回放已完成。
