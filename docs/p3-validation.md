# P3：移除 vikit 与独立核心

本阶段只修改 livo-atlas，`rpg_vikit` 保持
`6c886c8e5d83997806e00294826d528cea3581dd` 且工作树干净。参考为 P2-03 的
`752a3be256b4560ce537a09077f2b419b0b584b1`；冻结的源码、运行库、依赖版本、
bag 和输出均通过只读摘要核验。当前产品实现提交为 `53e52d7`，含整阶段评审修复。

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
| Noetic 普通 GTest | 150 项通过，15 个测试程序 |
| 独立核心 shared / static | 各 130 项通过，使用同一普通测试清单 |
| 公开普通头独立包含 | 23 个通过 |
| shared / static 安装后移动前缀 | 外部工程成功 find_package，连续 LO/LIO/带跟踪点 VIO 通过 |
| 独立 catkin shared / static 消费 | 标定校验与连续 LO/LIO/VIO 通过，未借用原工程目标 |
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
采样峰值 RSS：控制组完整窗口 822.62 MiB，默认线程窗口 402.46 MiB。
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

整阶段使用一个 gpt-6-astra 新上下文进行只读评审，原结论为需修复两项 Important，
无 Critical/Minor。执行者另将共享 Frame 编号竞争提升为 Important，三项统一修复：

1. COLMAP 标定在任何文件打开/清理前完整校验。真实坏标定回归验证既有日志和
   reconstruction 标记文件保持不变，非有限/溢出配置被拒绝。
2. catkin 导出补全标定、估计器和运行库依赖，并按静态链接顺序排列。独立项目先复现
   shared/static 链接失败，修复后实际运行校验与 LO/LIO/VIO。
3. Frame 全局编号改用原子递增。8 线程构造 160,000 帧的用例先观察到重复编号，
   修复后编号唯一。该用例不替代整个前端的完整并发验证。

没有派发第二次评审。所有列出的未评判行为均已在执行记录中逐项裁定；不增加
鱼眼数学修正、非法 FrontendOptions 的全面防护、跨传感器精度保证、掉电原子存储
或同目录多 writer 支持。Frontend 调用者仍须满足公开头声明的标定/图像/网格条件。
未延期任何 Minor。修复后的构建、独立调用、ROS/启动、生命周期和 gdb 已通过；
最终代码的短/136 秒/导出回放再次精确匹配参考，默认线程回放正常结束。

初次评审所核验的原始回放、XML、库和摘要保留在包外 `pre-review/`，最终结果使用
顶层同名场景及 `review-fix-*`、`review-life-*`；不会改写 P2-03 冻结参考。

## 已裁定的取舍及代价

以下顺序与执行记录一致，包含评审中明确未判定的每个行为：

| 决定 | 依据与代价 |
| --- | --- |
| 沿用现有开发分支 | 用户授权直接实施；若需额外隔离，要从阶段提交另建分支。 |
| 保留旧相机特殊分支，删除未调用金字塔 | 保持依赖迁移可比较；模型修正和金字塔须另行实现、验证。 |
| native 数值样本采用 4 ULP | 有独立 FMA 对照；跨平台超限须重查，bag/文件门槛仍精确。 |
| 默认 Livox 雷达类型并补初始化 | 已复现 LO 存储复用错误；其他传感器仍需自己的实测。 |
| 评审重申光轴/近轴及首系数开关契约 | 本阶段维持原行为；仍需后续单独纠正模型。 |
| Frontend 由调用者满足配置前置条件 | 避免新增完整配置框架；违规输入仍可能除零或越界。 |
| 修复 Frame 编号竞争，不声明完整并发安全 | 原子递增及唯一性回归已通过；全前端并发仍需额外验证。 |
| 不推定其他传感器的物理精度 | 当前官方 bag 为 Livox；其他家族须单独回归。 |
| 不增加掉电事务或同目录多 writer | 保持既定流/目录契约；掉电可留下部分输出，同目录写入可覆盖。 |

## 恢复交付

完整历史包位于工作区 `livo-atlas-p3-20261024.bundle`。对应 HEAD、包大小、
SHA-256 和空 bare 仓库恢复结果记录在包外 `completion.json`、`recovery.json`，
以避免文档提交与自身 bundle 摘要相互依赖。恢复代码后，按 `core-build.md`
重新构建，不复用编译缓存；bag 和包外验证证据另行复制。

```bash
git clone -b refactor-fast_livo2 /实际路径/livo-atlas-p3-20261024.bundle livo-atlas
```

当前阶段为 P3 完成；后续 P4 再接入 ROS2 Humble。开发分支与工作目录保留，
没有合并或推送。
