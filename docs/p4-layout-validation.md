# P4 修正：统一入口与职责目录验证

状态：统一入口、文件迁移、构建/单元/启动/安装与两套完整 bag 回归已通过，独立评审与交付收尾中。
参考为 P4 `5884f15728e3a1dfd394beaa1f3a595db87d35af`。
设计见[修正规格](superpowers/specs/2026-10-24-p4-unified-entry-layout-design.md)，
步骤见[实施计划](superpowers/plans/2026-10-24-p4-unified-entry-layout.md)，
当前路径和 include 迁移表见[源码组织](core-build.md#源码组织与-include-迁移)。

## 实现与迁移

- `src/app/mapping_main.cc` 是两个 ROS 版本的同一实际入口；公共 `cmake/node.cmake`
  创建一次 `fastlivo_mapping`，目标私有宏选择 ROS1/ROS2 的初始化、适配器、executor 和退出。
- IMU、VIO/frame、体素地图/视觉地图点、预处理按职责归位；LIVMapper 归 ROS1。
  旧 common/utils 集中到 `internal/legacy`，生成的 Livox ROS1 消息头保持原标准命名空间。
- 7个源文件、12个头文件与 P4 原路径逐一归一化比较，除 include/guard 外正文一致。
  类名、库目标和估计数学保持；根目录旧头没有转发层，源码消费者按迁移表更新 include。
- 新公共入口按 clang-format14 Google 风格排版；原算法文件保持正文，避免把格式化混入数值迁移。

## 已通过的检查

| 检查 | 实测 |
|---|---|
| Noetic / Humble 编译输入 | 同一入口 TU、节点私有版本宏、普通核心无 ROS include/macro |
| Noetic C++ / 真实 ROS | 158 / 12 项 |
| Humble C++ | 176 项 |
| 最终 Noetic / Humble 实际启动 | 50 / 16 项 |
| Python | Noetic13项通过、1个ROS2模块按Python3.8条件跳过；Humble5项通过 |
| 普通头自包含编译 | 两种 ROS 和 mode0 shared/static 各36个 |
| mode0 shared / static | 各138项；移动安装目录后消费者推进 LO/LIO/连续VIO |
| mode0 关闭 OpenMP | 自身 OpenMP 编译选项/并行宏关闭；移动安装目录后消费者推进 LO/LIO/连续VIO |
| Humble ament 安装 | 新头路径、移动安装前缀、普通消费者无 ROS 运行依赖 |
| Noetic DESTDIR 安装 | canonical mapper 头安装正确、无旧根头 |
| Humble colcon | 真实安装节点构建通过，用于后续 DDS 回放 |

mode0 真实编译输入没有节点，也没有 ROS 版本宏。Humble 与 Noetic 平台/依赖沿用
[P4 环境](p4-validation.md#环境与固定输入)；没有借跨环境容差放宽回归。

## 官方 bag 回归

输入仍为同一个 `Retail_Street.bag`；Humble 读取已核验的 P4 rosbag2 转换，原输入只读。
两种接入分别对照各自 P4 冻结环境结果，`OMP_THREAD_LIMIT=1`、回放速度0.5。
构建已完成，回放串行，不启动 RViz。

| 场景 | LIO / VIO / 位姿 | 同环境严格比较 |
|---|---|---|
| Noetic 3s产品 | 26 / 25 / 26 | 四数组、发布语义及57个产品一致 |
| Noetic 完整136s窗口 | 1351 / 1350 / 1351 | 四数组及发布语义一致 |
| Humble 3s产品 | 26 / 25 / 26 | 四数组、发布语义及57个产品一致 |
| Humble 完整136s窗口 | 1351 / 1350 / 1351 | 四数组及发布语义一致 |

## 本机证据

工作区 `validation/P4-layout-20261024/` 保存新证据，旧 `P4-humble-20261024` 只读。
`entry-red.json` 与 `entry-checks.json` 保存真实编译输入的前后对照；
`source-moves.json` 保存19个迁移文件的正文比较；`sdk-*-checks.json`、
`humble-mode2-installed.json`、`noetic-install.json` 保存安装检查。
每次 bag 回放分别绑定14个实际程序/库的前后 SHA，包含输入计数、正常退出和有限数值检查。
`acceptance.json` 已通过：31个原源码 Log 文件和 rpg_vikit 的提交/工作树保持原样。
两套完整回放的数组维度均为1351×8、2707×19、2701×20、27365×7，
对各自 P4 参考的最大绝对差均为0，发布语义也严格相等。Humble 保持全部30157条输入
的外部观察者字节/计数及节点接收计数核验，正常退出。各自短段57个产品逐字节相等。

最终评审与完整历史 bundle 恢复记录在交付收尾完成后补充。
