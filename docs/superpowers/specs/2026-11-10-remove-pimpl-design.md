# 移除 Pimpl，恢复直接可读的类结构

## 决策与范围

2026-11-10 用户明确选择：当前 8 处 Pimpl 全部移除，包括 Frontend 和 LivoExporter。
目标是减少源码跳转，让类的状态、组件关系及方法实现直接可见。
此前“部分保留”的建议不采用。本次只整理类结构，不扩展 SLAM 或高斯功能。

涉及 LivoSystem、Frontend、CloudColorizer、LivoExporter，及 ROS1/ROS2 各自的
LivoNode、LivoPublisher；原路径和公开函数签名保持。实施计划见
[移除 Pimpl 计划](../plans/2026-11-10-remove-pimpl.md)。

## 实现约定

- 头文件声明普通类、private 方法及成员，`.cc` 直接定义 `ClassName::Method`。
  删除 Impl 声明、Impl 类、impl_ 指针及纯转发函数；不换名为 Data/State/Context 继续包装全部实现。
- 保留真实组件的所有权：Camera、ImuProcessor、VIOManager、VoxelMapManager、各 Store
  和 OnlineSubmapManager 可以继续由 unique_ptr 拥有；这与 Pimpl 转发层不同。
- 使用前置声明减少不必要包含；值成员需要完整类型时直接包含对应头，不为了缩短头文件再造包装类。
  普通核心头不得依赖 ROS；ROS 头只存在于各自接入层。
- Frontend 保持原成员顺序与销毁约束：视觉对象停止借用后才删除体素根节点，相机晚于视觉对象释放。
- LivoSystem 保持原组件声明及构造顺序，OnlineSubmapManager 仍是私有模块，通过前置声明及
  `.cc` 中的析构定义避免把 `src/` 私有头引入安装接口。
- LivoExporter 的 TextFile 作为有明确文件资源含义的 private 值结构保留，直接含路径和 ofstream；
  Check/Open/Close 改为对应 private static 方法，保留错误路径、checked-close 和失败后拒绝写入的行为。
- ROS 两端把订阅、定时器、发布器直接作为成员，成员回调捕获实际节点 this；
  声明顺序保证订阅/定时器先于其借用对象销毁，不引入线程、继承层或公共回调接口。

## 兼容与验收

算法公式、初始化参数、同步、发布、导出、在线提交顺序及异常语义保持。
特别保留 RawSaveFailureBlocksOnlineMutations 对应的系统级错误保护，不能在删除转发函数时丢失。
保留原删除复制及现有移动约束，不新增对象移动能力；用户指定的六个类型名保持。

公开类对象布局会改变，相关库、节点和下游消费者全部重新编译；不承诺旧二进制直接混用新库。
回归参考保存整套旧运行库，候选使用整套新运行库，不能混合两种布局。
验收包括已有行为测试、单头编译、shared/static 安装消费、真实 Noetic/Humble 接入及同环境 bag 配对。
纯结构整理优先复用现有测试，不新增只断言成员布局或模仿实现的测试。

## 执行边界

Google C++17，无新增依赖、物理库、算法接口或数据格式。只改 livo-atlas，保留已有路线文档改动。
验证新建包外目录，串行 `-j1`、nice 10、`OMP_THREAD_LIMIT=1`，不用 RViz；旧证据及 bag 不修改。
用户已指定当前会话直接实施。遵守其先通知再切换模型的要求：当前只落文档，待用户切换后继续正式编码。
