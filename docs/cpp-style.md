# C++ 编码约定

新增和实质修改的代码遵循 [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)。
仓库 `.clang-format` 使用 `BasedOnStyle: Google`；格式化只能检查布局，不能代替所有权、依赖和接口审阅。
当前工程采用 C++17。官方指南当前建议 C++20；本项目暂时维持现有工具链，这是明确记录的兼容性约束，标准升级需单独验证。

## 命名与文件

- 新源文件使用 `.cc`，头文件使用 `.h`；文件名使用 `snake_case`。
- 类型和函数采用 `PascalCase`；变量采用 `snake_case`；类成员加末尾下划线；普通数据结构的公开成员不加末尾下划线。
- 常量采用 `kPascalCase`，新类型放在 `livo_atlas` 命名空间，ROS 适配器可使用 `livo_atlas::ros1` / `ros2`。
- 头文件使用完整项目路径对应的 include guard，独立包含时可以编译，直接包含其使用的声明。
- include 按相关头、系统头、标准库、其他库、工程头的顺序组织，避免依赖间接包含。
- 使用两空格缩进，函数和控制语句的左花括号放在同行；不在公共头写 `using namespace`。

保留的原版文件和过渡兼容入口可能仍有旧命名，例如 `StatesGroup` 的成员、`PointCloudXYZI` 别名及 `resetpose`。
这些名称仅用于已存在的算法消费者；新 API 采用新约定。逐个模块迁移，不做全仓库纯格式化或机械改名，以保持数值变更可追踪。

## 所有权与错误处理

- 数据值优先使用值类型；唯一拥有的组件使用 `std::unique_ptr`；跨线程共享只读数据时才使用明确的共享所有权。
- 兼容旧 PCL 的 `PointCloud::Ptr` 使用其实际指针类型，不直接替换成不兼容的 `std::shared_ptr`。
- 初始化所有成员；对时间、点内偏移、坐标变换和外参数组顺序写出单位与含义。
- 算法对象构造时不读取 ROS 参数或启动线程；ROS适配器负责订阅与定时器。
  文件所有者必须明确：ResultWriter及组合它的MappingSession会按显式OutputOptions创建输出，
  参数和话题必须在此前校验；普通Frontend构造不写文件。
- 输入错误在边界校验并返回带上下文的结果；新核心代码避免通过异常表达正常分支。保留现有第三方异常时由适配器边界处理。
- 非拥有指针明确生命周期；异步结果不得引用会被下一次估计覆盖的状态或图像缓冲区。

## 控制规模

一个类承担一项可说明的职责。优先普通结构和直接调用；有第二个实际实现或实际替换需求时再增加多态接口。
相机模型的多实现是明确需求，适合小接口；当前算法各模块不需要先套上统一插件基类。
ParameterReader已有ROS1/ROS2两个真实实现，接口只包含当前参数所需的类型读取和名称描述；
参数键与默认值集中维护，避免两套清单漂移。MappingSession只组合已有组件；ROS适配器持有其引用。
为 GTSAM/3DGS 保留数据与版本关系，实施时再增加优化器和训练流程。
测试覆盖边界、同步与数值行为，不为简单字段读写或格式选择增加测试。

## 格式检查

在安装了支持 C++17 的 clang-format 的机器上，对本次新增或实质修改的文件执行格式检查；仅调整路径的原算法文件保留正文：

```bash
clang-format -style=Google --dry-run --Werror src/app/mapping_main.cc
git diff --check
```

格式化后仍需构建和运行行为测试。本机宿主环境未安装 clang-format；统一入口阶段使用已有 Humble 容器中的 clang-format14，
对新 `mapping_main.cc` 执行 Google 格式化和只读复核。原算法文件仅迁移路径/include/guard，
未整文件格式化；各阶段具体检查范围以验证报告为准。
