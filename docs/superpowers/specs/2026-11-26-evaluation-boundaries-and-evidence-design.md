# 留出协议边界与运行证据补全

承接 c8cdc27 保留的两项小优化，用户已授权继续实施并在本地提交。

## 方案与范围

直接沿用现有查询协议、worker和报告，不增加协议版本、配置项或线程。
另起评估服务或改变指标会扩大范围；本次不采用。选择现有接口内补齐校验与报告。

- 查询刚体变换的齐次行在 C++ / Python 端统一采用绝对误差不超过 1e-9，
  与已有普通位姿协议一致；SO(3)校验和冻结相机身份检查保持现有要求。
- spatial 查询的 width、height分别等于 owner保留相机尺寸，不能仅比较像素总数。
  继续允许独立 heldout相机，原有preview最后训练相机限制保持。
- 每份成功的在线报告添加 software（worker握手软件信息与NumPy版本）及 inference。
  inference.single / joint记录各次推理的timings、Gaussian数量和CUDA分配/保留峰值。
  没有联合推理时joint为null。memory_peak_scope="worker_lifetime"，明确累计峰值，
  不重置分配器、不统计成单查询增量，不修改训练RNG/模型/优化器。
- 复用infer_blocks已返回的数值，新增字段计入原有2MiB报告与磁盘预算。

## 验证

跨语言实测边界内/外齐次行、同像素数不同宽高、不同像素数相机；使用原始socket
数据同时检查C++和Python验收结果。真实CUDA评分检查报告字段及训练状态/RNG不变，
覆盖单块/双块报告和原有OOM/预算失败。运行SDK及完整Python/CUDA回归，双ROS编译。
不为仅增加校验与元数据重复生成大型bag资产。
