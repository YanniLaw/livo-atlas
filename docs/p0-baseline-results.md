# P0 基线结果与退出缺陷

日期：2026-09-27；P0-03～P0-08 依据用户授权自动验收。结论：核查已执行，阶段未通过；P1 尚未开始。

**2026-10-01后续状态：** P0-F01已修复并完成5秒及两轮30秒回归，P0在约定范围内通过。当前基线改用 `validation/P0-F01/short_a`、`short_b`，详见[修复验收](acceptance/P0-F01.md)。下文保留2026-09-27的原始失败分析和当时结论。

## 输入与复现

基准 commit：`ca556e5ad6706ba20dc8dff91010faae546b7528`。未修改已跟踪的算法、配置和构建文件。环境沿用 P0-01：Ubuntu 20.04 / ROS Noetic；i9-13980HX，32逻辑CPU，约31GiB内存。原始 bag 身份见 [数据登记](test-data.md)。预先固定的范围和门限见 [验证方案](p0-validation-protocol.md)。验证工具由 Luna max 编写，主代理审阅并实际执行。

工作目录 `/home/yanni/livo-atlas_ws`，实际运行命令如下（复验须另用不存在的输出目录及空闲端口）：

```bash
/usr/bin/python3 src/livo-atlas/tools/run_baseline.py --bag /home/yanni/livo-atlas_ws/Retail_Street.bag --output /home/yanni/livo-atlas_ws/validation/P0/short_a --duration 30 --rate 0.5 --port 11331
/usr/bin/python3 src/livo-atlas/tools/run_baseline.py --bag /home/yanni/livo-atlas_ws/Retail_Street.bag --output /home/yanni/livo-atlas_ws/validation/P0/short_b --duration 30 --rate 0.5 --port 11331
/usr/bin/python3 src/livo-atlas/tools/compare_baselines.py validation/P0/short_a validation/P0/short_b
```

工具内部加载 ROS 和工作区环境，独立启动 master；加载 AVIA/相机原配置，开启唯一名称的轨迹，关闭PCD/图像保存、RViz和冗余图像转发，`use_sim_time=false`。运行期间备份源码日志，结束后恢复；本次日志副本保存在各结果目录。运行参数快照为 `params.yaml`。

## 两轮30秒结果

| 指标 | short_a | short_b |
|---|---:|---:|
| 有效轨迹点 / odom / path | 296 / 296 / 296 | 296 / 296 / 296 |
| 非空点云 / RGB图像 | 295 / 295 | 295 / 295 |
| 峰值RSS（字节） | 385683456 | 387686400 |
| 回放墙钟耗时（秒） | 61.0945 | 61.0926 |
| LIO内部耗时均值 / P95（毫秒） | 56.328 / 69.560 | 57.665 / 69.266 |
| VIO内部耗时均值 / P95（毫秒） | 10.039 / 13.226 | 10.125 / 13.289 |
| bag播放器 / 采集器退出码 | 0 / 0 | 0 / 0 |
| mapper退出码 | **-11（SIGSEGV）** | **-11（SIGSEGV）** |

时间戳集合一致，无重复、回退或无效轨迹行；位置差为0。两份轨迹文件 SHA-256 完全相同：`edc943620f88400a4d92701eca54290c9d4602c122eb433ec445b001086f445f`。四元数归一化后反余弦比较产生约3.42e-6度的最大舍入残差。重复性子项满足0.02米/0.1度门限；没有真值，不能推导ATE/RPE精度。

每轮还有296个空点云消息：当前LIVO在LIO和VIO路径均调用发布，不能将空消息计为有效建图输出。初始化日志显示先进行IMU初始化；序列末端还受同步等待限制，输入帧数与轨迹数并非一一对应。末个LIO之后是否完成VIO需要按事件追踪，此处不把计数差称为掉帧。内部计时日志各提取295条，不是端到端延迟或实时性证明。仅保存了末次非空发布点云，未输出全局地图。

**证据解释：** 两轮使用的早期验证工具在结束前写入 `run_status=complete`，没有把析构时的失败计入状态。因此原始 `summary.json` 的该字段错误，原文件保留不改；其 `exit_codes.mapper=-11` 是实际退出证据。工具现已修正退出判定，`comparison.json` 明确 `acceptance.passed=false`。后续不能只读旧 summary 的 complete 字段。

## P0-F01：有数据运行后的退出崩溃

两次30秒回放正常产生输出，发送SIGINT结束时均出现SIGSEGV；额外5秒GDB诊断也复现：

```bash
/usr/bin/python3 src/livo-atlas/tools/run_baseline.py --bag /home/yanni/livo-atlas_ws/Retail_Street.bag --output /home/yanni/livo-atlas_ws/validation/P0/shutdown_debug --duration 5 --rate 0.5 --port 11332 --gdb
```

`validation/P0/shutdown_debug/mapper.log` 第2062行收到SIGSEGV，主线程栈为：

```text
__GI___libc_free(mem=0xf01f1)
boost::detail::sp_counted_impl_p<pcl::PointCloud<pcl::PointXYZINormal>>::dispose()
boost::detail::sp_counted_base::release()
LIVMapper::~LIVMapper()
main()
```

这将故障定位到映射对象析构时释放点云的路径；尚不能据此判定具体哪个点云、最早内存破坏位置或依赖ABI问题。`LIVMapper` 的空析构函数仍会触发成员析构。P0-01 无数据启动正常不能覆盖这一情况。

GDB进程本身返回0不代表被调试程序正常退出，诊断模式不参与性能比较和基线通过判断。该次原始 summary 的 `diagnostic_signal=null` 是早期解析仅识别 `Program received` 而未识别 `Thread 1 ... received` 的缺陷，实际信号以日志为准；解析工具随后修正，原始证据保留。

后续最小修复任务建议：先精确定位点云生命周期或内存破坏原因，仅修改必要源码/构建项；单独记录修复前后差异；完成有数据5秒退出、两轮30秒回放及既定轨迹/资源比较，mapper必须正常返回0。不得通过跳过析构、强制退出或放宽标准掩盖问题。该源码修复尚未实施，不能将P0-F01标为通过。

## 证据与覆盖边界

- `validation/P0/short_a/`、`short_b/`：参数、各进程日志、输出计数、轨迹、资源采样、源码日志副本和末次点云。
- `validation/P0/comparison.json`：重复性、计时、内存、整体失败判定。
- `validation/P0/shutdown_debug/`：5秒诊断及所有线程栈。
- `validation/P0/contracts.json`、`contracts_v2.json`：坐标/内参/相邻时间样本；v2补充往返误差。
- `validation/P0/binaries.sha256`、`cpu.txt`、`memory.txt`：运行产物与机器身份。

未覆盖：完整135.47秒算法回放、LIO/LO及其他设备模式、物理标定精度、真实场景坐标轴标定验证、真值精度、闭环、长期内存增长、3DGS导出正确性。全包结构检查在P0-02完成，不能替代这些运行验证。当前先处理可重复退出故障，不继续扩大回放长度。

恢复方式：本轮只新增文档和验证工具；运行脚本已恢复原有源码日志，仅清理本次唯一名称的临时轨迹。需要撤回时按文件清单处理新增内容，保留 bag、历史日志及本轮失败证据，不执行全仓重置。
