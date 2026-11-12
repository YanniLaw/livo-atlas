# 几何回环候选验证

C1 提供可选的普通 C++ 几何验证库和只读离线工具。输入为同一位姿视图的封存体素子地图，
输出候选相对位姿、来源修订和质量诊断。它使用里程计邻域初值，适用于先验仍足够接近的场景。
候选通过几何检查后还需要后续 C2 的一致性与图准入；当前工具不提交位姿更新。

各平台的实际验证与限制见 [C1 验证记录](p6-02c1-validation.md)。

## 构建与安装

默认 `LIVO_ATLAS_WITH_LOOP_CLOSURE=OFF`，开启后导出 `livo_atlas::loop_closure`。
算法库不暴露 ROS/GICP/GTSAM 类型，GTSAM 开关独立。已有 PCL 安装需包含 registration 和 kdtree。

```bash
cmake -S /path/to/livo-atlas -B /path/to/build \
  -DLIVO_ATLAS_ROS_VERSION=0 -DLIVO_ATLAS_WITH_LOOP_CLOSURE=ON \
  -DLIVO_ATLAS_WITH_GTSAM=OFF -DCMAKE_BUILD_TYPE=Release \
  -DSophus_DIR=/path/to/compatible-sophus-config \
  -DCMAKE_INSTALL_PREFIX=/path/to/install
nice -n 10 cmake --build /path/to/build -- -j1
(cd /path/to/build && OMP_THREAD_LIMIT=1 ctest --output-on-failure)
cmake --install /path/to/build
```

可追加 `-DBUILD_SHARED_LIBS=OFF` 验证静态工程库。Noetic/Humble 构建使用同一选项；
工作空间准备和运行方式见[双 ROS 说明](ros-builds.md)。开启验证库不会自动改变 ROS 节点流程。

## 验证已结束的数据集

DATASET 指向含 `dataset.yml`、`frames/` 与 `online_submaps/` 的关键帧目录。
在线成员索引必须完整、全部封存；工具当前只支持显式选择原始版本 0。
输出目录必须不存在，且不能放在输入数据集内部。

```bash
OMP_THREAD_LIMIT=1 nice -n 10 /path/to/install/bin/livo_atlas_verify_loops \
  /path/to/keyframes /path/to/new-results --pose-version 0
```

工具先读取元信息检索候选，再从现有 GeometryMapper 获取主成员体素几何；不读取图像。
每个候选依次处理并写盘，不在内存保存全部历史云或全部配准结果。
退出码 0 表示遍历完成，包括零候选和全拒绝；参数语法错误返回 2，运行失败返回 1。

可通过 `--config FILE` 指定 YAML。省略字段沿用默认值，未知键、非数值和非法整数会被拒绝。
配置分 `search` 与 `registration` 两组，支持公共选项全部字段，默认值见
[公共头](../include/livo_atlas/backend/loop_closure.h)和[设计](superpowers/specs/2026-11-12-p6-02c-loop-closure-design.md)。
例如：

```yaml
%YAML:1.0
---
search:
  search_radius_m: 15.0
  max_candidates: 5
registration:
  max_iterations: 50
  match_distance_m: 0.5
```

工具固定以 0.2 m 几何体素和最多 3 块既有缓存重建局部几何；其平面参数记录在摘要中。
配准临时云默认再以 0.5 m 降采样，质量诊断使用原几何质心及其有效法向。
几何缓存上限不包含被外部共享引用固定的数据或 PCL 临时状态，不是进程 RSS 上限。

为保护 PCL 的 float 乘积和平方距离，原始几何坐标及双方向初值变换后的点按分量限制在
`sqrt(FLT_MAX)/16` 的保守运算包络内；超出时显式拒绝。每次 GICP 更新也检查后续查询是否
仍在该包络内。这个上限只用于数值保护，不能解释为该尺度下仍有足够配准精度。

## 输出与判定

- `summary.yml`：schema、`complete`、来源路径、帧和子地图数量、实际全部选项、状态计数和固定引用体素峰值。
- `matches/TARGET-SOURCE.yml`：每对候选的两端 ID/修订、位姿版本、初值、正反求解、独立质量和耗时。
- `failure.yml`：运行中途失败时的原因。保留已有记录，重试必须换用新的输出目录。

只有摘要 `complete=1` 才表示遍历完成。中途失败的记录可以检查，但不能当成一次完整结果。
尚未执行的方向或质量指标明确缺失，不用零数值冒充。只有 `status=verified` 才写
`target_from_source`，将 SOURCE 局部点变换到 TARGET 局部系；两端局部系分别是其锚点 body 系。

`verified` 需要两方向求解未到上限，并通过互为最近邻覆盖、残差、双端闭合、初值修正及退化检查。
稳定的拒绝原因包括 `insufficient_points`、`resource_limit`、`not_converged`、`iteration_limit`、
`low_overlap`、`high_residual`、`inconsistent_transform`、`large_correction` 和 `degenerate`。

阈值是待标定的实验起点。重复结构可能产生一致但错误的配准，无标注数据不能仅凭通过数报告
闭环准确率；本阶段不具备大漂移全局检索、在线图接纳、全局 TF 或在线 Gaussian mapping。
