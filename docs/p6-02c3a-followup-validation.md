# C3a 验收器版本顺序补验

2026-11-15，起点 `9a31ab4`。上轮独立审阅的唯一延期 Minor 已关闭。
本轮调整包外验收脚本，C++、ROS 节点和原始回放数据沿用已验收版本。

## 问题与修复

旧脚本逐样本检查地图修正是否匹配任一最终正式版本，没有约束这些解释的顺序。
因此，时间递增但修正唯一对应 v1→v0 或 v2→v1 的损坏记录可能通过。

新版在匹配集合中选择不小于前一个解释的最小版本；不存在可行版本时立即拒绝，
且不写入通过结果。选择最小可行值会保留最多的后续可能性，不会因过早选择较大版本误拒绝有效记录。
仍保留完整的匹配集合，并在结果中记录 `minimal_version_explanation_runs`。
这些运行段属于一种可行解释，不能当作 ROS 实际传输了精确版本号。

同值修正仍可解释为不同版本。例如，v0 与 v2 相同，观测到 I→A→I 时，
允许 0→1→2 的解释；v1 与 v3 相同，A→B 时允许 1→2，不强行先选 v3。

## 文件与复验

新版及所有证据位于工作区：

```text
validation/P6-02c3a-followup-20261115/
├── baseline/check_global_outputs.py
├── check_global_outputs.py
├── test_global_version_order.py
├── check_global_synthetic.py
├── recheck_recordings.py
├── recheck-summary.json
├── rechecked/<recording>/global-checks.json
└── sources.json
```

上一阶段 `P6-02c3a-20261114` 的脚本、正式回放与旧判定结果保留为历史证据。
本轮结果写入新目录，输入通过只读使用方式关联旧记录；`sources.json` 记录新旧脚本 SHA256。
迁移电脑时按项目惯例单独复制包外 validation 证据目录。

```bash
python3 -m unittest discover -v \
  -s validation/P6-02c3a-followup-20261115 -p 'test_*.py'
python3 validation/P6-02c3a-followup-20261115/check_global_synthetic.py
python3 validation/P6-02c3a-followup-20261115/recheck_recordings.py
```

`recheck_recordings.py` 要求新的结果目录，拒绝覆盖已有结果。
对日后新回放可直接向新版 `check_global_outputs.py` 传入包含 `global-output.json` 与 `keyframes/` 的目录；
该目录须尚无 `global-checks.json`。目前该工具检查成对 TF 录制，publish_tf=false 继续由实际 ROS 测试覆盖。

## 结果

- 新增 5 个顺序/歧义用例：先观察到旧实现的两种回退漏检及缺失解释结果，再修复后全部通过。
- 原有非零变换、错误位姿、第二父边和错误配对时间 4 个合成用例全部通过。
- Noetic 宿主 Python 3.8 与 Humble Python 3.10 均通过新增 5 个用例。
- 原始 12 份实际回放记录全部通过新版复核；六份 global ON 记录均存在非递减解释，完整记录为 0→1→2。
- 完整原生 55 个程序通过；宿主 Python 工具集 22 项通过、既有 Python<3.10 迁移模块跳过，Humble 全部 27 项通过。

原始记录的位姿、TF、时钟、传感器输入和二进制没有重采集或替换。本轮关闭验收器缺口，
不改变 [C3a 当前位姿契约](global-pose-output.md)及原有三项实施裁定。
