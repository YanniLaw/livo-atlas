# Spatial Heldout Comparison Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline with one final fresh read-only review.

**Goal:** 共用空间选块，完成完整Noetic bag的uniform/coverage配对验收。
**Architecture:** 共用C++投影/排序内核，外围分别验证在线/最终来源；离线用小型CLI读取有界JSON。
**Tech Stack:** C++17/Eigen/OpenCV，Python/NumPy/Torch/gsplat，Noetic，Humble构建回归。
**Spec:** ../specs/2026-11-26-spatial-heldout-comparison-design.md

## Global Constraints

仅livo-atlas；Google C++；函数实现空一行；无Pimpl/新依赖；同一worker；不改算法默认预算。
本地framework_completion提交，无推送/合并；不复制bag、不改旧验证资产。

## Review Focus

- 在线提取后要保持所有投影边界/排序/溢出结果，验证已有SDK空间测试及跨工具结果。
- 历史owner不能选未来组；missing owner元数据不能回退或伪装成功。
- 工具/输入变化不能发布有效报告；JSON/子进程输出与磁盘必须有界。
- online不同as_of不可混作相同固定训练模型；配对必须绑定完整manifest/共同查询。
- 最终spatial双块预算不能低估；失败/取消查询须保留完整身份和状态。

## Task 1：测试收尾

- [x] 新增只改变高度的协议用例，确认原始socket双方拒绝。
- [x] 实际双块CUDA推理覆盖在线报告joint字段、单/双模型数量与训练RNG不变。
- [x] 定向验证；运行现有相关测试。

## Task 2：共享选择内核与离线工具

**Files:** include/livo_atlas/gaussian/gaussian_projection.h, src/gaussian/gaussian_projection.cc,
src/gaussian/gaussian_render.cc, tools/select_gaussian_neighbor.cc, cmake/online_gaussian.cmake。
**Interfaces:** GaussianSpatialCandidate{id,extents,parent_from_submap}; GaussianSpatialSelection{id?,projection,count};
SelectGaussianSpatialNeighbor(candidates,camera,parent_from_body,owner)；结果语义与原选择器完全一致。

- [x] RED：调用不存在的公共函数/工具验证选邻组、历史过滤、无候选和非法预算。
- [x] 提取现有投影/CoordinateBound/排序，在线包装仍做Usable/ValidateBinding。
- [x] CLI有界输入/输出与严格摄像机解析，复用元数据decoder；构建/安装该工具。
- [x] GREEN并运行SDK全部测试，原空间回归必须不变。

## Task 3：最终评分接入

**Files:** tools/3dgs_probe/heldout_selection.py（新）, evaluate_heldout_snapshot.py,
tests/3dgs_probe/test_heldout_selection.py（新）, test_evaluate_heldout_snapshot.py。
**Interfaces:** 读取已验证snapshot+frozen query，按config policy产生selected_ids/selection证据。

- [x] RED：spatial config、非相邻选择、owner缺元数据、helper失败/变更不发布有效报告。
- [x] 复用C++工具、绑定helper SHA，预估spatial两块最坏输出；temporal对齐直接前组。
- [x] 报告记录各查询选择依据/工具/政策；保留只读及原有失败完整性。
- [x] 全Python/CUDA、SDK与双ROS构建回归。

## Task 4：完整配对与交付

- [x] 复用完整参考帧冻结172帧manifest，确认查询数与磁盘预算。
- [x] Noetic uniform/coverage各两次完整135.47秒训练+在线评分+Finalize，依次使用GPU。
- [x] 同一配置spatial最终只读评分，复算摘要/指标/lineage/前端文件；比较共同查询。
- [x] 汇总质量与资源/完整性；更新路线图过期描述和复现文档。
- [x] 一次独立审查、重要问题RED/GREEN修复、本地提交。
