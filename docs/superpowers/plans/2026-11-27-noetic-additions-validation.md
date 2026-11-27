# Noetic Stratified Additions Validation Plan

> **For agentic workers:** Execute inline in the current workspace.

**Goal:** Validate end-to-end behavior of flat and stratified additions on one complete Noetic recording.

**Architecture:** Reuse the known Noetic node, current model worker, existing timing and final snapshot tools.

**Tech Stack:** Noetic, Python, NumPy, CUDA and the existing SDK.

**Spec:** docs/superpowers/specs/2026-11-27-noetic-additions-design.md

## Global Constraints

Fixed source and input, only worker seed_distribution varies. Keep 1 GiB free and at most 4/7.5 GiB
for two/four runs. No modifications to product algorithms or historical data. Run GPU jobs sequentially.

## Review Focus

- Both comparisons use the same bag, native build, declared boot order and recording period.
- Repeat decision uses frozen historical P95 spreads and sampled online metrics, before judging quality.
- Poses, loop IDs and original sensor state must remain comparable. Holdout labels stay excluded.
- Verify model assets, online terminal states and raw final scoring samples on matching views.
- Compare coverage turnover with the corresponding flat baseline, and use a common mask for texture and depth.

### Task 1: Run and verify the complete replay

**Files:** local validation/noetic-additions-20261127/run_study.py, protocol.json and logs.
**Interfaces:** current tools/run_baseline.py, existing worker-measure.py, evaluate_heldout_snapshot.py.

- [x] Freeze build inputs, bag, split, typed worker config, launch flags, historic timing bands and code digests.
- [x] Replay flat then additions, checking runtime evidence and the repeat decision. If triggered, run additions then flat.
- [x] Score each admitted final model through the existing evaluation entry. Match the frozen frame set
      and persist child exit and hash receipts.

### Task 2: Audit and document

**Files:** local audit_results.py, results and verification evidence, docs/noetic-stratified-additions.md,
docs/stratified-additions.md and docs/engineering-roadmap.md.

- [x] Recheck independent payload hashes, timing samples, frontiers, loop evidence and split discipline.
- [x] Compare on the shared masks and derive pooled error statistics and coverage turnover.
- [x] Record all repeats separately, describe the tradeoffs and reproducible entry points, verify evidence and commit locally.
