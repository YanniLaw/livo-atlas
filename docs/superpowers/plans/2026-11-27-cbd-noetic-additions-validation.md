# CBD Noetic Stratified Additions Validation Plan

**Goal**

Locate the Retail heldout depth error and observe whether the additions variant transfers to a second
full online sequence. Execute the approved validation in the current workspace.

**Design**

[Retail depth and CBD replay](../specs/2026-11-27-cbd-noetic-additions-design.md)

**Evidence locations**

- Raw and local reproducible scripts in `validation/cbd-noetic-additions-20261127/`.
- A single results document with the replay entry, with links from the Retail comparison and roadmap.

## Task 1 — Diagnose Retail depth

- [x] Bind the four scored archives and rederive their joint common-depth aggregates and each view's
  contribution. Cross-check against the existing audit and keep shared denominators for single/joint.
- [x] Profile views 25 and 65 by depth range and reference depth jumps. Save error comparison images
  with fixed scales. Where the data point to a two-block effect, check the exact rendered model
  combinations with the existing rasterizer after the timed runs.

## Task 2 — Run the full CBD pair

- [x] Freeze bag/split, native source and binary, tool hashes, configurations, thread budget,
  sequence order, and disk caps. Verify 97 keys and the independent CBD observations.
- [x] Complete flat then additions with heldout exclusion, loop closure, stage timings, Gaussian
  persistence and finalization. Evaluate each final map through the existing scoring entry.
- [x] Verify launch receipts, input counts, thread log, loop/frontier identities, exit, model and
  score assets, shared cameras and selectors, and independent pooled quality/timing statistics.

## Task 3 — Record and commit

- [x] Recheck archive identity and conclusions. Write the paired changes and depth localization,
  document the fixed replay and diagnostic commands, and commit the documents locally.
