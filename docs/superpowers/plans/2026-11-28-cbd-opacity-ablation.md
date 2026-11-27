# CBD Opacity Ablation

**Goal**

Check whether reducing opacity for the traced 26 far-layer Gaussians reduces the view 73 depth
regression while preserving observed coverage and image quality in view 65.

**Design**

[CBD opacity ablation](../specs/2026-11-28-cbd-opacity-ablation-design.md)

## Task 1 — Freeze and render

- [ ] Bind the old ray archive, score reports, sealed blocks, and camera controls. Freeze indices
  and the opacity multiplier before generating the new results.
- [ ] Reproduce both original renders and apply the opacity-only intervention. Save modified
  samples, coverage information, and the original and modified view 73 ray traces.

## Task 2 — Verify and record

- [ ] Recompute per-view RGB, coverage, depth errors and exact ray contributions. Compare on
  the frozen targets and audit any lost coverage.
- [ ] Repeat the renders and CPU comparisons, verify input and result receipts, and document
  the measured behavior and experimental scope. Commit the report locally.
