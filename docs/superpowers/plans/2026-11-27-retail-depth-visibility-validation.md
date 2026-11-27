# Retail Depth Visibility Validation

**Goal**

Trace the previously localized depth errors to the reference support and the actual rendered ray weights.

**Design**

[Fixed reference and ray protocol](../specs/2026-11-27-retail-depth-visibility-design.md)

## Task 1 — Freeze targets and reproject LiDAR neighbors

- [ ] Verify prior floating-point archives and split receipts. Freeze the two exact pixel cohorts,
  renderer configurations, neighborhood rules, and input hashes in the workspace validation folder.
- [ ] Recover the query camera and reconstruct the original per-pixel LiDAR depth and mask.
  Use the same alignment and z-buffer convention to measure neighbors at both fixed radii.

## Task 2 — Trace and compare gsplat weights

- [ ] Render the 12 archived model and mode combinations on the frozen cameras. Reuse the shipped
  rasterizer and capture actual ordered Gaussian indices only for the selected pixels.
- [ ] Reconstruct the archived alpha and expected depth. Retain ordered weights, submap identities,
  and grouped contributions. Verify the per-ray sums and compare on the same reference pixels.

## Task 3 — Record the decision

- [ ] Audit the input and result files. Record depth support, per-block evidence and the limitations,
  with error screenshots, local reproduction commands and the decision for the next experiment.
- [ ] Link the result from the CBD/Retail comparison and roadmap, and commit the documents locally.
