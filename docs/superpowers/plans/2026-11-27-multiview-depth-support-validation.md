# Multiview Depth Support Validation

**Goal**

Validate the depth effect on the remaining frozen Retail heldouts and the full CBD heldout set using
the original float scores and projected LiDAR neighbors.

**Design**

[Multiview depth support](../specs/2026-11-27-multiview-depth-support-design.md)

## Task 1 — Freeze and verify inputs

- [ ] Bind the audited score samples, original reference labels, source clouds, split, and finalized
  pose frontiers. Freeze the eligible query list for each scene before computing support or metrics.
- [ ] Reproduce per-pixel depth and the original mask for every query. Verify unchanged cameras,
  selections, and common depth masks for all render arms.

## Task 2 — Run the paired support comparison

- [ ] Project the bounded LiDAR neighbors and classify every original reference label at 2 and 4 px.
  Check a fixed set of pixels against the existing per-pixel implementation.
- [ ] Compare each pair on common single or joint labels. Reconcile subgroup and all-pixel counts,
  error sums, and positive and negative squared-error changes. Preserve the compact support votes.
- [ ] Recompute the result from the frozen inputs, including per-view directions and both radii.

## Task 3 — Record and commit

- [ ] Record the measured scope, outcome, interpretation, and reproduction instructions. Link the
  report from the prior diagnosis and roadmap and commit the documents locally.
