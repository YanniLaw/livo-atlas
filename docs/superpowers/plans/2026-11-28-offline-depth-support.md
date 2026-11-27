# Offline Depth Support and Regressing Views

**Goal**

Expose a CPU depth-support entry for frozen final-snapshot score archives and localize the remaining
well-supported depth regressions in Retail and CBD.

**Design**

[Offline depth support](../specs/2026-11-28-offline-depth-support-design.md)

## Task 1 — Implement and check the entry

- [ ] Add failing focused checks in `tests/3dgs_probe/test_depth_support.py` for native depth
  projection, neighborhood support, shared-pixel metrics, and archive/runtime CLI behavior.
- [ ] Implement `tools/3dgs_probe/depth_support.py` and `evaluate_depth_support.py`. Reuse the heldout
  snapshot and exact score-sample readers. Provide a bounded report, same-camera controls, and
  exclusive publication.
- [ ] Pass focused checks and the full Python/CUDA suite. Review the new tool and test coverage.

## Task 2 — Verify retained archives and investigate residuals

- [ ] Run the new entry on the original Retail and CBD archives. Verify every view's common label
  counts and previous CPU score results for the six runs and both radii.
- [ ] Use frozen Retail view 1 and CBD views 73 and 57 to locate remaining regressions on matching
  pixels. Check projected support at both radii, spatial concentration, and signed error direction.
- [ ] Write and link a reproducible report. Check archived evidence and commit locally.
