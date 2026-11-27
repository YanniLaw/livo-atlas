# CBD View 73 Depth Attribution

**Goal**

Measure which of the sealed CBD models affects the remaining distant depth regression, with
per-ray evidence on fixed well-supported pixels.

**Design**

[CBD 73 depth attribution](../specs/2026-11-28-cbd-73-depth-attribution-design.md)

## Task 1 — Freeze and render

- [ ] Bind the archived scores, common geometry, depth support and heldout lineage for view 73.
- [ ] Freeze the full far-depth set and the 64 ray samples before rendering.
- [ ] Render and trace the two single and four joint variants. Match all original alpha/depth
  endpoints and validate the ray sums.

## Task 2 — Verify attribution and record

- [ ] Compare the common intersections, signed depth, SSE, replacement effects and interactions.
  Reconstruct per-layer and per-submap changes from the raw ray weights.
- [ ] Verify the result a second time and check source/output receipts. Document scope,
  findings and the reproducible commands, then commit locally.
