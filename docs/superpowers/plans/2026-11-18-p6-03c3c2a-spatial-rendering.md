# P6-03c3c2a Spatial Rendering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the five C3c1 reliability findings and select at most one spatial
Gaussian neighbor using confirmed content metadata and the retained query camera.

**Architecture:** The existing owner, one worker and one model remain. Candidate
CPU state supplies tentative bounds; only accepted confirmation updates the owner
index. A pure C++ selector projects bounded extents; the worker only renders the
selected sources. Default temporal messages and persisted asset/map schemas stay
unchanged. ROS display is a separate C3c2b plan.

**Tech Stack:** Google C++17, Eigen, OpenCV, Python 3.10, pinned gsplat 1.5.3,
Noetic and Humble, existing CMake/catkin/ament and unittest/GoogleTest.

**Spec:** `docs/superpowers/specs/2026-11-18-p6-03c3c2-spatial-rendering-design.md`

## Global Constraints

- Work in the existing `framework_completion` checkout; no push, merge or tag.
- Modify only livo-atlas; preserve vikit, bag, prior validation and environments.
- Ordinary C++ values, no Pimpl; functions separated by a blank line; comments
  explain conventions and invariants. New C++ is at most 80 columns.
- Both ROS versions retain the same `src/app/livo_main.cc` entry and shared
  parameter validation. Humble validation is CPU/entry/SDK, not CUDA evidence.
- One GPU channel/model/mailbox; total in-flight 1 and pending render 1. Preserve
  two content attempts, one due-render opportunity and one synchronization bound.
- At most 32 confirmed Gaussian groups, 512 primary frames/declarations,
  10 extents/group, 10000
  Gaussians/block, 20000 selected Gaussians, 1M pixels, 64KiB header, 16MiB body,
  131200-byte render request. No filesystem work during C++ selection.
- Render rotations use 1e-6 SO3 tolerance without changing frontend tolerance.
- Export OMP_NUM_THREADS=1, OMP_THREAD_LIMIT=1, OPENBLAS_NUM_THREADS=1 before
  configure/build/tests. One compiler or GPU workload at a time; builds use -j1.
- New evidence goes to `validation/P6-03c3c2a-20261118`; reduce retained evidence
  toward 1GiB only after hashing/archiving this phase's disposable duplicates.
- Every behavior change has a witnessed RED/GREEN; docs need no mirrored tests.
  Use the current session inline, then one fresh whole-range final reviewer.

## File Structure

- `gaussian_map.h` owns content-bound extent/camera/metadata values and optional
  confirmed entry metadata; the index owns their confirmation lifetime.
- New public `gaussian_render_policy.h` owns the two policy enum values.
- New public `gaussian_projection.h` and `src/gaussian/gaussian_projection.cc`
  own the conservative projection and bounded integer rectangle union.
- New `src/gaussian/render_validation.{h,cc}` centralizes render-only pose and
  metric checks. Existing selector, codec, scheduler and writer keep their jobs.
- New `tools/3dgs_probe/spatial_metadata.py` extracts and validates lightweight
  metadata; existing protocol/actor modules negotiate and transport it.
- Existing shared options, replay tools/report and focused test files gain the
  policy and independent source evidence. No ROS publisher is introduced here.

## Review Focus

1. Source changes between candidate, decision, checkpoint, prune and reply:
   accepted metadata must follow the exact content, including restored binding.
2. Finite extreme transforms/K/scale and near-plane crossing must retain any
   uncertain projection, rather than silently culling a possible contribution.
3. Unsupported capability or forged metadata must fail before it can be treated
   as an ordinary unavailable neighbor; default temporal layouts stay unchanged.
4. Fatal timeout, joined writer and repeated waiting polls must release owned
   outputs and expose honest current counts without unbounded work/log growth.
5. A syntactically valid altered source hash/type or disguised non-PNG must not
   pass accepted-file verification; no enhanced evidence is invented for old runs.

## Verification commands

The phase owns `run-core.sh`, `run-python.sh`, `run-all.sh` adapted from the frozen
C3c1 harness, with all output redirected to phase logs. `run-all.sh` builds and
runs core CTest in its actual build directory, then Python probe and tools suites.
Task-specific filters are recorded in the task ledger. Task completion command:
`/home/yanni/livo-atlas_ws/validation/P6-03c3c2a-20261118/run-all.sh`.
Expected: exit 0, no failed tests; optional owner real-worker tests are run
explicitly later and their default skipped status is not GPU proof.

### Task 1: Close C3c1 reliability and evidence findings

**Files:** render scheduler/mapper/header, new render_validation, render codec,
writer/header/SDK docs, core CMake; owner log and `tools/joint_render_report.py`;
tests/gaussian/{online_gaussian_render,gaussian_render,render_protocol}_test.cc,
tests/io/gaussian_render_writer_test.cc, tests/test_joint_render_report.py.

**Interfaces:**
- Produce `ValidateRenderPose(const RigidTransform&)` and
  `ValidateRenderMetrics(const GaussianRenderMetrics&, int pixels)` in the
  internal render_validation header, throwing invalid_argument.
- `ValidateRenderImage(const GaussianRender&)` shares image/camera/composition,
  timing and metric validation between decoded DTOs and the direct SDK writer.
- `GaussianRenderSourceIdentity(const GaussianRenderRequest&)` is the public
  bounded header-identity function used by the shared node entry and SDK.
- Append `pending_storage`, bounded string `wait_reason`, `wait_map_sequence`
  and `wait_changes` to GaussianRenderStats. Add private
  `SetRenderWait(const std::string&, std::uint64_t)` for state-change dedup.
- Owner accepted logs carry `source_identity`, the lowercase hex of the existing
  EncodeGaussianRenderRequest header, at most twice the 64KiB header budget.
  The report decodes/parses it and compares the entire original as_of object.

- [ ] Add failing timeout/Finish count tests, waiting reason and unchanged-map
  polls, missing single-block reason/zero block generation, SO3 between 1e-6 and
  1e-4, perfect/empty/positive-MSE PSNR inconsistencies, changed source hash/type
  and a JPEG renamed PNG. Run focused tests; expect assertion failures.
- [ ] Clear current render/storage objects only at definitive failure or join;
  preserve cumulative/peak stats. Surface and cache unusable-source reasons by
  map sequence; distinguish interval and map synchronization without Poll spam.
- [ ] Share render checks across DTO/codec/writer; require single omission reason
  and no omission for a dual request. Reuse existing request encoder for log
  identity; require original joint_render type and actual PNG format in report.
- [ ] Document one serial publisher per output directory in public writer and
  SDK docs; no new locks. Run focused tests; expect PASS, then full command above.
- [ ] Commit `fix: close Gaussian rendering reliability findings`.

### Task 2: Confirm content-bound metadata under an opt-in capability

**Files:** gaussian_map/index, mapper Command/Reply, submap commands/protocol;
new Python spatial_metadata, actor/online_worker/submap_protocol; associated
map-index, wire, actor and installed consumer tests, public-header CMake;
new src/gaussian/render_metadata.cc and render_metadata_protocol.{h,cc}; new
gaussian_render_policy.h and the defaulted core-options policy member.

**Interfaces:**
- `GaussianExtent {uint64_t origin_keyframe_id; Eigen::Vector3d min, max;
  double max_scale;}`. Bounds are actual optimized local means.
- `GaussianCamera {uint64_t keyframe_id, calibration_id; int width, height;
  Eigen::Matrix3d K; RigidTransform camera_from_body; double capture_time;}`.
- `GaussianRenderMetadata {GaussianContentId content;
  vector<GaussianExtent> extents; GaussianCamera camera;}`; append optional
  `render_metadata` to GaussianSubmapEntry and candidate Command.
- Produce policy enum/header and OnlineGaussianOptions.joint_render_policy here
  so opt-in capability negotiation has its final configuration from the start;
  Tasks3/4 consume these instead of introducing a temporary enable flag.
- `ValidateGaussianRenderMetadata(const GaussianRenderMetadata&,
  const GaussianContentId&, const vector<uint64_t>& frame_ids)` checks exact
  identity, finite ordered bounds, positive scale, <=10 origins exactly covering
  retained IDs, latest query camera, <=1M pixels, K convention and render pose.
- Keep old RecordConfirmed signature; overload with const optional metadata&.
  Record content and metadata together, preserve on reuse, replace on training.
- Python `extract_metadata(state, content)` returns wire dict from existing
  CPU candidate arrays and latest retained view; `validate_metadata(metadata,
  content, frame_ids)` enforces the same semantics. No second full-state copy.
- Add OnlineGaussianModel.capture_candidate_state(context), sharing the existing
  capture body with capture_state(context). Spatial candidate capture is held in
  pending and reused on accept; temporal capture timing remains unchanged.
- Internal DecodeGaussianRenderMetadata(node, content, frame_ids) and
  WriteGaussianRenderMetadata(out, metadata) provide the focused wire boundary.
- Opt-in hello capabilities are exactly joint_render_v1 followed by
  joint_render_spatial_v1. Candidate alone appends render_metadata; accepted
  decision checks the same metadata before owner index insertion. Persisted
  schema stays unchanged, asset in-memory descriptions may retain metadata.

- [ ] Add failing extraction/identity/origin/query/budget tests and lifecycle
  candidate/discard/accept/reuse/rebuild/checkpoint/prune tests. Pin old default
  headers. Run focused CPU/Python tests; expect failure for absent metadata.
- [ ] Implement values/validation/extraction and negotiated codec transport.
  Actor pending metadata never becomes confirmed until accept; no data leak on
  empty activation or failure. Bound asset metadata by existing references/prune.
- [ ] Run lifecycle and real actor tests, old protocol tests, public headers and
  old C++17 aggregate consumer; expect PASS. Run full completion command.
- [ ] Commit `feat: confirm bounded Gaussian render metadata`.

### Task 3: Pure C++ conservative projection and spatial selector

**Files:** new policy/projection headers and source, gaussian_render.cc/header,
CMake, tests/gaussian/gaussian_projection_test.cc and gaussian_render_test.cc.

**Interfaces:**
- `enum class GaussianRenderPolicy {kTemporal, kSpatial};`.
- `GaussianProjection {uint64_t candidate_pixels; double distance;
  bool uncertain;}` and `ProjectGaussianExtents(const vector<GaussianExtent>&,
  const GaussianCamera&, const RigidTransform& camera_from_submap,
  double coordinate_abs_upper)`; the coordinate bound contains query/block
  parent and camera-body translations needed by the spec's FP32 allowance.
- Append policy (default temporal), candidate_count, candidate_pixels,
  projection_uncertain and selection_seconds to request; these describe the
  chosen neighbor (zero area/count if none). Keep old selector overload and add
  `SelectGaussianRender(map, poses, generation, policy, reason)`.
- Append optional render_metadata to GaussianRenderBlock so a selected request
  owns the exact content bounds/camera used to choose it. Temporal blocks omit it.
- Internal projection_envelope.h defines GaussianEnvelope with half-open integer
  left/top/right/bottom and uncertain, and ProjectGaussianEnvelope(extent,
  camera, camera_from_submap, coordinate_abs_upper). The public aggregate uses
  it for the <=10 rectangle union; the real GPU probe can inspect that same
  production envelope without expanding the public SDK result type.
- The coordinate bound is the sum of absolute translation components of query
  parent, block parent and camera/body; means use a componentwise L1 upper bound.
- IsCurrent reselects using request.policy and compares selection identity,
  camera metadata and content, excluding diagnostic timing from identity.

- [ ] Add failing front/behind/edge/near/far/inside/half-pixel/extreme/uncertain
  projection cases and deterministic union/tie tests. Synthetic owner3, invisible
  previous2 and visible old0 must select0+3; unavailable latest never falls back.
  Test malformed partitions, missing metadata, <=32 groups and exact ranking.
  Run focused CTest; expect failure for missing API/temporal-only selection.
- [ ] Implement spec section6 envelope: transformed8 corners, outward
  64*epsilon_float allowance, scale*(1+1e-4), z clip [.01,1e10], effective K
  +.5 once, clamped Jacobian radius (3.33,.3,+2), <=10 rectangle integer union.
  Legal unstable arithmetic returns uncertain/full canvas; invalid DTO throws.
- [ ] Spatial selector applies existing usability/canonical checks and metadata
  validation, includes owner, ranks area descending/distance ascending/ID
  ascending; no candidate gives no_spatial_candidate. Validate source ordering
  and new spatial summary, retain default temporal semantics byte-for-byte.
- [ ] Run focused tests and full completion command; expect PASS.
- [ ] Commit `feat: select Gaussian neighbors by conservative projection`.

### Task 4: Wire spatial policy through the existing worker and owner

**Files:** options/shared parameter reader, mapper/scheduler/render codec,
Python joint_render_protocol/online_render/joint_renderer/online_worker,
tests/core/global_output_options_test.cc, ROS1/2 node_options tests,
tests/3dgs_probe and online_gaussian_render_test.cc.

**Interfaces:**
- Append OnlineGaussianOptions.joint_render_policy default kTemporal;
  gaussian.joint_render_policy accepts only temporal/spatial. Spatial requires
  enabled+persist_submaps+joint_render_enabled before any output creation.
- Spatial render header appends policy=spatial and selection summary only under
  negotiated joint_render_spatial_v1; temporal request/reply keysets unchanged.
  The root adds `policy: "spatial"` and `selection` with candidate_count,
  candidate_pixels, projection_uncertain (0/1), selection_seconds. Each selected
  block adds render_metadata. A new internal ValidateRenderSelection(request)
  checks the same summary/reason rules in DTO and header-only reply parsing.
  Worker validates nonadjacent earlier blocks, exact query metadata/camera and
  body targets; reply echoes the original spatial request identity.
- The GPU envelope test uses a small test-only gaussian_projection_probe
  executable: bounded line-delimited JSON metadata/coordinate input, actual
  ProjectGaussianExtents/ProjectGaussianEnvelope output. No new runtime worker
  or SDK-only diagnostic method is introduced.
- Preserve original scheduler quotas and source/timeout fatal vs OOM/write
  preview-only classification. Return-time re-selection includes metadata.

- [ ] Add failing unsupported capability, invalid mode/dependencies, forged
  metadata/camera/summary, stale spatial map, restored old binding generation and
  nonadjacent third-resident asset tests. Pin temporal wire fixtures; run focused
  tests and observe failure for absent policy integration.
- [ ] Connect shared options and capability, candidate index insertion,
  selection/admission/return validation. Keep full primary body arrays only for
  selected blocks and retain request/body limits. Worker inference is read-only.
- [ ] Run actual GPU nonadjacent asset render with third block resident, snapshot
  parameters/Adam/grads/frontiers before and after. Compare actual gsplat projected
  support/radii with C++ envelope on seeded random and adversarial fixtures; no
  tested visible Gaussian may fall outside the envelope. Run real OOM and writer
  failure continuation/Finalize and all focused tests; expect PASS.
- [ ] Run full completion command and commit `feat: integrate spatial joint rendering`.

### Task 5: Replay controls and independently checked spatial evidence

**Files:** gaussian_replay_options.py, run_baseline.py/run_ros2_baseline.py,
joint_render_report.py/compare_gaussian_replays.py, owner logging, test scripts,
README and SDK/validation docs; associated tools tests.

**Interfaces:**
- CLI `--joint-render-policy {temporal,spatial}` defaults temporal; shared
  options emit the policy only consistently with validated joint rendering.
- Python validate_render_header(header) validates the exact request keyset and
  identity without requiring unavailable primary body arrays; decode and report
  share it. The report checks every accepted header, including unretained jobs.
- Report/summary distinguish policy, selected IDs, candidate area/count,
  uncertainty, selection time and nonadjacent count, while checking whole
  accepted-header identity and existing conservative input cutoff.
- Old archived logs retain their original evidence strength; new acceptance
  requires source_identity, never substitutes rewritten owner records.

- [ ] Add failing both-runner forwarding/default/dependency tests and report
  tampering/policy/summary/nonadjacent tests. Run tools suite; expect failures.
- [ ] Implement shared forwarding and verified reporting, preserve omitted
  persist/Finalize YAML semantics and path checks, document new mode and limits.
- [ ] Run tools tests/full completion command; expect PASS. Commit
  `feat: report spatial Gaussian selection and source evidence`.

### Task 6: Full matrix, official bag, fresh review and retained handoff

**Files:** phase harness/evidence, docs validation record and roadmap, plan ledger.

**Interfaces:** Consume Tasks1-5 public APIs, exact header identities and report.
Produce reproducible versioned evidence and explicit remaining C3c2b boundary.

- [ ] Freeze source/peer/node/libs and bag checksums after build. Run shared and
  static SDK Gaussian OFF/ON with old consumers/headers; Noetic/Humble entry and
  real typed params. Expect all pass, no Gaussian/Torch/CUDA/vikit linkage OFF.
- [ ] Run short and full Noetic paired OFF/temporal/spatial persist+Finalize
  replays with identical OMP/settings/source. Expect frontend arrays/raw retained
  observations/published semantics exact; current durable successful Finalize.
  Spatial requires >=6 actual dual RGB contributions before conservative input
  cutoff in >=3 query groups. Report Retail nonadjacent count honestly (zero is
  a dataset coverage gap); synthetic real-GPU nonadjacent success is mandatory.
- [ ] Collect CPU selection/worker time, RSS/VRAM, stale/admitted/saved/peaks and
  latency. Run actual owner GPU tests, OOM/write-failure continuation and timeout
  classification, full suite and matrix; expect PASS and bounds1/1.
- [ ] Commit product/validation docs, create whole-range review package using
  fdaea4e as implementation base; one most-capable fresh reviewer reads exact
  spec/plan/ledger and Review Focus. Re-grade findings by user effect; Important
  and Critical each get RED/GREEN in one fix pass and final full suite/matrix/
  relevant real replays; defer only Minor, record all rulings with cost if wrong.
- [ ] Archive exact ledger/raw review/logs with SHA256 before deleting only this
  plan's scratch directory; hash own retained artifacts, clean own redundant
  builds/data toward1GiB. Leave old evidence untouched. Final report links results,
  includes all rulings/deferred minors and identifies C3c2b as next separate work.

## Plan self-review

All spec sections4-8 map to Tasks1-6; section9 is explicitly deferred. Metadata
types and optional argument additions preserve source compatibility. Review
Focus1/3 tests belong Tasks2/4, 2 belongs Tasks3/4, 4/5 belong Task1/5. A single
final reviewer and inherited inline execution avoid repeated permission stops;
the user has already delegated local implementation and verification.
