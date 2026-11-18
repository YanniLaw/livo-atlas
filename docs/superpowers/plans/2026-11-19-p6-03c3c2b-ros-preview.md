# C3c2b ROS Preview Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish owned accepted Gaussian images with versioned invalidation through the shared Noetic/Humble entry.

**Architecture:** One plain C++ snapshot/controller and bounded CPU publisher worker; shared generated-message conversion, two thin ROS publishers. A composite message is authoritative; standard Image is a convenience mirror.

**Tech Stack:** Google C++17, Eigen/OpenCV, ROS1 Noetic/ROS2 Humble, existing CMake/catkin/ament, GoogleTest/Python.

**Spec:** `docs/superpowers/specs/2026-11-19-p6-03c3c2b-ros-preview-design.md`.

## Global Constraints

- Current framework_completion checkout, livo-atlas only; no Pimpl, no push/merge/tag, no vikit/source-data changes.
- Existing single src/app/livo_main.cc, no PNG/map reads or GPU requests for ROS output.
- One CPU publish worker, one in-flight and one pending; at most2 distinct render payloads;1M pixels/3M bytes each.
- Default publishing OFF; require existing Gaussian/persistence/joint ON before output creation.
- Render interval100..60000ms inherited; steady-clock rate, urgent invalidation/terminal transitions.
- Optical frame1..128 ASCII bytes, sensor time[0,2^31)sec, exact copied RGB/K/IDs, normalized display quaternion.
- Composite revisions are authoritative; no cross-topic or shutdown-delivery guarantee, no fake image or TF.
- OMP_NUM_THREADS=1 OMP_THREAD_LIMIT=1 OPENBLAS_NUM_THREADS=1 before configure/build/test; one -j1 compiler or GPU workload at a time.
- New phase validation/P6-03c3c2b-20261119 only; archive identities before own duplicate cleanup toward1GiB.
- Inline execution explicitly requested; one fresh final reviewer, no per-task agents.

## File Structure

- include/livo_atlas/gaussian/gaussian_render_snapshot.h: plain status/header/owned RGB reference.
- include/livo_atlas/gaussian/gaussian_render_output.h and src/gaussian/gaussian_render_output.cc: bounded worker/queue/lifetime.
- include/livo_atlas/core/gaussian_render_output_options.h: simple defaulted config, loaded by shared LivoConfig.
- GaussianMapIndex::sequence(), OnlineGaussianMapper::GetRenderSnapshot(): O(1) owner capture.
- msg/GaussianRender{Frame,Source}.msg: common ROS1/2 schema.
- src/ros/gaussian_render_messages.h: shared validation/conversion; ROS1/2 gaussian_render_publisher.{h,cc}: native publishing only.
- existing shared main, options/build branches, replay tools and tests gain optional publication.

## Review Focus

1. A producer restart, same capture time with different jobs, and a stale in-flight ready result must not create a false current version.
2. Invalidation/Finish during a blocked publish must not hold owner locks or retain more than one pending RGB; no publication after join.
3. Oversized/malformed RGB, invalid source identity/camera and timestamp rounding limits must clear ready payloads consistently in both ROS versions.
4. Shutdown or subscriber disconnection cannot be silently counted as delivery; late subscribers and mirrored images must honor the declared QoS/as-of limits.
5. Publishing OFF/ON must preserve frontend and Gaussian scheduling semantics, optional-library boundaries, old aggregates and existing evidence strength.

## Verification commands

Phase owns run-core.sh/run-python.sh/run-all.sh and matrix/replay scripts, based on previous archived harness with new paths. Core uses cmake -E chdir BUILD ctest --output-on-failure -j1. Task completion runs run-all.sh; explicit ROS/matrix/bag checks are recorded separately. Expected exit0, no failed tests. GPU optional owner cases are explicit, never inferred from skips.

### Task 1: Owner capture, shared options and bounded plain C++ output

**Files:** snapshot/output/options headers and source above; mapper/index header+source, livo_config.cc/header, online_gaussian.cmake; tests/gaussian/gaussian_render_output_test.cc, online_gaussian_render_test.cc, tests/core/global_output_options_test.cc.

**Interfaces:** GaussianRenderStatus seven values0..6 as spec. GaussianRenderSnapshot owns producer_id/revision/session_id, version scalars, parent_frame/policy/status/reason and shared_ptr<const GaussianRender>. GetRenderSnapshot()const is owner-only, revision/producer empty until output stamps. GaussianMapIndex::sequence()const O(1). GaussianRenderOutputOptions(enabled=false,optical_frame,minimum_interval=1000ms). Output Consumer=std::function<bool(const GaussianRenderSnapshot&)>; Observe(const Snapshot&)noexcept, Finish()noexcept, GetStats()const. Stats observed/submitted/coalesced/published/unavailable/failed/pending/in_flight/peaks/last_revision/max_worker_seconds/finished/error. Config fields gaussian_output appended to LivoConfig; both parameters as spec.

- [ ] Add RED lifecycle/current-key capture, disabled/error/stopped precedence and options-before-IO tests. Add output tests: blocked first callback while100 replacements leaves pending1; stale pending dropped; monotonic revision with same capture time; urgent invalidation bypasses interval; invalid DTO and throwing consumer produce one cleared ERROR; Finish joins and releases weak_ptr; disabled output creates no worker/callback.
- [ ] Run focused build/CTest, expect missing API or assertions FAIL. Implement bounded source capture, validation and worker with no callback under mutex and no full-map copying. Correctly clear nonready owned RGB, dedup/coalesce and clock waits. Run focused PASS.
- [ ] Commit feat: add bounded Gaussian preview output. Run full completion suite; expected PASS.

### Task 2: Shared ROS schemas, conversion and native transport

**Files:** two msg files; shared converter; ROS1/2 headers/sources; cmake/ros1.cmake/ros2.cmake/tests.cmake; tests/ros1/gaussian_render_publisher_test.cc and ROS2 counterpart.

**Interfaces:** bool GaussianRenderPublisher::Publish(const GaussianRenderSnapshot&)const; constructor(node,optical_frame,map_frame). Shared MakeGaussianRenderMessage<Message>(snapshot,optical,map_frame,stamp). Fields exactly spec section4; Frame revision/producer and source session separate. relative gaussian_render and gaussian_render/image; both queue1/latched or reliable/transient_local depth1.

- [ ] Add failing actual ROS1/2 publication tests for RGB8 bytes, K, sensor stamp, inverse camera pose, sources, empty invalidation and late cache. Include big uint64 IDs, duplicate capture times, same-payload new revision, invalid DTO/time, foreign identity and context shutdown. Expected missing message/publisher FAIL.
- [ ] Generate messages, implement common conversion and thin transports, conditional Gaussian linking. Conversion must publish a cleared ERROR for bad ready payload before propagating failure. Run both ROS test groups and unchanged installed C++17 client PASS.
- [ ] Commit feat: publish versioned Gaussian frames for ROS1 and ROS2. Full completion PASS.

### Task 3: Shared main, replay controls and receiver evidence

**Files:** src/app/livo_main.cc, typed ROS options tests, tools/gaussian_replay_options.py, both runners, new tools/gaussian_preview_report.py and tests, compare_gaussian_replays.py, usage docs.

**Interfaces:** CLI --gaussian-publish-preview (false default), --gaussian-preview-frame-id(defaultgaussian_camera_optical), shared params. Output creation/Observe/Finish in same main and publisher lifetime beyond join. New report validates actual received combo revisions/status/empty payloads and matches READY source/camera/RGB to owner accepted identities and retained PNG when available; existing publish-disabled evidence cannot be retrofitted.

- [ ] Add RED both-runner parameter/dependency/forwarding and report tests: malformed pixels, nonready payload, reordered revision, duplicate timestamp/different job, false shutdown/old-session proof. Add main paused-clock ON/OFF startup tests. Expect FAIL for absent wiring.
- [ ] Wire Observe after mapper Poll and initial capture; stop output before Gaussian terminal workflow, log bounded worker counters and exact terminal facts. Handle nonready without stale wait_reason. Add shared controls and report/observer, preserve omitted persistence/Finalize flags and path checks. No new frontend topic semantics.
- [ ] Run focused tools and main entry PASS; commit feat: connect Gaussian preview publication and replay evidence. Full completion PASS.

### Task 4: Matrix, official bag, final review and archive

**Files:** phase harness/evidence, docs/p6-03c3c2b-validation.md, docs index/plan, own ledger.

- [ ] Pin source/bag/node/libs; SDK shared/static Gaussian OFF/ON; Noetic/Humble typed config, real late reader/paused clock/invalidations and optional disabled entry. Expected PASS; no GPU/vikit linkage in ordinary core/OFF.
- [ ] Actual Noetic short+full spatial rendering pairs with publish OFF/ON (both persistent+Finalize). Observe combo+mirror; strict frontend arrays/raw observations/existing topic digests exact, successful13 current/durable Finalize; at least one pre-input-cutoff READY matches owner and PNG, versions monotonic/nonready empty, slots≤1/1. Slow/throwing consumer and shutdown proofs from controlled native tests; don't promise impossible delivery after ROS shutdown. HumbleCPU/real transport claims only.
- [ ] Whole-range fresh most-capable review from69c0e0e; classify by user effect, one Important/Critical RED/GREEN fix pass and full/matrix/relevant real retest, defer only Minor. No re-review. Full final gate PASS.
- [ ] Archive exact ledger/raw review/all decisions/logs/SHA, verify retained map and old2681-file C3c2a manifest and bag unchanged; cleanup only own duplicates/container phase, target≤1GiB. Remove only this plan scratch after archive and preserve current branch.

## Self-review

Spec sections2/3→Task1, section4→Task2,5→Tasks1/3,6→Task4. Review focus cases assigned to Tests1/2/3 and real matrix; same native transport callback contract throughout. User explicitly opted for direct design+implementation, so preserve native execution without another handoff question. Single final reviewer remains required.
