# Gaussian scheduling and log-token validation implementation plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Diagnose the intermittent two-content-attempt render test and reject malformed boolean log tokens.

**Architecture:** Preserve the existing single-worker mailbox and owner-thread scheduler. Test contention at mailbox admission without changing the protocol, adding queues, or relaxing scheduling assertions.

**Tech Stack:** C++17, GoogleTest, Python unittest, ROS1 Noetic, ROS2 Humble.

**Spec:** [Online joint rendering design](../specs/2026-11-18-p6-03c3c1-online-joint-rendering-design.md), scheduling and resources; [regression tools](../../regression-tools.md).

## Global constraints

- Work only in `livo-atlas`, on the authorized current development branch.
- Keep one in-flight mailbox and preserve foreground/repair fairness.
- Preserve nonblocking owner polling; do not add production sleeps or feature macros.
- Follow Google C++ style, separating function definitions with a blank line.
- Retain source data and existing validation evidence; reuse build trees to limit disk usage.

## Review focus

- A temporarily occupied mailbox must not silently spend an eligible render turn.
- Both map synchronization and render command admission need controlled contention coverage.
- A missed map acknowledgement must still yield to content rather than starve training.
- Boolean values followed by whitespace, commas or end of line remain valid.
- Boolean prefixes such as `trueish` and `falsehood` must fail validation.

### Task 1: Identify and cover the scheduling boundary

**Files:** `tests/gaussian/online_gaussian_render_test.cc`, `include/livo_atlas/gaussian/online_gaussian_mapper.h`, `src/gaussian/render_scheduler.cc` if justified by reproduction.

- [x] Record baseline repetition and add event details to failures.
- [x] Add narrow test access for controlled mailbox contention; reproduce the scheduling assertion before changing production behavior.
- [x] Make the smallest demonstrated correction, keeping dispatch nonblocking and preserving synchronization-miss behavior.
- [x] Run focused tests, repeated scheduling cases and the complete SDK suite.

### Task 2: Validate complete boolean tokens

**Files:** `tools/timing_analysis.py`, `tests/test_regression_tools.py`.

- [x] Add malformed boolean suffix cases and observe failure.
- [x] Require an allowed token boundary and retain valid delimiter coverage.
- [x] Stabilize the existing failed-rebuild fixture after reproducing identical modification/generation timestamps; leave the production build runner unchanged.
- [x] Run the complete regression-tools unittest suite.

### Task 3: Verify and record

- [x] Build and run the affected Gaussian test in native Noetic and container Humble.
- [ ] Obtain one fresh read-only final review, address material findings with failing/passing tests.
- [x] Update the engineering roadmap and regression documentation with actual results and limitations.
- [ ] Commit locally; do not push or merge.
