# Graceful ROS shutdown implementation plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Deliver a received STOPPED before closing ROS during normal process shutdown.

**Architecture:** A small process-signal owner in `src/app` feeds the shared main. Preserve multithreaded ROS execution; stop callbacks without first shutting down the context, then run existing output/system finalization. Add transport waiting only if process tests demonstrate it is needed.

**Tech Stack:** C++17, POSIX signals, Noetic, Humble, GoogleTest, Python ROS integration tests.

**Spec:** [Graceful shutdown design](../specs/2026-11-26-graceful-shutdown-design.md).

## Global constraints

- Work in the authorized current development branch; modify only livo-atlas.
- Google C++ style; blank lines between functions; no Pimpl or extra thread pool.
- Keep one shared main and the existing multithreaded executors.
- No ROS/logging/allocation inside a signal handler; no dependence on /clock.
- Preserve configured worker deadlines, incomplete facts and failure exit codes.
- Preserve user input and old validation artifacts; reuse build trees.

## Review focus

- Late/repeated SIGINT or SIGTERM must not close the context before STOPPED.
- Fast teardown must retain the actual terminal message until transport can send it.
- Callback failures or an already closed context must still fail without hanging.
- A paused simulation clock must not block cancellation or any new waiting.
- Slow and timed-out worker cases must preserve deadlines and failure reporting.

### Task 1: Reproduce the real process boundary

**Files:** `tests/ros_common/shutdown_process.py`; validation artifacts.

- [ ] Add a small no-bag process harness using a controlled protocol peer and actual ROS subscribers.
- [ ] Run current Noetic and Humble nodes with paused simulation time and SIGINT; expect missing STOPPED before any production change.
- [ ] Cover SIGTERM, slow stop and a worker stop timeout without a CUDA dependency.

### Task 2: Preserve the context through final publication

**Files:** `src/app/shutdown_signal.h`, `src/app/shutdown_signal.cc`, `src/app/livo_main.cc`, `cmake/node.cmake`, `cmake/tests.cmake`, `tests/app/shutdown_signal_test.cc`.

- [ ] Add signal ownership tests, then implement the minimal safe signal flag and handler restoration.
- [ ] Disable default ROS signal shutdown; use the flag to stop both executor forms and explicitly close ROS last.
- [ ] Verify the process regression, including fast stop, repeated signals and paused /clock.
- [ ] If verified transport loss persists, record the exact boundary before adding a bounded transport wait.

### Task 3: Verify, review and record

- [ ] Run SDK and related Noetic/Humble tests, plus short real GPU/Finalize replays and timeout validation.
- [ ] Obtain one fresh read-only final review; address material findings with failing/passing tests.
- [ ] Record actual counts, limitations and reproducible commands; update the roadmap.
- [ ] Commit locally, retain the current branch, and restore the container to stopped.
