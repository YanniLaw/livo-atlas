# Task scheduling in LivoAtlas

Source from Cartographer 2.0.0 at commit
`105c034577220268cd28a304a185adbec46b729f`.
[Upstream source](https://github.com/cartographer-project/cartographer/tree/105c034577220268cd28a304a185adbec46b729f)

- `src/common/thread_pool.{h,cc}` and `src/common/task.{h,cc}` build into
  `livo_atlas::core` and use the `livo_atlas` namespace. The port implements
  Cartographer's ready queue and task dependencies with C++17 mutexes, a
  condition variable and standard containers. Workers on Linux lower their
  priority when permitted.
- `src/backend/work_queue.h` defines the upstream work-item and work-queue
  types for serial pose-graph work. `LoopWorker` schedules its tasks on
  the ready queue inside `ThreadPool`.

The upstream copyright notices and the Apache 2.0 license apply to these
files. The livo-atlas root license declares GPLv2. Review the terms before
redistributing a build that combines them. [Apache 2.0 license](LICENSE.cartographer)
[ASF license compatibility](https://www.apache.org/licenses/GPL-compatibility.html)
