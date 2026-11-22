# Cartographer scheduler

Source from Cartographer 2.0.0 at commit
`105c034577220268cd28a304a185adbec46b729f`.
[Upstream source](https://github.com/cartographer-project/cartographer/tree/105c034577220268cd28a304a185adbec46b729f)

- `cartographer/common/thread_pool.{h,cc}` and `cartographer/common/task.{h,cc}`
  retain Cartographer's ready queue, dependency handling and task states. The
  port replaces Abseil synchronization, annotations and hash map with C++17
  mutexes, a condition variable and STL containers. Names are nested under
  `livo_atlas` so the SDK can coexist with upstream Cartographer libraries.
  Linux workers continue at the inherited priority if lowering it fails.
- `cartographer/mapping/internal/work_queue.h` preserves the upstream work-item
  and work-queue types. The online loop backend uses the ready queue inside
  `ThreadPool`.

The upstream copyright notices and [Apache 2.0 license](LICENSE) apply to these
files. The livo-atlas root license declares GPLv2. Review the terms before
redistributing a build that combines them.
[ASF license compatibility](https://www.apache.org/licenses/GPL-compatibility.html)
