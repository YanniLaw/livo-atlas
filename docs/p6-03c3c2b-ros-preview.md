# P6-03c3c2b：ROS Gaussian 预览使用说明

本功能发布 owner 已接纳的联合渲染图像，不读 PNG 或地图，不增加 GPU 请求。
同一个 `src/app/livo_main.cc` 同时接入 Noetic 和 Humble；转换位于
`src/ros/gaussian_render_messages.h`，两个原生 publisher 只负责时间类型和发布。

## 开启

默认 `gaussian.publish_preview=false`。开启时必须同时启用 Gaussian、
`gaussian.persist_submaps` 和 `gaussian.joint_render_enabled`，并编译可选 Gaussian 库。
`gaussian.preview_frame_id` 默认 `gaussian_camera_optical`，是独立虚拟光学帧；
不发布它的 TF。光学帧限128字节、父地图帧限256字节，均校验 ASCII slash 分段与名称冲突。
发布普通更新的间隔继承 `gaussian.joint_render_interval_ms`；状态切换立即处理，
包括恢复 READY，避免连续失效/等待消息推迟第一张有效图像。
同状态的WAITING/INVALIDATED版本头更新仍按间隔节流；待发状态转换的紧急属性会保留。

两套回放工具使用相同参数，在已有持久联合渲染命令后增加：

```bash
--gaussian-publish-preview --gaussian-preview-frame-id gaussian_camera_optical
```

完整依赖参数为 `--gaussian-socket PATH --gaussian-output NEW_DIRECTORY
--gaussian-persist-submaps --gaussian-joint-render`；空间选择另加
`--joint-render-policy spatial`，最终修复另加 `--gaussian-finalize-on-shutdown`。
省略发布开关时不覆盖 YAML 中已有的发布设置；持久化和 Finalize 的省略语义也保留。
参数校验和已有路径保护发生在节点输出创建前。

## 两个相对话题

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `gaussian_render` | `livo_atlas/GaussianRenderFrame` | 有版本的状态、来源、相机、RGB 组合消息，程序以此为准 |
| `gaussian_render/image` | `sensor_msgs/Image` | 标准 Image 镜像，便于 RViz 查看 |

Noetic 为 queue1/latched；Humble 为 reliable/transient_local/keep_last1。
Humble 晚加入订阅者应请求 transient_local；volatile 订阅者等待下一条更新。
两个话题不保证原子对应或跨话题顺序，不能用 Image 的 stamp/ROS1 seq 推测版本。

## 版本和相机合同

- `producer_id` 标识发布实例，`revision` 在该实例内递增，合并会留下缺口。
  固定一个逻辑发布者，拒绝旧/重复 revision；重启后换 producer，接收端显式重新绑定。
  `session_id` 则是 Gaussian 源会话，不能与发布实例混用。
- READY 是 as-of 快照，携带完整来源子集和当时的版本头。
  stamp 是查询图像的采集时间，范围为 `[0,2^31)` 秒；不改成 `now()`。
  K 保留原始像素中心约定，不再加0.5。RGB8/step/尺寸受1M像素、3M字节限制。
- `parent_frame` 为 `camera_init` 或配置的地图帧；`parent_from_camera` 是
  `camera_from_parent` 的逆，四元数归一化用于显示，不承诺矩阵逐位无损。
- WAITING、INVALIDATED、DISABLED、ERROR、STOPPING、STOPPED 都没有图像、来源、
  查询 job/K/采集时间/像素指标，pose 为 identity。收到非 READY 即撤回自己的显示状态。
- 慢 native publish 由单个 CPU worker 执行，最多一个执行槽和一个最新待发槽。
  已开始的调用不能强行撤销；完整校验后还会核对最新revision再接纳回调；
  串行回调防止旧 revision 在新 revision 之后调用。
  `Finish()` join，SDK consumer 必须最终返回，publisher 的生命周期覆盖 join。

`published` 只表示 native 调用完成，远端送达由独立接收记录证明。
关闭 context 的竞态可产生部分发布，因此 unavailable 尝试也可能被接收端观察到。
这里 STOPPED 表示预览流收尾；SLAM/地图最终修复是否完成须看独立 Finalize 报告。
ROS 已关闭时终端帧不能保证送达，接收端还须监控连接/发布者存活。
空 Image 会替换缓存，但 RViz 可能保留最后纹理，不能承诺自动清屏。

## 验证记录

回放工具生成 `gaussian-publication.json`，包含实际收到的组合消息、标准镜像摘要、
回调时刻和错误，不保存第二份 RGB 原图。校验命令为：

```bash
python3 tools/gaussian_preview_report.py --run RUN_DIRECTORY --previews GAUSSIAN_DIRECTORY
python3 tools/compare_gaussian_replays.py --off OFF_RUN --on ON_RUN \
  --previews ON_GAUSSIAN_DIRECTORY --preview-publication-comparison
```

成对比较要求两侧持久联合渲染和 Finalize 都开，只改变发布开关；保持既有前端数组、
原始文件、话题语义、最终地图和输入截止时刻检查，并要求至少一份截止前实际收到的
READY 与 owner 接纳身份及保留 PNG 的相机/RGB 对应。旧验证不能补写接收记录冒充新证据。
实际验证结果见[本阶段验证记录](p6-03c3c2b-validation.md)。
