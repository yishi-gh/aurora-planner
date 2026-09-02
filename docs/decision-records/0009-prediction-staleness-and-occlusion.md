# ADR 0009: Prediction Staleness and Occlusion Safety Boundary

- 日期：2026-09-01
- 状态：已确认并实现第一版
- 范围：动态预测信息的新鲜度、显式遮挡、ROS 2 watchdog 和恢复流程
- 关联：ADR 0005、0006、0007、0008

## 决策

动态目标消息增加 `occlusion_active` 和可选 `occluded_track_ids`。显式遮挡表示动态目标集合不完整或不可用于生成新轨迹；即使 `tracks` 为空，也必须进入信息失效策略。没有遮挡标志的空 `tracks` 批次仍是有效的“无目标心跳”，不能被误判为遮挡。

节点以 `risk.max_prediction_age`（默认 `0.5 s`）判断最新动态快照是否过期，并以 10 Hz watchdog 检查 ROS 时间相对最新快照的年龄。信息变为不可用时：

1. 清除待处理规划和动态主动重规划事件；
2. 停止发布新的可执行轨迹；
3. 暂时保留当前已经通过静态和动态门控的活动轨迹；
4. `risk.stale_hold_duration`（默认 `0.5 s`）后，若活动轨迹仍在执行窗口内则锁存 `INFORMATION_STALE` 急停；若活动轨迹已结束或不再覆盖当前时间，则立即锁存急停。

进入 stale 状态后收到新鲜动态信息不会自动重规划，而是设置恢复待确认标志。只有显式急停 reset、重新收到有效动态快照、再提交新的 `PlanningRequest`，才允许恢复规划。reset 会清除旧动态快照、旧请求和活动轨迹，因此旧请求或旧目标不能在复位后自动复用。

## 时间语义

- 快照新鲜度以动态批次 `header.stamp` 为信息时间；不使用 ROS 消息接收时刻替代它。
- 风险核心支持 `evaluation_stamp`，用 `evaluation_stamp - snapshot_stamp` 记录 `information_age`。
- 规划候选的风险评估以请求 `planning_stamp` 对齐；watchdog 以当前 ROS 时钟检查活动轨迹执行安全。
- simulated time 还没有有效时，watchdog 不会把未来时间戳误判为过期。

## 原因

空批次可能表示“当前确实没有目标”，也可能是感知链路遮挡或目标列表不完整；仅靠目标数量无法区分二者。动态预测过期后继续发布新轨迹会把旧信息误当作当前世界状态，因此必须将信息新鲜度放在发布门控和轨迹生命周期之外再检查一次。短暂保留已验证轨迹可以避免单个消息抖动立刻触发急停，但保留窗口不能让系统无限执行未知环境中的旧规划。

## 影响

- 新增消息字段会改变 `DynamicObstacleTrackArray` 和 `RiskReport` 的接口版本，需要同步 Humble/Jazzy 构建和 bag schema。
- stale 诊断结果不携带轨迹；锁存期间请求静默丢弃，不生成看似已处理的结果。
- 恢复流程是显式的三步边界，不允许新动态心跳单独解除急停，也不允许 reset 单独恢复旧任务。
- 本切片仍未实现 track 生命周期、遮挡重识别和存在概率校准；这些能力必须在此安全边界上游提供完整输入。

## 验证

- `test_dynamic_risk_evaluator` 覆盖显式遮挡、空心跳和 `evaluation_stamp` 新鲜度。
- `test_dynamic_obstacle_adapter` 覆盖遮挡字段保留和非空遮挡 ID 的不可用语义。
- `test_information_stale_rosbag2_replay.py` 覆盖动态停止更新、stale hold、信息过期急停、轨迹保留和 reset + 新心跳 + 新请求恢复。
- `test_dynamic_occlusion_rosbag2_replay.py` 覆盖显式遮挡、信息失效拒绝和遮挡急停。
