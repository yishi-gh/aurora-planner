# ADR 0010: Detection Input and Track Lifecycle Boundary

## 状态

已确认，第一版已实现

## 背景

AURORA 的第一版动态风险门控消费上游已经关联好的
`DynamicObstacleTrackArray`。为了形成独立的无人机规划项目，系统还需要能够
在没有外部跟踪器时接收未经关联的三维障碍物检测，并输出同一套 track/prediction
契约。两种输入不能导致两套风险语义或两套安全降级语义。

## 决策

1. 保留 `/aurora/dynamic_obstacle_tracks` 作为兼容输入。该输入中的
   `track_id`、状态、协方差、形状、预测模型和存在概率由上游跟踪器负责；
   AURORA 不重复关联这些已经形成的 track。
2. 新增可选的未经关联检测输入和 `aurora_tracking` 纯 C++ 核心。内部跟踪器
   将检测转换为与现有 `TrackState` 等价的快照，再复用同一个预测器、风险评估器
   和发布前安全门控。
3. 检测短时丢失时，track 不立即删除：继续进行运动预测，并随丢失时间增加
   状态协方差；超过配置的保留窗口后进入 `LOST`，不再作为正常动态安全信息的
   “已知完整目标集合”使用。
4. 目标重新出现时，允许通过空间和运动一致性重捕获原 track。重捕获不能
   静默改变历史身份；关联失败时创建新 track，并由风险策略决定新生目标的
   保守处理。
5. 跟踪器输出仍必须满足现有信息新鲜度、显式遮挡、协方差有效性和时间线契约。
   跟踪器不能绕过 `INFORMATION_STALE`、`DYNAMIC_COLLISION` 或急停锁存。

## 不采用

- 不把 Nav2 或二维目标跟踪器直接作为三维动态风险输入。
- 不在本阶段引入学习型检测/预测器替换确定性运动基线。
- 不让兼容的已关联 track 输入再次经过内部数据关联。

## 已冻结的接口细节

- `UnassociatedObstacleDetection` 要求三维 position；position covariance、velocity、
  velocity covariance 和 shape 可选；检测 header 必须与批次 header 使用相同 frame 和时间。
- 有 position covariance 时使用 Mahalanobis 门限，否则使用 Euclidean 门限；候选先门控，
  再做确定性 Hungarian 一对一分配。
- 新 track 从 `TENTATIVE` 开始，连续两次匹配后为 `CONFIRMED`；未匹配先为
  `OCCLUDED`，超过 `0.5 s` 为 `LOST`，超过 `2 s` 删除。
- 保留窗口内重捕获沿用原 ID；tracker reset 只清除活动记录，不重置本实例的 ID 序列。
- `/aurora/dynamic_obstacle_detections` 使用 `SensorDataQoS`；`dynamic_input_mode` 默认
  `external_tracks`，可选 `internal_detections`。两路来源同时新鲜时拒绝混用。

## 未冻结事项

- 具体传感器驱动到该消息的适配、外部检测器质量标定和跨传感器时间同步；
- 存在概率、无人机状态不确定性和地图质量进入统一风险策略的模型；
- 多模型/学习型预测和风险软代价。

## 影响

跟踪器是新增的算法核心包，但它只负责观测更新、关联和生命周期；预测、风险和
规划安全门控保持单一实现。这样可以用同一组静态/动态 rosbag2 场景分别验证
“外部 track 输入”和“内部 detection 输入”的结果一致性。
