# ADR 0011: ROS 2 Unassociated Detection Adapter

## 状态

已确认并实现第一版

## 决策

1. 新增 `aurora_msgs/msg/UnassociatedObstacleDetection` 和
   `UnassociatedObstacleDetectionArray`。位置是唯一必需的三维字段；协方差、速度和
   形状用显式 `has_*` 字段表达，避免把零值误解为缺失或确定测量。
2. 批次与每个检测都携带 `std_msgs/Header`。适配器要求批次 frame 为 `map.frame`，
   检测 frame 和时间与批次一致；非法元素计数并保留合法元素，非法批次不会作为空心跳。
3. 适配器只做 ROS 类型转换、时间/frame 校验和 3x3 协方差有限性/对称性/半正定性检查；
   跟踪、关联和生命周期仍由 `aurora_tracking::ObstacleTracker` 负责。
4. 节点通过 `dynamic_input_mode` 选择主输入。默认 `external_tracks` 保持已有接口兼容；
   `internal_detections` 启用 AURORA 内部 tracker。两个来源均保留新鲜时间戳，时间上同时
   新鲜时设置冲突标志，复用现有信息失效、stale hold 和急停策略。
5. 内部 tracker 产生的 `OCCLUDED` 目标仍转换为正常 track 并预测；`LOST` 目标使动态
   集合不完整，不能发布新的未验证轨迹。

## 验证

- `test_unassociated_obstacle_adapter` 覆盖可选字段、形状、空心跳、非法输入和协方差；
- `test_internal_detection_ros2.py` 在 ROS 2 Jazzy 真实 DDS 环境中验证检测消息进入 tracker、
  预测和动态风险门控，并检查节点干净退出。
