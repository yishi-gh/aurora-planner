# 参数参考

参数由 `aurora_ros` 节点声明；配置文件位于 `src/aurora_bringup/config/aurora_planner.yaml`。下表列出当前公开参数和默认语义。单位为米、秒、米每秒或相应 SI 单位。

## 地图、传感器和车辆

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `map.frame` | `map` | 核心地图和动态风险统一坐标系 |
| `map.origin_x/y/z` | `-20/-20/-2` | 地图原点 |
| `map.dimensions_x/y/z` | `80/80/40` | 体素数量 |
| `map.resolution` | `0.5` | 体素边长 |
| `map.occupancy_threshold` | `0.8` | 占据判定阈值 |
| `map.p_hit/p_miss` | `0.65/0.35` | log-odds raycast 更新概率 |
| `map.p_min/p_max` | `0.12/0.90` | 概率截断范围 |
| `map.inflation_radius` | `vehicle.radius` | 静态障碍膨胀半径 |
| `map.reject_unknown` | `true` | 静态验证是否拒绝 UNKNOWN |
| `map.require_fresh_observation` | `true` | 是否启用地图观测 watchdog |
| `map.max_observation_age` | `1.0` | 最大有效地图观测年龄 |
| `pointcloud.max_points` | `100000` | 单次点云点数上限 |
| `pointcloud.max_range` | `30.0` | 射线最大量程 |
| `pointcloud.confidence` | `1.0` | 点云观测置信度 |
| `tf.timeout_sec` | `0.05` | 点云 TF 查询超时 |
| `depth.enabled` | `false` | 是否启用 `sensor_msgs/Image` + `CameraInfo` 深度输入；关闭时不创建订阅 |
| `depth.max_points` | `100000` | 单幅深度图最多转换的有效射线数 |
| `depth.pixel_stride` | `1` | 深度图像素采样步长；超过点数上限时适配器会确定性增大步长 |
| `depth.min_range` / `depth.max_range` | `0.1` / `30.0` | 有效深度范围，单位米；16UC1 按毫米解释，32FC1 按米解释 |
| `depth.confidence` | `1.0` | 深度观测置信度 |
| `depth.camera_info_time_tolerance` | `0.5` | 非零 CameraInfo 时间戳与图像时间戳的最大差值；零时间戳表示静态标定 |
| `vehicle.radius` | `0.65` | 飞行器保守包络半径 |
| `vehicle.max_velocity` | `3.0` | 最大速度 |
| `vehicle.max_acceleration` | `6.0` | 最大加速度 |

只有有效点云或有效深度图射线和 TF 成功处理后才刷新地图新鲜度。深度适配器只接受单通道
`16UC1`（毫米）和 `32FC1`（米）编码，并要求 CameraInfo 提供有效的针孔投影；无效深度像素会被统计并跳过，
不会被当作自由空间。地图 watchdog 的故障和恢复语义见 ADR 0014。

## 规划、优化和验证

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `planning.horizon_mode` | `distance` | `distance` 或 `time` |
| `planning.local_horizon` | `6.0` | 局部目标距离或时间 horizon |
| `planning.goal_tolerance` | `0.25` | 局部目标位置容差 |
| `planning.max_projection_distance` | `inf` | 当前状态投影限制 |
| `planning.resampling_spacing` | `0.5` | A* 引导点弧长间距 |
| `planning.resampling_minimum_points` | `9` | 引导点最少数量 |
| `planning.optimizer_interval` | `0.25` | B-spline 节点间隔 |
| `planning.optimizer_clearance` | `0.65` | 静态优化安全距离 |
| `planning.optimizer_max_iterations` | `180` | 优化迭代上限 |
| `planning.optimizer_max_compute_time_sec` | `0.0` | 优化墙钟预算；0 表示不启用，超时结果仍须通过发布前安全门控 |
| `planning.optimizer_samples_per_span` | `8` | 优化采样密度 |
| `planning.validation_samples_per_span` | `16` | 发布前验证采样密度 |
| `planning.optimizer_lambda_risk` | `0.0` | 动态软风险权重；0 保持静态基线 |
| `planning.optimizer_max_risk_evaluations` | `10000` | 单次优化软风险查询预算 |
| `planning.stitch_prefix_duration` | `0.5` | 已验证轨迹接续前缀时长 |
| `planning.stitch_position_tolerance` | `0.5` | 接续位置容差 |
| `planning.stitch_velocity_tolerance` | `1.0` | 接续速度容差 |
| `planning.stitch_acceleration_tolerance` | `2.0` | 接续加速度容差 |

软风险回调异常、不可用或超预算时，优化器回退到 EGO 静态目标；无论软优化结果如何，发布前都必须重新通过静态和动态硬门控。

## 预测和风险

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `prediction.sample_interval` | `0.1` | 预测采样间隔 |
| `prediction.max_horizon` | `15.0` | 最大预测时域 |
| `prediction.max_samples` | `10000` | 预测采样上限 |
| `prediction.process_noise_acceleration` | `1.0` | CV 过程噪声 |
| `prediction.process_noise_jerk` | `1.0` | CA 过程噪声 |
| `prediction.default_position_variance` | `0.25` | 缺失位置方差 |
| `prediction.default_velocity_variance` | `1.0` | 缺失速度方差 |
| `prediction.default_acceleration_variance` | `4.0` | 缺失加速度方差 |
| `risk.enable_soft_cost` | `true` | 是否构造动态软风险场 |
| `risk.enable_map_quality` | `false` | 是否将地图质量样本送入统一风险上下文 |
| `risk.require_map_quality` | `false` | 是否要求地图质量输入有效 |
| `risk.allow_unknown_space` | `false` | 是否允许 UNKNOWN 进入显式软风险路径 |
| `risk.require_dynamic_information` | `true` | 是否要求动态信息心跳/快照 |
| `risk.max_prediction_age` | `0.5` | 动态预测最大年龄 |
| `risk.sigma_multiplier` | `3.0` | 保守协方差包络倍数 |
| `risk.warning_clearance` | `0.5` | 动态风险警戒距离 |
| `risk.sample_interval` | `0.1` | 动态风险轨迹采样间隔 |
| `risk.max_samples` | `100000` | 风险采样上限 |
| `risk.max_obstacles` | `1000` | 风险目标上限 |
| `risk.execution_position_variance` | `0.0` | 执行误差对角位置方差 |
| `risk.sensing/tracking/planning/execution_delay` | `0.0` | 延迟预算 |
| `risk.safety_margin` | `0.0` | 额外预测提前量 |
| `risk.stale_hold_duration` | `0.5` | 信息过期后的已验证轨迹保持窗口 |
| `risk.information_watchdog_rate_hz` | `10.0` | 信息 watchdog 频率 |

总延迟作为预测提前量，不改变轨迹绝对执行时间。定位、执行和目标位置协方差直接合并；3-sigma 硬门控不因存在概率降低而关闭。

## 跟踪和接口命名

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `dynamic_input_mode` | `external_tracks` | `external_tracks` 或 `internal_detections` |
| `tracking.mahalanobis_gate` | 9.0 | 有协方差检测的关联门限 |
| `tracking.euclidean_gate` | 1.5 | 无协方差检测的关联门限 |
| `tracking.confirmation_match_count` | 2 | CONFIRMED 所需连续匹配数 |
| `tracking.lost_after` | 0.5 | LOST 时间阈值 |
| `tracking.deleted_after` | 2.0 | 删除时间阈值 |
| `tracking.first_track_id` | 1 | 首个 track ID |
| `tracking.default_shape_radius` | 0.5 | 缺失目标形状时的球半径 |

所有 topic 和 service 名称都可通过 `topics.*`、`services.emergency_stop` 覆盖；公开默认值和 QoS 见 `docs/architecture.md`。高风险参数不应在运行中无审计地动态修改。

## 软件在环和飞控接纳

这些参数由 `aurora_sim` 声明，用于确定性执行边界和故障注入；具体 PX4/MAVROS 2/GZ 后端的发送频率和协议参数由外部适配器单独声明。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `simulation.setpoint_interval` | `0.05` | 飞控接纳边界生成三维 setpoint 的时间间隔 |
| `simulation.max_setpoints` | `2000` | 单条轨迹可生成的 setpoint 数量上限 |
| `simulation.reject_trajectories` | `false` | 软件在环中注入普通飞控拒绝，不能用于真实飞行默认配置 |
| `simulation.max_update_gap` | `0.5` | 执行状态更新 watchdog 超时后失败关闭 |

`aurora_flight_adapter::AdapterOptions` 的 `expected_frame` 与 `time_tolerance` 在软件在环中分别由 `simulation.frame_id` 和 `simulation.time_tolerance` 提供。真实飞控适配器必须显式设置相同的坐标系和时间容差，并记录到运行诊断。
