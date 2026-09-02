# AURORA-Planner ROS 2 架构与接口契约

本文档是实施路线图的接口级补充。它约束核心算法和 ROS 2 适配层之间的边界，避免后续为了接线方便把 ROS 类型、线程和时间语义泄漏到算法库。

## 1. 分层

```text
aurora_bringup / sensors
          |                    px4 / MAVROS 2 / GZ
      aurora_ros                     ^
          |                           |
       aurora_msgs              aurora_flight_adapter
          |                           ^
aurora_planner_core -> aurora_risk -> aurora_prediction
        |             |              |
 aurora_trajectory  aurora_search  track/predict models
        |             |
 aurora_math      aurora_map

aurora_tracking -> aurora_prediction

aurora_ros -- VALIDATED trajectory --> aurora_flight_adapter
```

底层纯 C++ 库不得 include `rclcpp`、`sensor_msgs` 或 `geometry_msgs`。适配层负责复制或转换数据，并在进入核心前完成 frame、时间和质量校验。

## 2. 节点和执行边界

首版采用单一 `aurora_planner_node`：它承载点云地图输入适配、规划请求输入、动态目标快照、核心规划调用、静态/动态安全门控、轨迹发布、状态诊断和急停服务。当前请求消息同时携带无人机状态和全局参考；后续可增加独立里程计适配，但不得让核心依赖 ROS 消息。这样先固定端到端时间语义和安全闭环；地图、预测和可视化拆分为独立节点属于后续工程优化，不改变核心接口。

首版稳定后再评估 composable nodes 或拆分节点。目标拆分形态如下：

| 节点 | 输入 | 输出 | 责任 |
|---|---|---|---|
| `aurora_map_node` | `PointCloud2`/深度、TF、里程计 | 地图快照、地图诊断 | 更新局部概率地图，不做规划 |
| `aurora_prediction_node` | 目标观测、TF、时钟 | track/prediction、预测诊断 | 可选的内部跟踪、关联和短期预测 |
| `aurora_planner_node` | 里程计、目标、地图快照、预测快照 | 验证轨迹、风险报告、状态 | 调用核心规划器和安全门控 |
| `aurora_traj_server` | 验证轨迹、当前时间 | 飞控/控制器目标 | 轨迹采样和执行时序，不改变规划结果 |
| `aurora_flight_adapter` | 已验证 `Trajectory`、飞控反馈 | 三维 setpoint、接纳/拒绝动作 | 独立复核轨迹并隔离 PX4/MAVROS/GZ 具体协议 |
| `aurora_visualization_node` | 地图、轨迹、预测、风险 | RViz2/Foxglove markers | 只读可视化 |

单节点内部仍需保留线程边界：ROS 回调线程只更新带版本号的输入快照，规划工作线程读取不可变快照并调用纯 C++ 核心，发布线程只发布已经通过门控的结果；急停服务只能改变锁存安全状态，不能绕过门控发布轨迹。节点拆分后复用同一快照和消息契约。

核心规划 API 的调用应有明确截止时间 `planning_budget`。ROS 回调不应在无界循环中等待地图或预测；输入快照缺失时返回结构化失败。

首版 `aurora_ros` 的接口契约如下：

| 方向 | 接口 | QoS/处理 | 结果 |
|---|---|---|---|
| 输入 | `/aurora/planning_request` | reliable，KeepLast(1)；回调只保存最新请求并唤醒规划线程 | 核心 `PlanningRequest` |
| 输入 | `topics.pointcloud`，默认 `/points` | `SensorDataQoS`；TF2 转换到 `map.frame` 后更新地图 | 概率体素地图版本递增 |
| 输入 | `topics.depth_image` + `topics.camera_info` | `depth.enabled=false` 默认关闭；启用后按图像时间缓存标定，支持 `16UC1`/`32FC1`，TF2 转换到 `map.frame` 后更新地图 | 有效深度射线刷新地图；坏编码、坏标定、坏 TF 或全无效图像不刷新 |
| 输入 | `/aurora/dynamic_obstacle_tracks` | reliable，KeepLast(1)；保存最新批次快照，规划线程内按候选时域预测；新时间戳触发一次主动重规划 | `DynamicObstacleTrackArray` |
| 输入 | `/aurora/dynamic_obstacle_detections` | `SensorDataQoS`；可选内部跟踪输入，默认不启用 | `UnassociatedObstacleDetectionArray` |
| 输出 | `/aurora/trajectory` | reliable，KeepLast(1)；仅发布静态与动态门控均通过的轨迹 | `Trajectory.validation_state=VALIDATED` |
| 输出 | `/aurora/planning_result` | reliable，KeepLast(1)；成功和失败均发布 | 结果、风险、失败和安全诊断 |
| 输出 | `/aurora/planner_status` | reliable，KeepLast(10) | FSM 状态、动作和触发原因 |
| 输出 | `/aurora/emergency_stop_state` | reliable、KeepLast(1)、transient local | 锁存急停状态 |
| 服务 | `/aurora/set_emergency_stop` | ROS 2 service | engage 锁存，reset 显式复位 |

规划线程在互斥锁内复制请求、活动轨迹和地图，释放锁后运行核心算法；点云和深度图回调只在锁内更新地图，发布器不在规划计算期间持有输入锁。深度图回调先读取最近一份同 frame 的 CameraInfo，再将针孔反投影得到的相机坐标射线通过图像时间的 TF 转换到地图坐标；有效射线不足时保持原地图新鲜度。当前节点为“最新请求优先”策略，未处理请求会被后来的请求覆盖；规划线程取走请求后立即清空 pending 槽位，后续地图或动态目标更新不能重放旧请求。动态目标回调在收到时间戳严格晚于最近规划时钟的快照时生成一次主动规划事件；相同时间戳不会重复触发。主动规划复用最近一次全局参考和 `request_id`；如果已验证活动轨迹覆盖动态快照时间，则从活动轨迹评估并同步三维无人机位置、速度、加速度和状态时间，再推进 `planning_stamp`。活动轨迹不覆盖该时间时不使用过期状态创建主动规划，等待新的显式 `PlanningRequest`；显式请求仍优先提供请求内容。

急停服务是锁存的安全边界：`engage=true` 立即禁止待处理工作，`engage=false` 只发起显式复位。复位会清除 pending 请求、最近请求、动态自动重规划事件和活动轨迹，并递增规划代次；规划线程在复位时再次清空这些槽位，以覆盖服务回调与新消息并发到达的情况。规划中的旧计算如果观察到急停、复位标志或代次变化，则丢弃结果，不发布轨迹、结果或状态；复位后必须收到新的显式 `PlanningRequest` 才能恢复规划。该策略必须在 rosbag2 集成测试中记录，不可解释为无界请求队列。

动态目标接入使用 `aurora_msgs/msg/DynamicObstacleTrackArray` 批量传输。当前单节点已订阅该接口：批次 header 必须使用 `map.frame`，每个目标 header 必须与批次的时间和 frame 一致；不支持的形状/模型、重复 `track_id`、非有限运动状态和非法 header 会单独计入 `invalid_track_count`。批次还必须明确表达 `occlusion_active`；可选 `occluded_track_ids` 用于标记受影响目标，非空 ID 列表本身也视为遮挡。显式遮挡与合法空目标心跳不同，前者进入信息失效策略，后者可作为无目标动态快照。批次快照仍会被保存，但风险门控拒绝含非法目标或遮挡标志的批次，避免剩余目标被误判为完整安全信息。节点不做数据关联和生命周期管理，`track_id` 由上游提供并作为唯一标识。

当前 AURORA 已增加可选的未关联检测输入和 `aurora_tracking` 纯 C++ 跟踪核心。参数
`dynamic_input_mode` 默认为 `external_tracks`，只有显式设置为
`internal_detections` 时，未关联检测才成为主动态输入；两类订阅仍同时存在，用于发现
误配置。已关联的 `DynamicObstacleTrackArray` 不经过内部重复关联；未经关联检测先经过
ROS 适配、数据关联和生命周期管理，再转换为同一套内部 track 快照。因此预测、风险评估、
信息过期、遮挡和急停边界只有一套实现。两个来源在时间上同时新鲜时，节点拒绝混用并将
快照送入现有 `INFORMATION_STALE` 策略。

当前纯 C++ `aurora_tracking::ObstacleTracker` 已冻结并接入第一版检测契约：位置是必选三维观测；位置协方差、速度、速度协方差和形状是可选字段；缺失字段使用显式保守默认值。带位置协方差的候选使用预测位置与检测协方差合成的 Mahalanobis 距离，没有协方差时使用欧氏距离；候选先按门限过滤，再用带虚拟未匹配项的确定性 Hungarian 一对一分配。每个新检测创建 `TENTATIVE` track，连续两次匹配后转为 `CONFIRMED`；未匹配 track 先进入 `OCCLUDED`，继续预测并参与风险评估，超过 `0.5 s` 进入 `LOST`，超过 `2 s` 删除；删除后的 ID 不复用，保留窗口内重捕获沿用原 ID。`LOST` 使内部动态集合标为不完整并进入现有信息失效策略。非法检测批次不会被当作空心跳，纯无效批次不推进内部时间；混合批次返回 `PARTIAL_INPUT`，供上层安全策略拒绝不完整动态信息。ROS 2 适配器要求批次和检测 header 使用同一 `map.frame` 与时间，位置协方差和速度协方差在边界正则化后再进入核心。

## 3. 核心输入快照

规划器收到的单次请求逻辑上等价于：

```text
PlanningRequest {
  request_id
  planning_stamp
  frame_id
  vehicle_state { position, velocity, acceleration, covariance }
  goal { position, velocity, acceleration, tolerance }
  global_reference
  map_snapshot { version, stamp, bounds, query_interface, quality }
  dynamic_snapshot { version, stamp, tracks, quality }
  vehicle_model { shape, max_vel, max_acc, max_jerk, max_snap }
  risk_policy
  compute_budget
}
```

`map_snapshot` 和 `dynamic_snapshot` 必须是同一规划时刻可解释的快照。允许其内部版本不同，但结果中必须返回版本号和年龄，便于回放定位。

## 4. 地图查询契约

地图查询不能使用单一 `bool`：

```text
MapState = FREE | OCCUPIED | UNKNOWN | OUT_OF_MAP

MapQueryResult {
  state
  occupancy_probability
  inflated
  observation_age
  confidence
  map_version
}
```

调用者必须明确指定：是否把未知当占据、是否使用膨胀层、允许的最大年龄和风险阈值。地图实现应保证同一快照内查询稳定；更新过程不能让读者看到半更新状态。

当前静态搜索基线使用 `aurora_search::AStar3D`，只接受静态 `VoxelMap` 快照。它使用 26 邻域和三维欧氏启发，轴向、面角和体角代价分别为 `resolution`、`sqrt(2) * resolution` 和 `sqrt(3) * resolution`；所有多轴移动的中间子格必须可通行。搜索将 `UNKNOWN`、`OCCUPIED` 和 `OUT_OF_MAP` 统一视为不可通行，但通过 `SearchStatus` 区分非法输入、端点阻塞、预算超时和无路。A* 输出只作为后续 B-spline 初值的离散引导，不能直接作为可执行飞行轨迹。

该基线的核心调用形态为：

```text
SearchResult search(start, goal, SearchOptions)
```

`SearchOptions` 提供最大扩展节点数和单调时钟计算预算；`SearchResult` 提供状态、世界坐标路径、离散栅格代价、扩展数和生成节点数。路径首尾保留请求中的精确世界坐标，中间点为体素中心。

## 5. 动态目标和预测契约

内部状态至少包含：

```text
Track {
  id
  source_stamp
  frame_id
  state_mean
  state_covariance
  shape
  existence_probability
  lifecycle_state
  last_update_age
}

Prediction {
  track_id
  reference_stamp
  states[] {
    absolute_stamp
    mean
    covariance
    shape
    mode_probability
  }
  model_id
  valid_until
}
```

规定：

- `absolute_stamp` 是碰撞检查的唯一时间依据；
- `valid_until` 到期后不可静默沿用；
- 协方差必须对称半正定，数值修正要计入诊断；
- 目标形状和飞行器形状必须统一在同一 frame 中比较；
- `existence_probability` 低于策略阈值时可作为软代价，但不能将未知目标当作确定安全；
- 预测节点只负责产生预测，是否接受预测由风险模块和安全门控决定。

统一风险上下文在核心侧定义为 `aurora_risk::RiskContext`：

```text
RiskContext {
  map {
    available
    snapshot_stamp
    map_version
    samples[] { stamp, position, state, occupancy_probability,
                observation_age, confidence, inflated, map_version }
  }
  vehicle {
    localization_position_covariance
    execution_position_covariance
  }
  delay {
    sensing_delay
    tracking_delay
    planning_delay
    execution_delay
    safety_margin
  }
}
```

`RiskContext` 在进入风险评估前必须通过 `validateRiskContext`。已提供的协方差必须有限、对称且半正定；延迟分量必须有限且非负；可用地图样本必须有合法状态、概率、年龄、置信度和快照版本。缺失地图或协方差可以作为显式缺失输入，由风险策略决定降级，不能被默认为高置信零误差。

ROS `VehicleState.state_covariance` 按 `[p, v, a]` 的 9x9 行主序传输。适配层将左上角位置 3x3 子块保存到核心 `VehicleState.position_covariance`，随后复制到风险上下文；执行误差通过 `risk.execution_position_variance` 提供首版对角默认值，默认 0 表示未提供。

统一风险策略 v1 的延迟和协方差规则见 `docs/decision-records/0012-unified-risk-policy-v1.md`。当前 3-sigma 动态门控仍保持硬包络行为；协方差合并、延迟提前量、存在概率软风险和地图质量风险已启用。

定位和执行误差在风险边界保留来源，经校验后按独立位置误差合并到相对协方差；延迟字段单位为秒，统一总量用于预测提前量。地图样本包含轨迹采样位置，用于防止地图质量样本与轨迹错位。ROS 节点默认 `risk.enable_map_quality=false`，保持既有 EGO 静态回放基线；显式启用后，地图 `OCCUPIED`、`OUT_OF_MAP` 和默认 `UNKNOWN` 拒绝轨迹，`FREE` 按质量字段形成软风险。

当前 `aurora_prediction::KinematicPredictor` 已实现无 ROS 的第一版传播基线。CV 使用六维位置/速度状态和白噪声加速度过程模型；CA 使用九维位置/速度/加速度状态和白噪声 jerk 过程模型。两者对外统一返回位置/速度 6x6 协方差，以匹配现有 `DynamicObstacleTrack` 和 `PredictedState` 消息；CA 的加速度协方差在内部使用显式默认方差，并设置 `acceleration_covariance_defaulted` 诊断标志。

预测器不做数据关联和状态滤波更新，只对已经形成的 track 快照执行 Kalman prediction step。它包含当前时刻和 horizon 末端样本，末端采用精确剩余步长；horizon、采样数量、状态有限性、协方差对称性和半正定性均有结构化失败状态。缺失协方差使用保守默认值，数值容差内的负特征值会被截断并记录 regularized，明显非半正定输入直接拒绝。

当前节点在规划线程内根据候选轨迹结束时间对每个快照 track 调用预测器，再将候选 B-spline 按 `risk.sample_interval` 采样。`aurora_risk::DynamicRiskEvaluator` 使用绝对时间线性插值，要求预测覆盖轨迹首尾；安全半径为 `vehicle_radius + obstacle_radius + 3 * sqrt(lambda_max(relative_position_covariance))`。盒、胶囊和多球先转换为保守包围球；包络相交返回 `DYNAMIC_COLLISION`，缺失/过期/不完整信息或显式遮挡返回 `INFORMATION_STALE` 并拒绝发布。风险输入可带独立 `evaluation_stamp`，报告记录 `information_age`。

风险上下文当前已完成首版策略：存在概率以不低于 0.05 的有效概率缩放软风险；飞行器定位/执行协方差与目标协方差直接相加；延迟总量转换为预测时间提前量；地图质量使用占据概率、观测年龄和置信度加权，最坏动态/地图项进入统一总分。动态软风险通过 ROS 无关回调接入 `StaticBsplineOptimizer`，有独立评估预算，失效时回退静态 EGO 目标；优化后仍必须通过发布前静态和动态硬门控。

## 6. 轨迹契约

纯数学层同时提供 `aurora_math::MinimumSnapTrajectory` 和 `aurora_math::StrictMinimumSnapTrajectory`。前者接受 `3 x N` 航点矩阵、分段时间、起终点速度和加速度，生成每段六项系数的五次多项式；该接口遵循 EGO 五次 `minSnapTraj` 基线，其平滑目标是积分平方 jerk。后者生成每段八项系数的七次多项式，以归一化段时间构造并求解积分平方 snap 目标，约束起终点位置、速度、加速度和 jerk，内部位置到六阶导数连续，评估时导数使用物理秒。

`aurora_math::allocateSegmentTimes` 根据相邻三维航点距离、最大速度/加速度/jerk、最小段时长和时间缩放系数生成确定性初始时间表。其加速度和 jerk 项是保守的 rest-to-rest 下界；它不是严格动力学可行性证明，生成的轨迹仍必须经过静态/动态安全门控和动力学验证。

`aurora_math::resamplePath` 将 A* 世界坐标折线按弧长转换为稳定的引导点序列。它清理连续重复点，以 `spacing` 为常规间距，在短路径上均匀补足 `minimum_points`（默认 7）个点，并强制保留请求中的精确首尾点。该序列仍然是离散引导，不代表已通过碰撞和动力学验证的轨迹。

当前静态轨迹切片由 `aurora_trajectory` 提供。`StaticBsplineOptimizer` 接收静态 `VoxelMap` 快照、EGO 非钳位 B-spline 初值和参考控制点，使用三阶控制点差分平滑、参考拟合、最近占据体素障碍势能以及速度/加速度超限代价；优化时冻结首尾三组控制点。`validateStaticTrajectory` 在位置、速度和加速度采样点上检查 `OUT_OF_MAP`、`OCCUPIED`、`UNKNOWN` 和动力学上限，并返回结构化状态。

当前 `aurora_planner_core` 已把这些算法串成无 ROS 的 `StaticLocalPlanner`：请求携带当前状态、全局参考和规划时间；`GlobalReference` 支持无时间航点、带时间采样的五次参考轨迹，以及由严格 minimum-snap 和自动时间分配生成的三维带时间参考。规划器先将当前状态投影到参考路径，再按距离或时间 horizon 提取局部目标，随后调用静态 A*、弧长重采样、EGO B-spline 初值、静态优化和验证。搜索失败、数值异常或验证失败时，结果不携带轨迹。

ROS 2 传输层使用 `aurora_msgs` 的自定义 B-spline 轨迹消息。`TrajectorySegment.start_stamp` 和 `Trajectory.header.stamp` 是绝对 ROS 时间；`source_start_time` 是对应 spline 时钟内的相对秒数；`duration` 和 `dt` 使用秒；控制点按 spline 索引顺序传输。控制器只能接受 `validation_state=VALIDATED` 且 `safety_report.accepted=true` 的轨迹，`REJECTED` 只用于结果和诊断，不得送入控制器。`SetEmergencyStop` 服务的 `engage=true` 进入锁存急停，`engage=false` 只请求显式复位；当前急停状态通过 `EmergencyStopState` 话题持续发布。节点另以 10 Hz watchdog 检查动态快照年龄：过期或遮挡时停止新轨迹、保留已验证轨迹至 `risk.stale_hold_duration`，再以 `EmergencyStopState::INFORMATION_STALE` 锁存急停。恢复必须经过 reset、新鲜动态快照和新的显式规划请求。

轨迹生命周期由 `PlannedTrajectory` 和 `TrajectorySegment` 表示。每个 segment 有绝对 `start_stamp`、有效 duration 和 spline 时钟偏移；因此旧轨迹可以被切出一个窗口作为新结果的安全前缀。只有旧轨迹已标记验证通过、当前状态与旧轨迹接近、前缀在当前地图快照上重新验证通过时才会接续；新旧段的边界位置、速度和加速度也会再次检查。静态前缀接续之后，`aurora_planner_core::StaticSafetyGate` 会对完整候选轨迹执行发布前认证：检查候选 segment 的时间窗口和数值有效性、段间状态连续性、与当前轨迹的起点状态连续性，再逐段调用 `validateStaticTrajectoryWindow` 检查静态地图、未知区域和动力学限制。门控返回 `SafetyGateStatus` 与采样统计，不直接执行旧轨迹保留、悬停或急停；FSM 读取结果后负责这些策略动作。当前 ROS 2 节点随后使用 `aurora_risk::DynamicRiskEvaluator` 复核动态时空风险；只有静态和动态门控都通过才发布轨迹。动态门控当前使用 3-sigma 几何包络，不提供概率安全保证。

静态重规划由 `StaticReplanFsm` 管理。它不直接调用规划器，而是输出动作，由上层执行 `StaticLocalPlanner::plan` 后回传 `PlanningResult`：

```text
INIT -> WAIT_TARGET                         (无全局参考)
INIT/WAIT_TARGET -> GENERATE                (收到全局参考，开始规划)
GENERATE -> EXECUTE                         (规划结果成功且轨迹已验证)
EXECUTE -> REPLAN                           (检测到重规划触发)
REPLAN -> GENERATE                          (启动新一轮规划)
规划失败且仍有安全旧轨迹 -> EXECUTE         (保留旧轨迹)
连续失败达到阈值 -> DEGRADED                (悬停/保持)
无安全回退或剩余安全时间不足 -> EMERGENCY_STOP
```

触发选择顺序固定为：当前轨迹碰撞、安全信息过期、优化/验证失败、动态障碍物更新、地图更新、轨迹接近结束、局部目标过期。动态更新是一次性观测事件，即使新快照尚未使当前轨迹碰撞，也必须重新评估并生成新候选；碰撞和信息失效仍拥有更高优先级。`REQUEST_REPLAN` 到 `START_PLANNING` 的两步决策保留同一个触发原因，便于 ROS 2 状态诊断和回放关联。优化或验证失败既可以通过 `planning_failed` 观测信号触发，也会在规划结果回调中记录为 `PLANNING_FAILURE`，不伪装成正常触发。默认连续失败阈值为 3 次；`DEGRADED` 中只有在当前轨迹仍安全且剩余时间大于急停阈值时才保持，否则进入锁存急停。`safety_information_stale` 是静态阶段预留给未来地图质量、预测和风险适配器的统一信号。

首版优化器采用确定性的梯度下降和回溯线搜索，优化状态 `CONVERGED`、`MAX_ITERATIONS`、`STALLED` 均保留给上层安全门控判断；`STALLED` 不是安全通过信号。当前障碍势能和验证仍是静态采样基线，不等价于连续时间或概率安全证明。

规划输出逻辑上等价于：

```text
ValidatedTrajectory {
  trajectory_id
  frame_id
  start_stamp
  duration
  position_bspline
  velocity_bspline
  acceleration_bspline
  constraints_summary
  map_version
  dynamic_version
  risk_report
  validation_state = VALIDATED | REJECTED | DEGRADED
  failure_or_fallback_reason
}
```

要求：

- 起始时间不早于规划时钟允许范围；
- 轨迹有效区间、控制点、`dt` 和导数维度一致；
- 轨迹采样不能跨越 frame 或时间基准；
- 只有 `VALIDATED` 或明确允许的 `DEGRADED` 输出可以发布；
- `REJECTED` 只能进入安全策略，不能交给控制器。

当前纯 C++ 编排包的主要状态为 `SUCCESS`、`GOAL_REACHED`、`NO_GLOBAL_REFERENCE`、`LOCAL_GOAL_UNAVAILABLE`、`SEARCH_FAILED`、`OPTIMIZATION_FAILED` 和 `VALIDATION_FAILED`。这些状态是 ROS 2 状态消息的候选来源；`PREFIX_UNSAFE` 当前表示一种计划内回退原因，而不是允许发布的结果状态。

FSM 另外提供 `INIT`、`WAIT_TARGET`、`GENERATE`、`EXECUTE`、`REPLAN`、`DEGRADED` 和 `EMERGENCY_STOP` 状态，以及 `WAIT`、`REQUEST_REPLAN`、`START_PLANNING`、`WAIT_FOR_RESULT`、`ACCEPT_NEW_TRAJECTORY`、`KEEP_CURRENT_TRAJECTORY`、`HOLD_POSITION` 和 `EMERGENCY_STOP` 动作。急停状态必须显式 reset；reset 只恢复等待新目标的初始状态，不自动恢复旧请求或旧轨迹，避免异常输入恢复后自动继续飞行。

## 7. 风险计算顺序

```text
轨迹采样 t_i
  -> vehicle envelope + tracking uncertainty
  -> static map query at t_i
  -> dynamic prediction query at absolute time
  -> relative covariance / confidence envelope
  -> collision probability or conservative bound
  -> risk aggregation and worst-case report
  -> hard gate + optimization cost
```

风险模块应输出分项结果而非只有一个总分：静态地图风险、动态目标风险、信息过期风险、控制跟踪风险、总风险、最小距离、最危险目标和最危险时刻。

第一版可以使用离散轨迹采样和保守上界，但采样间隔必须和最大速度、障碍物尺寸、允许穿透距离绑定，不能只使用固定低频采样掩盖碰撞。

当前第一版的实际门控子流程为：

```text
DynamicObstacleTrackArray
  -> batch/header/frame/track 校验
  -> immutable track snapshot
  -> candidate end time -> CV/CA prediction
  -> absolute-time B-spline samples
  -> 3-sigma relative position envelope
  -> conservative clearance and normalized risk
  -> DYNAMIC_COLLISION / INFORMATION_STALE / ACCEPTED
```

动态快照更新还会进入重规划 FSM 的事件通道：节点以快照时间戳相对最近规划时钟的单调推进作为去重依据；事件被规划线程取走后清除。该事件和动态风险门控是两个不同契约：前者保证信息变化会重新评估轨迹，后者决定候选轨迹是否可执行。

当动态信息进入 stale 状态时，节点清除 pending 请求和动态事件，停止新轨迹发布；watchdog 默认每 `0.1 s` 检查一次。已验证活动轨迹最多保留 `risk.stale_hold_duration`（默认 `0.5 s`），但若轨迹已结束或不再覆盖当前时间则立即进入锁存急停。stale 恢复不是自动事件：reset 会清除旧动态快照、请求和活动轨迹，之后必须先收到没有遮挡且未过期的动态批次，再收到新的显式规划请求。

该版本有意不实现数据关联、目标生命周期、IMM/学习预测、Monte Carlo 碰撞概率和风险梯度优化；这些能力必须在基线回放与校准数据之后单独确认。

## 8. QoS、时间和线程

- 传感器输入使用与实际驱动匹配的 sensor-data QoS；地图/轨迹/诊断分别定义可靠性和深度。
- 规划器使用 ROS steady clock 做计算预算，使用消息时间做世界状态对齐；二者不能混用。
- 所有跨线程快照使用不可变对象、版本号或双缓冲；不在优化器运行期间修改地图。
- 规划线程不能持有 ROS 发布锁；可视化和诊断不能阻塞规划。
- 轨迹服务采样应使用执行时钟和明确的轨迹版本，不能用“最新消息”猜测当前轨迹。

## 9. 参数分组

参数按以下命名空间组织，并导出 YAML：

```text
map.*
vehicle.*
trajectory.*
search.*
optimizer.*
prediction.*
risk.*
safety.*
runtime.*
```

每项参数要有单位、默认值、合法范围、是否允许运行时修改和对应测试。风险阈值、未知策略、最大信息年龄和紧急停车参数默认为只读。

## 10. 错误和状态码

核心 API 应返回结构化状态，至少包括：

```text
OK
INVALID_INPUT
STALE_MAP
STALE_PREDICTION
TF_UNAVAILABLE
NO_GLOBAL_REFERENCE
NO_SAFE_LOCAL_TARGET
SEARCH_TIMEOUT
OPTIMIZATION_TIMEOUT
OPTIMIZATION_FAILED
DYNAMIC_RISK_TOO_HIGH
STATIC_COLLISION
KINEMATIC_LIMIT_VIOLATION
EMERGENCY_STOP_REQUIRED
```

错误状态应能映射为 `diagnostic_msgs/DiagnosticArray`，并包含 request id、输入版本、年龄、求解耗时和 fallback 动作。

## 11. 执行器和仿真边界

`aurora_flight_adapter` 是 ROS 消息到飞控/执行器之间的接纳边界，不能修改规划结果，也不能绕过发布前安全门控。它只接受 `Trajectory.validation_state=VALIDATED` 且 `Trajectory.safety_report.accepted=true` 的消息，复核 frame、绝对时间、段连续性、三次 B-spline、源时间窗口和控制点，按配置间隔生成绝对时间的三维位置/速度/加速度 setpoint。它不发送具体飞控命令；PX4、MAVROS 2 或 GZ 适配器负责消费接纳结果并保留轨迹 ID、时间窗口和回执。

`aurora_sim` 是 ROS 2 外围执行适配器，复用上述接纳边界，再由无 ROS 的 `aurora::simulation::TrajectoryExecutor` 重建 B-spline，按绝对时间执行三维轨迹，并以有界速度/加速度的跟踪模型产生 `VehicleState`。拒绝结果通过 `TrajectoryExecutionStatus` 记录。

执行器把时间回退和过大的执行更新间隔视为安全故障，清除活动轨迹并进入停止状态；急停话题同样清除活动轨迹。飞控接纳边界将普通拒绝、轨迹 ID 不匹配和未接受的活动/完成回执映射为 `REQUEST_REPLAN`；反馈时间非法、未知状态、时钟回退和执行 `TIME_GAP` 映射为 `EMERGENCY_STOP`。它的状态反馈可用于软件在环测试，但不能作为真实飞控的动力学或安全证明。PX4/GZ 通过独立具体适配器接入，必须复用同一轨迹验收边界、时间语义、拒绝回执和异常降级规则。
