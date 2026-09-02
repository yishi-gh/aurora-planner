# ADR 0015：飞控轨迹接纳边界

- 状态：Accepted
- 日期：2026-09-02
- 范围：`aurora_flight_adapter`、`aurora_sim` 和未来 PX4/MAVROS 2/GZ 具体适配器

## 背景

规划器发布的是带绝对时间和安全报告的 ROS 2 B-spline 消息。软件在环执行器和具体飞控适配器都需要把它转换为控制目标；如果每个后端各自解释 `validation_state`、时间窗口或 segment，便可能出现规划器认为不可执行而执行端仍接受的分叉。飞控协议本身又不应进入纯规划核心，也不应由一个尚未安装的具体后端阻塞核心测试。

## 决策

1. 新增协议无关的 `aurora_flight_adapter::TrajectoryAdmission`，作为所有执行端共享的第二道接纳边界。它不发送飞控命令，不实现姿态控制，也不依赖 PX4、MAVROS 2 或 GZ。
2. 接纳的必要条件是：`validation_state=VALIDATED`、`safety_report.accepted=true`、header frame 与期望 frame 相同、时间有限且未过期、segment 绝对时间连续、degree 为 3、knot mode 合法、源时间窗口位于 spline 有效区间、控制点和评估出的三维位置/速度/加速度均有限。
3. 接纳成功后，边界按配置的固定间隔生成带绝对时间戳的三维位置、速度和加速度 setpoint，并限制单条轨迹的 setpoint 数量。边界保留轨迹 ID、frame、segment 和 spline 时间窗口，供具体后端建立回执关联。
4. 执行反馈使用统一动作映射：接受中的 `ACTIVE` 保持执行，接受的 `COMPLETED` 标记完成；普通拒绝、ID 不匹配和未接受的活动/完成状态请求 `REQUEST_REPLAN`；`EMERGENCY_STOP`、`TIME_ROLLBACK`、`TIME_GAP`、非法时间或未知状态进入 `EMERGENCY_STOP`。
5. `aurora_sim` 必须先调用该接纳边界，再把接纳结果交给确定性执行器。未来 PX4/MAVROS 2/GZ 后端复用同一边界，只负责协议、QoS、发送和回执转换。

## 影响

- 规划器发布前的静态/动态安全门控与执行端二次接纳形成明确的职责链；接纳层不能把拒绝轨迹“修正”为可执行轨迹。
- 软件在环可以验证执行协议、安全拒绝和三维时间采样，而不冒充 PX4 动力学或真实飞行验收。
- 具体飞控适配仍未实现；完成 SITL 需要增加协议适配、状态反馈、回执故障和端到端时钟测试，不能仅凭 `aurora_flight_adapter` 的单元测试标记为 SITL 完成。
- setpoint 数量上限和采样间隔是边界资源约束，不改变规划器轨迹的绝对时间语义；超限必须拒绝并请求重规划或进入更高安全策略。

## 验证

`test_trajectory_admission` 覆盖有效三维轨迹、验证状态/安全报告拒绝、过期和 frame 错误、非法 segment、setpoint 上限、反馈到重规划/急停的映射；`test_simulation_ros2.py` 验证规划器到执行器经过该边界的真实 DDS 链路。具体 PX4/GZ 验收保持在路线图阶段 6。
