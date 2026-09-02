# 仿真与执行闭环

## 当前实现

`aurora_sim` 提供一个确定性的三维软件在环执行层。它不是新的规划器，也不替代 PX4；它先调用 `aurora_flight_adapter::TrajectoryAdmission` 接纳 ROS 2 `aurora_msgs/msg/Trajectory`，再把接纳结果转换为三次 B-spline，在绝对时间上采样，并用有最大速度/加速度约束的跟踪模型推进三维飞行状态。该接纳库可由 PX4/MAVROS 2/GZ 具体后端复用。

执行器有独立的安全边界：只有 `validation_state=VALIDATED` 且 `safety_report.accepted=true` 的轨迹可以进入执行模型。轨迹段必须是三次 B-spline、时间连续、未过期，且 `dt`、控制点和时间窗口都合法。未验证、未通过安全报告、过期、时间回退和执行更新超时均失败关闭。

当前软件在环节点输出：

| 话题 | 类型 | 语义 |
|---|---|---|
| `/aurora/sim/vehicle_state` | `aurora_msgs/msg/VehicleState` | 加速度受限模型的当前三维状态 |
| `/aurora/sim/desired_pose` | `geometry_msgs/msg/PoseStamped` | 当前执行时刻的期望位置 |
| `/aurora/sim/execution_status` | `aurora_msgs/msg/TrajectoryExecutionStatus` | 接受、执行、完成和拒绝原因 |

输入为 `/aurora/trajectory` 和 `/aurora/emergency_stop_state`。执行状态消息中的 `TIME_ROLLBACK`、`TIME_GAP` 和 `REJECTED_UNVALIDATED` 是安全诊断，不应被解释为规划成功。

## 运行

在已构建工作区中：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch aurora_bringup aurora_software_in_loop.launch.py
```

该启动文件组合 `aurora_planner_node` 和 `aurora_sim_node`。规划器仍需接收点云、有效动态目标心跳和 `PlanningRequest`；状态预热后再发点云的顺序与静态 rosbag2 基线一致。

也可以单独启动执行器：

```bash
ros2 launch aurora_sim aurora_sim.launch.py
```

主要参数位于 `src/aurora_sim/config/aurora_sim.yaml`。`simulation.reject_trajectories=true` 用于模拟飞控拒绝；`simulation.max_update_gap` 用于模拟执行更新 watchdog。故障参数只用于测试和仿真，不应在真实飞行配置中默认打开。

## 已验证场景

`test_trajectory_admission` 覆盖接纳边界的验证状态、安全报告、frame、时间窗口、三维 setpoint 和反馈动作；`test_trajectory_executor` 覆盖纯 C++ 的三维跟踪、未验证/不安全轨迹拒绝、非连续/过期轨迹、急停、时钟回退和执行更新间隔超限。`test_simulation_ros2.py` 覆盖规划器到执行器的真实 DDS 链路、B-spline 消息转换、明确的垂向运动、未验证轨迹拒绝和注入的飞控拒绝。

这组测试证明软件在环接口和安全门控闭环，不证明真实动力学、PX4 参数、传感器噪声或飞行器姿态控制性能。

## PX4/GZ 外部后端边界

真实 SITL 使用 PX4 SITL、Gazebo/GZ、`px4_msgs`/`px4_ros_com` 或经审查的 MAVROS 2 适配器，以及必要的 `ros_gz_bridge`。这些依赖不是 AURORA 核心的编译依赖，也不在当前本机验证环境中；接入时必须将外部飞控状态、拒绝回执、执行时钟和姿态/位置反馈映射到本项目已有的轨迹生命周期和安全报告。

外部后端的验收顺序固定为：

1. 只接收 `VALIDATED` 轨迹，并验证飞控拒绝不会被误报为执行成功。
2. 用三维位置反馈驱动连续局部重规划，检查轨迹段边界和绝对时间。
3. 注入点云/深度掉线、TF 缺失、时钟回退、动态预测过期和飞控拒绝。
4. 证明所有异常都进入可解释降级或急停，且没有未验证轨迹到达飞控。

在 PX4/GZ 环境安装并通过上述矩阵前，路线图中的 SITL 交付保持 `部分完成`，不能标记为真实飞行验收完成。
