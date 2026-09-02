# ADR 0003: ROS 2 Static Adapter Contract

- 日期：2026-09-01
- 状态：已确认并实现首版
- 范围：`aurora_msgs` 与 `aurora_ros` 的静态规划传输层

## 决策

首版采用单一 `aurora_planner_node`，核心规划仍由无 ROS 的 C++ 库执行。节点负责：

- 订阅 `/aurora/planning_request` 和点云输入；
- 使用 TF2 将点云转换到 `map.frame`，以快照方式更新概率体素地图；
- 在独立规划线程中复制地图和请求快照，调用静态规划器、FSM 和发布前安全门控；
- 只发布通过 `StaticSafetyGate` 的 `aurora_msgs/msg/Trajectory`；拒绝结果只进入 `PlanningResult`、`SafetyReport` 和状态诊断；
- 通过 `aurora_msgs/srv/SetEmergencyStop` 设置或显式复位锁存急停，并持续发布 `EmergencyStopState`。

QoS 固定为：点云使用 `SensorDataQoS`；规划请求和轨迹使用 reliable、KeepLast(1)；规划状态使用 reliable、KeepLast(10)；急停状态使用 reliable、KeepLast(1)、transient local。

轨迹消息使用自定义 B-spline 分段格式。segment 起始时间是绝对 ROS 时间，`source_start_time` 是 spline 内部相对时间；控制点按 spline 索引顺序传输。动态预测字段先保留在消息契约中，但静态阶段不把“动态信息不可用”伪装为风险通过。

## 原因

该边界保持算法核心可独立测试，同时为后续动态预测和风险门控保留版本、时间、协方差和诊断字段。急停锁存和“只发布已认证轨迹”是执行安全契约，不能由外围节点绕过。

## 后果

- 目前需要补充真实 DDS topic/service 集成测试和 rosbag2 回放；仅成功启动不等价于通信闭环完成。
- 本机受限执行环境不能创建 DDS UDP socket，节点启动测试只能证明参数、线程和接口初始化，不能证明跨进程通信。
- 动态风险模块必须扩展核心门控输入和报告，但不得改变已有时间语义、急停显式复位语义和静态安全门控前置条件。

## 验证

- Ubuntu 24.04 + ROS 2 Jazzy：`colcon build --base-paths src` 成功，7 个包完成。
- `colcon test` 和 `colcon test-result --all --verbose`：69 个条目，0 errors、0 failures、0 skipped。
- 节点在 `ROS_LOG_DIR=/tmp/aurora_ros_log` 下完成初始化并响应 SIGINT；DDS socket 受沙箱权限限制的报错已记录为环境限制。
