# 外部后端接入边界

本项目的核心输出是经过静态安全门控和动态风险门控认证的三维时间参数化 B-spline。PX4、Gazebo/GZ、传感器驱动和可视化是适配层，不能重新解释或绕过核心安全状态。

## 接口责任

| 外部系统 | 适配输入 AURORA | 适配输出 | 必须保持的语义 |
|---|---|---|---|
| 视觉/深度/点云驱动 | `PointCloud2`、`Image` + `CameraInfo` 或检测消息、header、frame | 地图/检测输入 | TF 失败、坏编码、坏标定和坏消息不能刷新有效信息时间 |
| PX4/飞控 | 位置/速度回执、执行时钟、拒绝/故障回执 | 已验证轨迹的控制目标 | 只接受 `VALIDATED` 且 `safety_report.accepted=true` |
| Gazebo/GZ | 传感器、动力学、仿真时钟 | ROS 2 话题和回执 | 时钟回退、传感器掉线和飞控拒绝可注入并可观测 |
| `ros_gz_bridge` | GZ/ROS 消息桥接 | ROS 2 标准/自定义消息 | 桥接不改变 header 时间和坐标系 |

## PX4/GZ 推荐组合

可选组合为 PX4 SITL + Gazebo/GZ + `px4_msgs`/`px4_ros_com`，必要时使用 `ros_gz_bridge`。也可评估 MAVROS 2，但无论采用哪条链路，都不能将 `Trajectory` 直接当作未认证的控制器命令。

建议的适配器分层：

```text
PX4/GZ topics
    -> aurora_flight_adapter
       (frame/time/QoS/ack conversion)
    -> validated trajectory consumer
    -> PX4 setpoint or trajectory interface
```

`aurora_flight_adapter` 已提供协议无关的共同接纳层。具体适配器需要记录轨迹 ID、segment 起止时间、发送时间、飞控回执、当前状态时间和拒绝原因；不得绕过接纳层直接消费原始 ROS 轨迹。飞控拒绝只能产生 `REJECTED_UNSAFE`/降级诊断，不能被误报为轨迹执行成功。

## 验收顺序

1. 首先只发送人工构造且已经 `VALIDATED` 的静态三维轨迹，验证消息字段、坐标系、时间窗口和飞控拒绝回执。
2. 接入软件在环执行器，验证轨迹段接续、状态反馈、垂向运动、时间回退和更新间隔 watchdog。
3. 接入 PX4/GZ 三维动力学，验证连续局部重规划和执行误差反馈，不先引入真实传感器噪声。
4. 分别注入点云掉线、TF 缺失、动态预测过期、定位质量下降、时钟回退、轨迹过期和飞控拒绝。
5. 只有每一种异常都进入可解释降级/急停，且不存在未验证轨迹到达飞控的证据后，才可标记 SITL 阶段完成。

## 当前状态

`aurora_flight_adapter` 和 `aurora_sim` 已完成协议无关接纳加确定性三维软件在环执行边界，并有真实 DDS 规划器到执行器测试；当前环境未安装 PX4、Gazebo 或 GZ，因此本项目没有真实 SITL 通过证据，也没有具体 PX4/MAVROS 2/GZ 发送节点。安装外部后端后必须记录版本、启动命令、世界/机型参数、仿真时钟策略和完整故障矩阵。
