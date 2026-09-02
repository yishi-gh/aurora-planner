# ADR 0004: Kinematic Prediction Baseline

- 日期：2026-09-01
- 状态：已确认并实现第一版
- 范围：`aurora_msgs` 动态目标批量接口和 `aurora_prediction`

## 决策

动态目标输入使用 `aurora_msgs/msg/DynamicObstacleTrackArray` 批量封装，每个目标保留独立的 `track_id`、外形、存在概率、位置/速度/加速度和状态协方差。每个目标通过 `prediction_model` 选择单一模型：

- `CV`：六维 `[position, velocity]` 状态，使用连续白噪声加速度模型传播；
- `CA`：九维 `[position, velocity, acceleration]` 状态，使用连续白噪声 jerk 模型传播。

当前输入消息提供 `[position, velocity]` 的 6x6 协方差。CA 模型的加速度协方差采用显式参数化的保守默认值，输出统一为位置/速度 6x6 边缘协方差，并在结果中标记该缺省事实。

预测器固定输出当前时刻状态和 horizon 末端状态，中间按固定采样间隔生成；horizon 末端使用精确时间步长，不静默超出配置上限。缺失协方差使用配置的非零默认方差；输入协方差必须对称且半正定，只有数值容差内的微小负特征值允许被截断并记录为 regularized。

## 原因

CV/CA 是可解释、可单元测试且易于在预测失效时回退的第一版基线。按目标选择单一模型可以先验证模型差异和风险传播，避免在没有数据关联和校准数据前引入 IMM 或学习模型。

## 后果

- 本阶段只实现运动学预测，不包含检测数据关联、轨迹生命周期管理、遮挡重捕获或 ROS 2 节点接线。
- 6x6 传输协方差不能表达 CA 加速度不确定性，相关信息在核心内部使用默认值并必须进入诊断；后续若需要完整表达再升级消息版本。
- 过程噪声、默认方差、采样间隔和最大 horizon 必须在风险数据集上校准，不能直接视为飞行安全阈值。

## 验证

`test_kinematic_predictor` 覆盖 CV/CA 均值传播、精确 horizon、协方差增长、缺省协方差、半正定修正、非对称/非半正定输入、非法 horizon 和采样上限。Ubuntu 24.04 + ROS 2 Jazzy 全量回归为 8 个包、75 个测试条目，0 errors、0 failures、0 skipped。
