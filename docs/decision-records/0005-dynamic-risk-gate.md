# ADR 0005: Dynamic Conservative Risk Gate

- 日期：2026-09-01
- 状态：已确认并实现第一版
- 范围：`aurora_risk` 与 `aurora_ros` 动态目标门控

## 决策

第一版动态目标由上游直接提供唯一 `track_id`，AURORA 不实现数据关联。输入使用 `/aurora/dynamic_obstacle_tracks`，一个批次使用 `DynamicObstacleTrackArray.header` 的统一时间和 `map.frame` 坐标系；目标 header 必须与批次一致，非法目标单独计数。

规划线程根据候选轨迹结束时刻，对快照中的每个 track 调用 `aurora_prediction::KinematicPredictor`。CV/CA 预测状态按绝对时间查询，禁止对预测窗口外的数据静默外推。候选 B-spline 使用固定配置间隔采样。

第一版采用保守 3-sigma 几何门控，不将结果解释为碰撞概率：

```text
safety_radius = vehicle_radius + obstacle_radius
                 + 3 * sqrt(lambda_max(relative_position_covariance))
clearance = center_distance - safety_radius
```

默认 `vehicle_radius=0.65 m`。障碍球直接使用半径；盒、胶囊和多球转换为包围球。包络相交时返回 `DYNAMIC_COLLISION`；预测缺失、快照过期、预测不覆盖候选时域或批次含非法目标时拒绝发布。

## 原因

该基线可解释、确定、无需训练数据，并且可以直接验证协方差增大导致安全包络增大。它保持动态时间信息独立于静态占据层，避免把运动目标错误写成永久障碍。

## 后果

- 第一版节点内集成预测和风险计算，后续可拆成独立节点而不改变核心契约。
- 目标存在概率当前只做输入合法性检查，尚未改变包络大小；存在概率软代价属于后续策略确认项。
- 当前风险值是归一化几何风险分数，不是经校准的碰撞概率，也没有风险梯度优化。
- 没有动态快照时默认拒绝；可通过 `risk.require_dynamic_information=false` 显式关闭这一要求。
- `PlanningResult` 增加 `RiskReport` 字段，使动态门控失败在没有轨迹时仍可诊断。

## 验证

`test_dynamic_risk_evaluator` 覆盖空快照、球/盒/胶囊/多球、3-sigma 协方差单调性、绝对时间插值、预测末端覆盖、缺失/过期/非法信息和参数校验。Ubuntu 24.04 + ROS 2 Jazzy 全量回归为 9 个包、89 个测试条目，0 errors、0 failures、0 skipped。
