# ADR 0002: Static Replanning FSM and Failure Fallback

- 日期：2026-08-31
- 状态：已确认并实现第一版
- 范围：`aurora_planner_core` 的纯 C++ 静态规划阶段

## 背景

AURORA 需要将局部规划请求、执行中的轨迹、地图变化和规划失败组织成可验证的生命周期。仅由 ROS 回调分别处理这些事件，容易在规划失败、旧轨迹过期或信息异常时产生不一致动作，因此先在核心层定义确定性的状态机策略。

## 决策

采用以下状态：

`INIT`、`WAIT_TARGET`、`GENERATE`、`EXECUTE`、`REPLAN`、`DEGRADED`、`EMERGENCY_STOP`。

触发优先级固定为：

1. 当前轨迹碰撞
2. 安全信息过期
3. 优化或验证失败
4. 地图更新
5. 轨迹接近结束
6. 局部目标过期

规划器和 FSM 分离。FSM 输出动作，调用方执行 `StaticLocalPlanner::plan`，再通过 `onPlanningResult` 回传结果。成功且轨迹已验证时进入 `EXECUTE`；终点到达时进入 `WAIT_TARGET`。

连续规划失败时，只有当前轨迹仍通过外部安全检查且剩余时间超过急停阈值，才允许继续使用旧轨迹。失败次数达到默认阈值 3 次后进入 `DEGRADED` 并请求保持位置；无安全回退或剩余时间不足时进入锁存的 `EMERGENCY_STOP`。退出急停必须显式调用 `reset()`。

## 影响

- 状态转移可以脱离 ROS 进行单元测试和离线回放。
- `safety_information_stale` 和 `planning_failed` 是跨模块信号，不代表预测或风险算法已经实现。
- 旧轨迹是否安全由当前阶段的静态监视器或后续风险门控提供；FSM 不自行假设轨迹安全。
- 统一发布前安全门控、急停 ROS 2 输出和动态时空风险属于后续阶段。

## 验证

`test_static_local_planner` 覆盖初始等待、生成、执行、碰撞优先级、失败回退、降级悬停、剩余时间耗尽急停和无回退立即急停。
