# AURORA-Planner 性能基准

本文档记录可重复的核心规划性能基线。它不是实时飞行保证；真实部署仍需在目标 CPU、传感器负载、动态风险配置和飞控通信链路上重新测量。

## 核心静态规划

基准程序测量一次完整的纯 C++ `StaticLocalPlanner::plan` 调用，覆盖：

```text
局部目标提取 -> 3D 26 邻域 A* -> 弧长重采样
-> EGO B-spline 初值 -> 静态优化 -> 静态轨迹验证
```

固定输入为无 ROS 的三维自由体素地图（`32 x 20 x 12`，`0.5 m` 分辨率）、起点 `[-4, 0, 1]` m、目标 `[4, 0, 1]` m，局部距离 horizon `3.0 m`，优化最多 80 次迭代，验证每个 B-spline span 采样 16 次。测量前执行 10 次预热；每次使用独立请求 ID，地图和参数保持不变。

在 Ubuntu 24.04、GCC 13.3.0、ROS 2 Jazzy 本机于 2026-09-02 的一次测量结果：

| 指标 | 结果 |
|---|---:|
| 预热次数 | 10 |
| 测量次数 | 200 |
| 成功率 | 100% |
| 平均规划延迟 | 0.964 ms |
| P95 | 0.999 ms |
| P99 | 1.099 ms |
| 最大延迟 | 1.427 ms |

同一输入和构建配置下，启用核心软风险回调后的测量结果如下。该模式使用一个固定的、平滑的合成风险场和 `lambda_risk=0.15`，用于验证风险路径的开销与成功率；它不替代真实动态目标数据上的风险标定。

| 指标 | 静态基线 | 软风险 |
|---|---:|---:|
| 预热次数 | 10 | 10 |
| 测量次数 | 200 | 200 |
| 成功率 | 100% | 100% |
| 平均规划延迟 | 0.964 ms | 1.227 ms |
| P95 | 0.999 ms | 1.289 ms |
| P99 | 1.099 ms | 1.381 ms |
| 最大延迟 | 1.427 ms | 1.487 ms |
| 风险评估次数 | 0 | 2,238,600 |

软风险模式运行命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --base-paths src --packages-select aurora_planner_core \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DAURORA_BUILD_BENCHMARKS=ON
install/aurora_planner_core/lib/aurora_planner_core/aurora_static_planner_benchmark \
  --soft-risk --warmup 10 --iterations 200 \
  --output /tmp/aurora_soft_risk_benchmark.json
```

运行命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --base-paths src --packages-select aurora_planner_core \
  --cmake-args -DAURORA_BUILD_BENCHMARKS=ON
install/aurora_planner_core/lib/aurora_planner_core/aurora_static_planner_benchmark \
  --warmup 10 --iterations 200 --output /tmp/aurora_static_planner_benchmark.json
```

基准程序默认不启用动态软风险代价，也不包含 ROS DDS、PointCloud2 转换、TF 查询和飞控发布。动态风险开启后的延迟必须另行记录；任何算法或参数改变都应重新运行并比较 P95/P99、成功率和状态分布。

## 最新复核

2026-09-02 在同一 Ubuntu 24.04、GCC 13.3.0、ROS 2 Jazzy 环境中，以 10 次预热和 100 次测量重新运行：静态模式与软风险模式均为 100/100 成功。静态模式平均 0.956627 ms、P95 0.984530 ms、P99 0.991573 ms、最大 1.075223 ms；软风险模式平均 1.253466 ms、P95 1.365494 ms、P99 2.004979 ms、最大 2.007824 ms，共执行 1,172,600 次风险评估。该复核用于确认当前代码仍可运行；样本量较上一条 200 次记录小，不用于宣称性能提升。

同日重新执行了 13 包完整构建和允许 Fast DDS UDP 通信的串行全量测试，结果为 223 tests、0 errors、0 failures、0 skipped。该结果与本节 benchmark 使用同一工作区，但 benchmark 数值仍只代表纯 C++ 核心调用，不代表飞行实时性。

## 复核规则

重复复核时使用 10 次预热和固定测量次数，要求 JSON 可解析、成功率为 100%、P99 为有限值；软风险模式还应满足 `soft_risk_enabled=true` 且风险评估次数大于零。基准只用于确认构建和核心性能路径可运行，不把跨机器的绝对毫秒阈值当作硬门槛；目标硬件确定后再增加分层阈值。
