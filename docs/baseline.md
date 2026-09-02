# EGO 静态复现基线

## 1. 基线身份

- 参考仓库：`../ego-planner`
- 参考分支：`master`
- 参考 commit：`bfda51284c8c1b476043255a8145ef925a3778a5`
- commit 时间：`2025-03-08T17:23:46+08:00`
- 复现入口：`../ego-planner/standalone/ego_repro.cpp`
- 复现器性质：已有的缩小版、确定性 C++ 复现器，不是完整 EGO-Planner 实现
- 复现器未被修改；生成的可执行文件已清理

## 2. 已覆盖的行为

当前复现器包含：

- 三维规则体素地图和越界即占据查询；
- 盒状障碍物及安全 margin；
- 三维 26 邻域 A*，含线段采样碰撞检查；
- A* 路径重采样为 B-spline 控制点；
- 三次均匀 B-spline 位置评估；
- 局部最近障碍物查询和无 ESDF 障碍势能；
- 平滑、参考拟合、速度/加速度超限代价；
- 基于梯度和线搜索的控制点优化；
- 轨迹采样、碰撞统计和 CSV 输出。

它没有覆盖完整上游的 ROS 消息、深度图融合、实时局部地图维护、全局最小 snap 轨迹、FSM、轨迹服务、飞控和真实仿真。因此本记录只能作为算法数学和局部避障的第一层基线。

## 3. 可重复命令

在上游复现器目录执行：

```bash
make test
make all
./ego_repro --output /tmp/aurora_ego_repro_baseline.csv --iterations 180
make clean
```

编译参数：`g++ -std=c++17 -O2 -Wall -Wextra -pedantic`。

## 4. 固定场景和参数

| 项目 | 值 |
|---|---:|
| 地图范围 | `[-10,-8,0]` 到 `[10,8,8]` m |
| 体素分辨率 | `0.5` m |
| 障碍物 | `x=[-0.7,0.7]`, `y=[-3,3]`, `z=[0,4]` m |
| 障碍物 margin | `0.35` m |
| 起点 | `[-7,-5,1]` m |
| 终点 | `[7,5,1]` m |
| B-spline 阶数 | 3 |
| 控制点数 | 31 |
| 时间间隔 | `0.25` s |
| 碰撞余量 | `0.65` m |
| 最大速度 | `3.0` m/s |
| 最大加速度 | `6.0` m/s^2 |
| 优化迭代 | 180 |
| 轨迹采样数 | 240 |

## 5. 运行结果

| 指标 | 初始轨迹 | 优化后 |
|---|---:|---:|
| 总代价 | 79.9606 | 0.0775 |
| 采样碰撞点 | 2 | 0 |
| 轨迹长度 | 19.4925 m | 19.8678 m |
| A* 路径点 | 35 | 不变 |
| B-spline 控制点 | 31 | 31 |

结论：当前复现器测试返回 `PASS`，可以作为 AURORA 后续纯 C++ 核心的行为参考之一。它的障碍势能、梯度、边界处理和优化策略仍需在 AURORA 中拆分成可独立测试的模块，不能直接视为 EGO 原算法的逐项等价实现。

## 6. 本机环境记录

已检查环境：

- OS：Ubuntu 24.04.4 LTS，代号 `noble`；
- 编译器：GCC 13.3.0；
- `make`：可用；
- CMake：3.28.3；
- Eigen3：3.4.0；
- GoogleTest：系统库和 `ament_cmake_gtest` 可用；
- ROS 2：Jazzy，安装于 `/opt/ros/jazzy`；
- ROS 1：当前环境未发现，也不是 AURORA 目标运行时。

因此已验证两条路径：上游缩小复现器不依赖 ROS 的 C++17 构建，以及 AURORA `aurora_math` 在 ROS2 Jazzy/ament 下的构建和测试。当前 shell 默认将 Conda Python 放在 PATH 前面，而 ament 的包解析需要系统 Python 的 `catkin_pkg`；可复现构建时应使用：

```bash
PATH=/usr/bin:/bin:/opt/ros/jazzy/bin
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## 7. 与完整 EGO 参数的对应关系

上游 `plan_manage/launch/advanced_param.xml` 中可作为后续对照的参数包括：

- 地图分辨率 `0.1` m，地图尺寸由 launch 参数给出；
- 局部更新范围约为 `5.5, 5.5, 4.5` m；
- 膨胀参数约为 `0.099` m；
- `p_hit=0.65`、`p_miss=0.35`、`p_min=0.12`、`p_max=0.90`、`p_occ=0.80`；
- 最大速度 `2.0` m/s、最大加速度 `3.0` m/s^2；
- 规划 horizon `7.5` m，规划时间 horizon `3` s；
- 最大 jerk `4`，控制点间距 `0.4` m；
- 优化权重 `lambda_smooth=1.0`、`lambda_collision=0.5`、`lambda_feasibility=0.1`、`lambda_fitness=1.0`。

这些参数不能直接覆盖缩小复现器的参数。进入 AURORA 后，需要先建立参数映射表并通过同场景测试校准。

## 8. 基线限制和下一步

- 当前只有一个绕墙场景，不能证明多场景鲁棒性；
- 当前地图是直接标记占据，不包含 log-odds 射线融合；
- 当前优化器是简化梯度下降，不是上游 `lbfgs.hpp` 的完整复现；
- 当前使用离散采样作碰撞判定，不能替代连续时间安全验证；
- 当前没有轨迹时间戳、动态目标、协方差和风险字段。

当前已补充 EGO 风格 log-odds 射线融合：支持命中/未命中、最大量程、地图边界裁剪、概率上下界、观测年龄和置信度；`test_voxel_map` 的 9 个测试已通过。

下一小步：确认三维 26 邻域 A* 的邻域代价、启发函数、起终点策略、超时和失败接口；随后创建纯 C++ `aurora_search` 包。
