# AURORA-Planner 演示材料

这些截图来自本项目实际运行的 ROS 2 节点和确定性三维软件在环执行器，不是手工绘制的示意图。采集脚本先发送三维规划请求，记录 `VALIDATED` 轨迹，再注入 CV 动态目标并记录风险门控结果。

| 图片 | 展示内容 |
|---|---|
| [总览](aurora-demo-overview.png) | 三维规划、动态风险和执行闭环总览 |
| [三维规划](aurora-3d-planning.png) | 参考航点、验证后的 B-spline 轨迹和地图观测射线端点 |
| [动态风险](aurora-dynamic-risk.png) | CV 动态障碍物预测、3-sigma 不确定性包络和风险拒绝点 |
| [三维执行](aurora-3d-execution.png) | 软件在环中的期望位姿、实际状态和位置跟踪误差 |

## 复现

在 Ubuntu 24.04 + ROS 2 Jazzy 环境中执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 tools/demo/generate_demo_figures.py --output demo
```

脚本会把节点日志写入 `/tmp/aurora_demo_planner.log` 和 `/tmp/aurora_demo_sim.log`，并将采集元数据保存为 [`demo_manifest.json`](demo_manifest.json)。当前项目提供轨迹、风险和执行状态话题，但尚未实现独立的 `aurora_viz`/RViz Marker 节点；这些 PNG 是用于材料申报的离线渲染结果。图中结果对应 Jazzy 软件在环，不代表 PX4、Gazebo/GZ 或真实飞行验收。
