# 依赖与发行版矩阵

本文档区分 AURORA 的核心依赖、ROS 2 适配依赖和可选外部后端。核心算法不应因为安装 PX4、Gazebo 或某个传感器驱动而改变。

## 支持矩阵

| 平台 | ROS 2 | 编译器基线 | 状态 |
|---|---|---|---|
| Ubuntu 22.04 | Humble | GCC 11，C++17 | 目标平台；需 CI 证据 |
| Ubuntu 24.04 | Jazzy | GCC 13，C++17 | 本机已验证 |

两套平台必须使用匹配的 Ubuntu/ROS 2 组合。项目不支持在同一系统中混用 Humble 和 Jazzy 的环境变量或已编译产物。

## 核心构建依赖

| 依赖 | 用途 | 作用域 | 当前声明位置 |
|---|---|---|---|
| CMake >= 3.16 | 构建库和测试 | 必需 | 各包 `CMakeLists.txt` |
| C++17 编译器 | 核心实现 | 必需 | 各包 `CMakeLists.txt` |
| Eigen3 | B-spline、矩阵、协方差、几何 | 必需 | `eigen3_cmake_module` + `Eigen3` |
| `ament_cmake` | ROS 2 包构建 | 必需 | 各 `package.xml` |
| `ament_cmake_gtest` | C++ 单元测试 | 测试必需 | 各测试包 |
| GoogleTest | 单元测试运行时 | 测试必需 | ROS 2 vendor/系统包 |

核心库 `aurora_math`、`aurora_map`、`aurora_search`、`aurora_trajectory`、`aurora_prediction`、`aurora_tracking`、`aurora_risk` 和 `aurora_planner_core` 不 include `rclcpp`、`sensor_msgs` 或 ROS 时间类型。

## ROS 2 适配依赖

| 依赖 | 用途 | 必需条件 |
|---|---|---|
| `rclcpp` | 节点、参数、executor、时钟 | 运行 `aurora_ros`/`aurora_sim` |
| `aurora_flight_adapter` | 飞控轨迹接纳、B-spline 采样和反馈映射 | `aurora_sim` 与具体飞控适配器 |
| `rosidl_default_generators/runtime` | 自定义消息和服务 | 构建/运行 `aurora_msgs` |
| `builtin_interfaces`、`std_msgs`、`geometry_msgs` | 时间、header、几何和状态 | `aurora_msgs` |
| `sensor_msgs` | PointCloud2、Image、CameraInfo 输入 | `aurora_ros` |
| `tf2`、`tf2_ros`、`tf2_sensor_msgs` | 点云坐标变换 | `aurora_ros` |
| `ament_index_python` | 启动文件定位安装资源 | `aurora_bringup`/启动脚本 |
| `launch_ros`、`launch_testing_ament_cmake`、`launch_testing_ros` | 节点级集成测试 | 测试环境 |
| `rclpy`、`rosbag2_py` | Python 回放和 DDS 测试 | 测试环境 |

当前代码没有把 PCL、OpenCV、yaml-cpp、Armadillo、Boost、CUDA 或 RealSense 私有驱动带入规划核心。PointCloud2 的基础字段解析以及首版 `Image` + `CameraInfo` 针孔反投影均在 ROS 适配层完成；复杂点云过滤、深度校正和相机驱动仍可由 PCL/`depth_image_proc`/`image_pipeline` 作为外围前处理提供。

当前 13 个 AURORA `package.xml` 均声明 `GPL-3.0-only`；顶层许可文本和参考边界见 `LICENSE` 与 `NOTICE`。这只描述 AURORA 源包的声明，不替代目标系统依赖和可选后端的许可证核验。

## 安装与构建

在已经安装目标 ROS 2 的系统中：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
# 仅首次使用 rosdep 时执行
sudo rosdep init
rosdep update
sudo apt install python3-colcon-common-extensions python3-rosdep libeigen3-dev cppcheck
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -r -y
colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

本机 2026-09-02 的 `rosdep check` 未执行成功，因为系统 rosdep 尚未初始化；这不是包缺依赖的证据。需要管理员执行 `rosdep init` 后再运行 `rosdep update`，CI 容器中的依赖解析由工作流单独验证。

## CI 矩阵

`.github/workflows/ci.yml` 已定义 Humble/Ubuntu 22.04 和 Jazzy/Ubuntu 24.04 两个 job，步骤包括 rosdep、结构审计、构建、全量测试、结果汇总、核心/ROS 适配/飞控接纳/软件在环 C++ 静态分析、静态核心基准和软风险基准。当前仅有本机 Jazzy 证据；本机 Docker 没有 Humble 镜像，拉取官方 `ros:humble-ros-base` 于 2026-09-02 因访问 Docker Hub 超时失败，GitHub Actions 真实运行成功记录仍为 `OPEN`，不能用本机结果替代 Humble。

源包依赖图可用以下无第三方 Python 依赖的命令校验：

```bash
python3 tools/release/generate_source_sbom.py --check docs/source-sbom.spdx.json
```

该文件只描述当前 `package.xml` 的包和直接依赖；实际 apt/ROS 版本、可选后端和最终许可证必须在目标发行版环境中重新收集。

## 可选外部依赖

PX4/GZ、`px4_msgs`/`px4_ros_com`、`ros_gz_bridge`、PCL、OpenCV、`depth_image_proc`、OctoMap、nvblox、Ceres/NLopt/OSQP、Autoware 检测/跟踪组件均属于可选适配。引入前必须记录版本、许可证、运行时预算和回归结果，不能让可选库改变 AURORA 的轨迹验证和急停边界。
