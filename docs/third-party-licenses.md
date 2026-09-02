# 第三方许可证审计

本文档是发布前审计清单，不是法律意见。许可证结论必须以实际使用的源代码、二进制包和版本元数据为准；未核验项明确标记为 `OPEN`。

## AURORA 自有代码

| 范围 | 当前状态 | 发布动作 |
|---|---|---|
| 13 个 AURORA 包 | 已统一声明 `GPL-3.0-only` | 与顶层 `LICENSE`、`NOTICE` 和实际分发内容一起审计 |

顶层 `LICENSE` 是从记录的 EGO-Planner GPLv3 许可文本复用的标准许可文件；`NOTICE` 记录 AURORA 的版权声明、参考 commit 和源文件边界。`docs/source-sbom.spdx.json` 是根据当前 `src/*/package.xml` 生成的 SPDX 2.3 源包依赖图，可由结构审计工具校验；系统包版本、可选外部后端和未随源码分发的组件仍需在目标环境中单独核验。

当前 AURORA 的顶层许可选择和包元数据已经按用户确认完成；这不替代针对具体分发物的法律审查。

## 上游和可选依赖

| 组件 | 使用方式 | 当前记录 | 发布动作 |
|---|---|---|---|
| `ego-planner` | 只读算法参考；AURORA 当前代码未复制其 ROS 1 文件 | 上游仓库声明 GPLv3 | 保留参考 commit、核对是否存在实质复制，并决定兼容许可证 |
| ROS 2/ament/rclcpp/messages | 构建和运行时依赖 | 版本随目标发行版 | 从目标 ROS 发行版包元数据生成清单 |
| Eigen3 | 核心数学依赖 | 版本随系统/ROS 环境 | 核对实际包许可证并进入 NOTICE/SBOM |
| GoogleTest | 测试依赖 | ROS vendor 或系统包 | 发布源码包时确认是否随发行物分发 |
| PCL/OpenCV/depth_image_proc | 可选外围 | 当前核心不强制依赖 | 仅在启用适配器时审计 |
| PX4/GZ/bridge/MAVROS 2 | 可选外部后端 | 当前未安装/未验收 | 按实际版本和分发方式审计 |

## 审计流程

1. 已冻结 AURORA 顶层许可证和包许可证为 `GPL-3.0-only`，并替换所有临时 `TODO` 字段。
2. 已记录 `ego-planner` 参考 commit、参考方式和生成物边界；对任何直接复制代码仍必须保留上游版权和许可证要求。
3. 在 Humble 和 Jazzy 容器中收集 `ros2 doctor`、`dpkg-query`/包清单、外部源码版本和许可证文件。
4. 先用 `python3 tools/release/generate_source_sbom.py --check docs/source-sbom.spdx.json` 校验源包依赖图，再在 Humble/Jazzy 环境中收集安装包版本，生成包含运行时、测试时和可选依赖的最终 SPDX/SBOM。
5. 将 `LICENSE`、`NOTICE`、版权归属、README 许可证说明和所有 `package.xml` 一起检查后，才能创建发布标签。

## 当前开放项

许可证元数据和顶层文件已完成。实际安装环境的依赖版本/SBOM、可选外部后端许可证、Ubuntu 22.04 + ROS 2 Humble、PX4/GZ 和真实飞行验收仍为 `OPEN`；这些项目不应被本地 Jazzy 或确定性软件在环结果替代。
