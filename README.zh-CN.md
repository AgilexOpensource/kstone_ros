# kstone_ros

[English](README.md)

[kstone](https://github.com/AgilexOpensource/kstone) 全身 IK 引擎的 ROS 2 节点：通过 ROS 话题发送末端位姿、关节目标和底盘速度，并为机器人驱动输出关节与底盘命令。

> **安全：** 驱动须自行监测命令缺失，并执行自己的限位和急停。本节点发出的零命令不是硬件故障保护。

## 环境要求

已在 Ubuntu 22.04、ROS 2 Humble、Pinocchio 4.0.0、Coal 3.0.3 和 kstone 0.0.1 上测试。

## 安装

在同一个工作空间中构建引擎和节点：

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install ros-humble-pinocchio ros-humble-coal nlohmann-json3-dev
mkdir -p ~/kstone_ws/src && cd ~/kstone_ws
git clone https://github.com/AgilexOpensource/kstone_ros.git src/kstone_ros
git clone --recurse-submodules \
  https://github.com/AgilexOpensource/kstone.git src/kstone
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to kstone_ros
source install/setup.bash
```

直接 `colcon build` 即可：两个包默认以 Release 构建，kstone 由 colcon 构建时不编译测试、工具和示例。使用其他位置安装的引擎或运行本包测试，见 [docs/install.zh-CN.md](docs/install.zh-CN.md)。

## 无硬件快速上手

随包的 [omni.yaml](config/omni.yaml) 和 [omni.urdf](config/omni.urdf) 描述一个全向底盘、三个工具平移轴和一个夹爪，运行在 loopback 机器人上。启动节点：

```bash
source ~/kstone_ws/install/setup.bash
ros2 launch kstone_ros kstone.launch.py
```

每个新终端先执行 `source ~/kstone_ws/install/setup.bash`。echo 和 `-r 10` 命令会一直运行，按 Ctrl+C 停止。

查看状态和可用接口：

```bash
ros2 topic echo /kstone/state --qos-durability transient_local
ros2 topic echo /kstone/discover --qos-durability transient_local
```

在 `base_link` 中移动工具：

```bash
ros2 topic pub -r 10 /kstone/tool_pose/control/pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.1, z: 0.4}, orientation: {w: 1.0}}}"
```

设置夹爪位置：

```bash
ros2 topic pub -r 10 /kstone/control/joint sensor_msgs/msg/JointState \
  "{name: [finger_joint], position: [0.03]}"
```

在底盘机体坐标系中控制速度：

```bash
ros2 topic pub -r 10 /kstone/chassis/control/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.2}}"
```

驱动或仿真器发布新鲜的 `~/joint_states`（自由根还需 `~/odom`）之前，驱动输出（`~/feedback/joint_states`、`~/feedback/cmd_vel`）不发布。

## 接口

名称为节点私有：默认节点名 `kstone` 时，`~/state` 即 `/kstone/state`。`M` 是 YAML 中的模块 ID。

| 名称 | 类别 | 说明 |
|---|---|---|
| `~/joint_states` | 订阅 `sensor_msgs/JointState` | 驱动的关节反馈 |
| `~/odom` | 订阅 `nav_msgs/Odometry` | 驱动的底盘里程计（仅自由根） |
| `~/feedback/joint_states` | 发布 `sensor_msgs/JointState` | 发给驱动的关节位置命令 |
| `~/feedback/cmd_vel` | 发布 `geometry_msgs/Twist` | 发给驱动的底盘速度命令 |
| `~/state` | 发布 `std_msgs/String` | JSON 引擎状态，锁存 |
| `~/discover` | 发布 `std_msgs/String` | JSON 模块与接口列表，锁存 |
| `~/control/joint` | 订阅 `sensor_msgs/JointState` | 所有 `joint_target` 模块的目标 |
| `~/M/control/pose` | 订阅 `geometry_msgs/PoseStamped` | frame 目标，或底盘位姿目标 |
| `~/M/control/cmd_vel` | 订阅 `geometry_msgs/Twist` | 底盘机体坐标系速度 |
| `~/M/feedback/pose` | 发布 `geometry_msgs/PoseStamped` | frame 或底盘的当前位姿 |
| `~/home`、`~/M/home` | 服务 `std_srvs/Trigger` | 让所有受控模块或单个模块回零 |
| `~/home_cancel` | 服务 `std_srvs/Trigger` | 让所有正在回零的模块就地停下 |
| `~/M/enable` | 服务 `std_srvs/SetBool` | 启用或禁用一个模块 |
| `~/driver_io` | 服务 `std_srvs/SetBool` | 暂停或恢复与驱动的反馈和命令收发 |

反馈门控、目标规则、回零、state 字段、全部节点参数、QoS 和故障处理见 [docs/interfaces.zh-CN.md](docs/interfaces.zh-CN.md)。

## 接入自己的模型

复制随包 YAML 和 URDF，替换模型，并配置模块、限位、频率和回零位置。模块 ID 须匹配 `[A-Za-z_][A-Za-z0-9_]*`。唯一的底盘模块会自动识别。

```bash
ros2 launch kstone_ros kstone.launch.py \
  config_file:=/absolute/path/control.yaml \
  config_variables:='{"urdf": "/absolute/path/model.urdf"}'
```

`config_variables` 填充 YAML 中的 `${name}` 占位符；上面的 `urdf` 键仅在 YAML 使用 `${urdf}` 时生效。URDF 路径可相对于 YAML。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `config_file` | `config/omni.yaml` | kstone 引擎 YAML |
| `params_file` | `config/bridge.yaml` | 节点参数文件 |
| `config_variables` | `{}` | `${name}` 的字符串值 JSON 对象 |
| `duration` | `0.0` | 引擎逻辑时间上限（秒）；`0` 运行到关闭 |
| `realtime` | `inherit` | `true` 应用 YAML 的 `thread` 块，`false` 忽略它 |
| `debug_port` | `inherit` | 供 `kstone-viewer` 连接的 kstone DebugServer 端口；`0` 关闭 |
| `namespace` | 空 | 节点命名空间，用于运行多个节点 |
| `use_sim_time` | `false` | 使用 `/clock` |

默认路径位于本包的 share 目录。`inherit` 沿用 YAML 或参数文件中的值。其他节点参数（坐标系、超时、驱动话题名）写入 [bridge.yaml](config/bridge.yaml) 的副本，并通过 `params_file` 传入。

## 故障排查

- 找不到 `kstone_ros` 包：在当前终端加载 `/opt/ros/humble/setup.bash` 和 `install/setup.bash`。
- `find_package(kstone)` 失败：用 `git -C src/kstone submodule update --init --recursive` 初始化引擎子模块，或将 `CMAKE_PREFIX_PATH` 指向已安装的引擎（[docs/install.zh-CN.md](docs/install.zh-CN.md)）。
- 启动时崩溃或符号未定义：不要混用 pip 或 Conda 的 Pinocchio/Coal 与 ROS 包。用同一套库重新构建 kstone 和 kstone_ros，并用 `ldd "$(ros2 pkg prefix kstone_ros)/lib/kstone_ros/kstone_node"` 检查。
- 机器人不动：检查 `~/state` 中 `joints_attached` 和 `base_attached` 是否为 `true`；否则发布新鲜的 `~/joint_states` 和 `~/odom`。
- 目标被拒绝：对照 `~/discover` 检查坐标系和关节名，并查看 `~/state` 中的 `fault` 和 `outcomes`。

## 许可证

Apache-2.0，见 [LICENSE](LICENSE)。
