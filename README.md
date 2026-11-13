# kstone_ros

[中文](README.zh-CN.md)

ROS 2 node for the [kstone](https://github.com/AgilexOpensource/kstone) whole-body IK engine: send end-effector poses, joint targets and chassis velocities over ROS topics, and get joint and chassis commands for your robot driver.

> **Safety:** Your driver must watch for missing commands and enforce its own limits and emergency stop. Zero commands from this node are not a hardware fail-safe.

## Requirements

Tested on Ubuntu 22.04 with ROS 2 Humble, Pinocchio 4.0.0, Coal 3.0.3 and kstone 0.0.1.

## Install

Build the engine and the node in one workspace:

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

A plain `colcon build` is enough: both packages build in Release by default, and kstone skips its tests, tools and examples when built by colcon. To use an engine you installed elsewhere, or to run the package tests, see [docs/install.md](docs/install.md).

## Quick start without hardware

The shipped [omni.yaml](config/omni.yaml) and [omni.urdf](config/omni.urdf) describe an omnidirectional base, three tool slide axes and a gripper, running on a loopback robot. Start the node:

```bash
source ~/kstone_ws/install/setup.bash
ros2 launch kstone_ros kstone.launch.py
```

In each new terminal, source `~/kstone_ws/install/setup.bash` first. Echo and `-r 10` commands run until Ctrl+C.

Inspect state and the available interfaces:

```bash
ros2 topic echo /kstone/state --qos-durability transient_local
ros2 topic echo /kstone/discover --qos-durability transient_local
```

Move the tool in `base_link`:

```bash
ros2 topic pub -r 10 /kstone/tool_pose/control/pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.1, z: 0.4}, orientation: {w: 1.0}}}"
```

Set the gripper position:

```bash
ros2 topic pub -r 10 /kstone/control/joint sensor_msgs/msg/JointState \
  "{name: [finger_joint], position: [0.03]}"
```

Drive the chassis in its body frame:

```bash
ros2 topic pub -r 10 /kstone/chassis/control/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.2}}"
```

Driver outputs (`~/feedback/joint_states`, `~/feedback/cmd_vel`) stay silent until a driver or simulator publishes fresh `~/joint_states` (and `~/odom` for a free root).

## Interfaces

Names are private to the node: with the default node name `kstone`, `~/state` is `/kstone/state`. `M` is a module ID from the YAML.

| Name | Kind | Description |
|---|---|---|
| `~/joint_states` | Sub `sensor_msgs/JointState` | Joint feedback from the driver |
| `~/odom` | Sub `nav_msgs/Odometry` | Base odometry from the driver (free root only) |
| `~/feedback/joint_states` | Pub `sensor_msgs/JointState` | Joint position commands for the driver |
| `~/feedback/cmd_vel` | Pub `geometry_msgs/Twist` | Chassis velocity command for the driver |
| `~/state` | Pub `std_msgs/String` | JSON engine state, latched |
| `~/discover` | Pub `std_msgs/String` | JSON list of modules and interfaces, latched |
| `~/control/joint` | Sub `sensor_msgs/JointState` | Targets for all `joint_target` modules |
| `~/M/control/pose` | Sub `geometry_msgs/PoseStamped` | Frame target, or omni chassis pose target |
| `~/M/control/cmd_vel` | Sub `geometry_msgs/Twist` | Chassis body-frame velocity |
| `~/M/feedback/pose` | Pub `geometry_msgs/PoseStamped` | Current pose of the frame or chassis |
| `~/home`, `~/M/home` | Srv `std_srvs/Trigger` | Home all controlled modules, or one module |
| `~/M/enable` | Srv `std_srvs/SetBool` | Enable or disable one module |

See [docs/interfaces.md](docs/interfaces.md) for feedback gating, target rules, homing, the state fields, all node parameters, QoS and fault handling.

## Use your own model

Copy the shipped YAML and URDF, replace the model, and configure modules, limits, rate and home positions. Module IDs must match `[A-Za-z_][A-Za-z0-9_]*`. The single chassis module is found automatically.

```bash
ros2 launch kstone_ros kstone.launch.py \
  config_file:=/absolute/path/control.yaml \
  config_variables:='{"urdf": "/absolute/path/model.urdf"}'
```

`config_variables` fills `${name}` placeholders in the YAML; the `urdf` key above only applies if your YAML uses `${urdf}`. URDF paths may be relative to the YAML.

| Argument | Default | Description |
|---|---|---|
| `config_file` | `config/omni.yaml` | kstone engine YAML |
| `params_file` | `config/bridge.yaml` | Node parameters file |
| `config_variables` | `{}` | JSON object of string values for `${name}` |
| `duration` | `0.0` | Engine logical-time limit in seconds; `0` runs until shutdown |
| `realtime` | `inherit` | `true` applies the YAML `thread` block, `false` ignores it |
| `debug_port` | `inherit` | kstone DebugServer port for `kstone-viewer`; `0` is off |
| `namespace` | empty | Node namespace, for running several nodes |
| `use_sim_time` | `false` | Use `/clock` |

Default paths are in this package's share directory. `inherit` keeps the value from the YAML or the params file. Other node parameters (frames, timeouts, driver topic names) go in a copy of [bridge.yaml](config/bridge.yaml) passed as `params_file`.

## Troubleshooting

- Package `kstone_ros` not found: source `/opt/ros/humble/setup.bash` and `install/setup.bash` in this terminal.
- `find_package(kstone)` fails: initialize the engine submodules with `git -C src/kstone submodule update --init --recursive`, or point `CMAKE_PREFIX_PATH` at an installed engine ([docs/install.md](docs/install.md)).
- Crash or undefined symbols at startup: do not mix pip or Conda Pinocchio/Coal with the ROS packages. Rebuild kstone and kstone_ros against the same libraries and check them with `ldd "$(ros2 pkg prefix kstone_ros)/lib/kstone_ros/kstone_node"`.
- Robot does not move: check that `joints_attached` and `base_attached` in `~/state` are `true`; if not, publish fresh `~/joint_states` and `~/odom`.
- Targets rejected: compare frames and joint names with `~/discover`, and read `fault` and `outcomes` in `~/state`.

## License

Apache-2.0, see [LICENSE](LICENSE).
