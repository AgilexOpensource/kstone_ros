# kstone_ros interfaces

[中文](interfaces.zh-CN.md)

Reference for the topics, services, state fields and parameters of `kstone_node`. Back to the [README](../README.md).

Names are private to the node: a node at `/left/kstone` resolves `~/state` to `/left/kstone/state`. Remapping is supported, and `~/discover` reports the resolved names, types, frames and QoS. `M` is a module ID from the YAML. Topics are created for `frame_target`, `chassis` and `joint_target` modules.

## Driver topics

| Topic | Type | Description |
|---|---|---|
| `~/joint_states` | Sub `sensor_msgs/JointState` | Measured joint positions from the driver |
| `~/odom` | Sub `nav_msgs/Odometry` | Measured world-to-root pose, or control-center pose (see `center_frame`); free root only |
| `~/feedback/joint_states` | Pub `sensor_msgs/JointState` | Joint position commands in URDF tree order, not measured feedback |
| `~/feedback/cmd_vel` | Pub `geometry_msgs/Twist` | Chassis command in the chassis body frame |

Driver outputs depend on real feedback, so the robot never jumps to a stale model target when the driver connects:

- `~/feedback/joint_states` stays silent until fresh `~/joint_states` covers every unlocked joint. `~/feedback/cmd_vel` stays silent until fresh `~/odom` arrives.
- Attaching takes `feedback_timeout_s` of continuously fresh feedback. During that window the measured state overwrites the model command every cycle, and the first cycle also resets module targets to it, so the first published command starts from the measured state. Enabled modules stay enabled.
- Joints and base attach independently. The free root is its own engine feedback group `base` unless the YAML `feedback_groups` places it.
- When feedback goes stale for `feedback_timeout_s`, that output detaches: joint commands stop, and `cmd_vel` sends zero for one `command_timeout_s`, then stops. Fresh feedback attaches it again.
- The engine keeps running without feedback; there is no dry-run mode. To test without hardware, publish `~/joint_states` and `~/odom` from a simulator.

## Engine topics

| Topic | Type | Description |
|---|---|---|
| `~/state` | Pub `std_msgs/String` | JSON snapshot, faults, accepted targets and request outcomes; latched |
| `~/discover` | Pub `std_msgs/String` | JSON list of modules and interfaces; latched |
| `~/control/joint` | Sub `sensor_msgs/JointState` | Position targets for every `joint_target` module, routed by joint name |

`~/control/joint` rules:

- Use `name` and `position` in any order. `header.frame_id` must be empty; velocity and effort are ignored.
- Any subset of joints is accepted; omitted joints keep their module's current target.
- Names outside every `joint_target` module are ignored with a throttled warning. Values are clamped to the module limits.
- Duplicate names, a name/position count mismatch, non-finite values or ambiguous ownership reject the whole message.

## Module topics

| Topic | Type | Description |
|---|---|---|
| `~/M/control/pose` | Sub `geometry_msgs/PoseStamped` | `frame_target` target, or omni chassis planar pose target |
| `~/M/control/cmd_vel` | Sub `geometry_msgs/Twist` | Chassis body-frame velocity (omni or differential) |
| `~/M/feedback/pose` | Pub `geometry_msgs/PoseStamped` | Current frame pose or chassis control-center estimate, not the target |

- Frame targets and feedback use the module's configured reference frame. Chassis pose targets and feedback use `world_frame`.
- A chassis pose target must have zero z and a yaw-only rotation.
- Pose inputs must use the frame listed in `~/discover`; an empty `frame_id` means that frame. There is no TF conversion.
- Quaternions must be finite and nonzero; they are normalized.
- Twist uses only `linear.x`, `linear.y` and `angular.z`; a differential chassis also ignores `linear.y`. Velocities over the chassis limits are limited, with a throttled warning.
- The latest accepted target wins per module and holds until replaced, as with `kstone run`. Only port timeouts, such as chassis velocity, lapse.

## Services

| Service | Type | Description |
|---|---|---|
| `~/home` | `std_srvs/Trigger` | Run kstone `home_all` and zero the chassis velocity |
| `~/M/home` | `std_srvs/Trigger` | Home one homeable module |
| `~/M/enable` | `std_srvs/SetBool` | Enable or disable one module |

```bash
ros2 service call /kstone/tool_pose/home std_srvs/srv/Trigger "{}"
ros2 service call /kstone/tool_pose/enable std_srvs/srv/SetBool "{data: false}"
```

- `~/home` homes the modules in control, the ones that last received a target. Parts that nobody controls, or that a non-homeable module controls, stay put; `~/M/home` still homes them.
- Homing moves to the YAML home positions. Disabled modules may reject home, and `~/home` is not atomic.
- The response `accepted request_id=N` means queued, not finished. Follow the result in `~/state` as described below.
- Stop target publishers before homing; `outcomes` keeps 32 entries, and a target stream quickly pushes service results out.

## State fields

`~/state` is a JSON object with `schema_version` `1`, published at `state_rate_hz`.

| Field | Description |
|---|---|
| `fresh` | Every feedback group was within `feedback_timeout_s` in the last cycle |
| `joints_attached`, `base_attached` | Joint and chassis driver outputs are attached; `base_attached` is `false` without a free root |
| `written` | A safe command was written this cycle and the snapshot is not stale |
| `quality` | `nominal`, or `degraded` when a stage held, was inexact or the step was clipped |
| `fault` | Empty when healthy, otherwise a reason such as `driver_stale_feedback`, `held_or_stale`, `stopped`, `duration_complete` |
| `expired_modules` | Motion modules disabled by a fault or feedback loss; a new target reactivates them |
| `modules.M` | `enabled`, `running`, `held_by`, `home_state`, `ports`, `outputs` |
| `outcomes` | Last 32 request results `{request_id, applied, error}` |
| `cycles`, `failed_cycles` | Engine cycle count and failed cycles |
| `engine_time` | Engine logical time in seconds |
| `snapshot_stamp` | Snapshot time in seconds, node clock |
| `generation` | Increments on fault, recovery or reattach; requests from an older generation are discarded |
| `lifecycle` | `0` not started, `1` running, `2` engine thread failed, `3` stopped |
| `rejected_requests` | Requests rejected because the engine was not running or the queue was full |
| `submitted_inputs` | Topic inputs submitted to the engine |
| `dropped_logs`, `truncated_logs` | Dropped and truncated log entries |

In `modules.M`, `running` means the module was in this cycle's solve, `held_by` is the preempting module ID or `null`, and `home_state` is `null` for modules that cannot home.

`home_state` values are `idle`, `waiting` (queued), `executing`, `succeeded`, `timed_out` (no progress within `timeout_s`) and `aborted` (disabled, skipped cycle or resync). A final state persists until the next home request.

To check that homing finished:

1. The service returns `accepted request_id=N`.
2. Find `request_id` `N` in `outcomes`. If `applied` is `false`, `error` gives the reason.
3. Follow `modules.M.home_state`: `waiting`, `executing`, then `succeeded`. `timed_out` or `aborted` means it failed.

After `~/home`, watch only the modules that passed through `waiting` or `executing`; the others keep their previous `home_state`.

## Node parameters

All parameters are read at startup. Defaults are in [bridge.yaml](../config/bridge.yaml); the launch arguments are listed in the [README](../README.md#use-your-own-model).

| Parameter | Default | Description |
|---|---|---|
| `config_file` | empty (launch: `config/omni.yaml`) | kstone engine YAML |
| `config_variables` | `{}` | JSON object of string values for `${name}` in the YAML |
| `duration` | `0.0` | Engine logical-time limit in seconds; `0` runs until shutdown |
| `realtime` | unset | Optional bool; `true` applies the YAML `thread` block, `false` ignores it, unset uses the YAML |
| `topic_prefix` | `~` | Prefix of every topic and service name |
| `world_frame` | `world` | World frame |
| `base_frame` | `base_link` | Model root frame |
| `frame_prefix` | empty | Prepended to every frame ID, like `robot_state_publisher` |
| `center_frame` | empty | `~/odom` child frame that means the chassis control center |
| `chassis_module` | empty | Chassis module ID; empty selects it automatically |
| `command_timeout_s` | `0.5` | Maximum command age when the engine applies it |
| `feedback_timeout_s` | `0.25` | Maximum driver feedback age that keeps outputs attached |
| `state_rate_hz` | `30.0` | `~/state` publish rate |
| `joint_command_topic` | `~/feedback/joint_states` | Extra topic for joint commands |
| `base_command_topic` | `~/feedback/cmd_vel` | Extra topic for chassis commands |
| `feedback_joint_topic` | `~/joint_states` | Extra joint feedback topic |
| `feedback_base_topic` | empty | Extra root pose feedback topic, `geometry_msgs/PoseStamped` |
| `publish_tf` | `false` | Publish the world-to-root TF |
| `debug_port` | `0` | kstone DebugServer port for `kstone-viewer`; `0` is off |
| `debug_host` | `127.0.0.1` | DebugServer listen address |
| `debug_token` | empty | DebugServer token; empty falls back to `$KSTONE_TOKEN` |

- Feedback and command stamps that are too old or in the future are rejected. A zero stamp uses the reception time.
- `command_timeout_s` is shortened by a smaller positive port timeout from the YAML.
- With `center_frame` set, `~/odom` messages with that `child_frame_id` are converted with the chassis YAML `center_in_root`; messages with `base_frame` as child stay root poses.
- The chassis body frame is the chassis module's YAML `base_frame`.
- The driver topic parameters add a second topic; `~/joint_states`, `~/feedback/joint_states` and `~/feedback/cmd_vel` always exist.
- `publish_tf` requires a free root and different `world_frame` and `base_frame`.
- When realtime is on and the YAML `thread` block needs higher rtprio or memlock limits, the engine runs `sudo prlimit` on itself and asks for a password on the terminal. Without a terminal it warns and continues.
- A non-loopback `debug_host` requires a token. Launch `debug_port:=7311` overrides the params file.
- `~/M/feedback/pose`, driver commands and TF follow the engine rate in the YAML (`rate_hz`).

## QoS

| Interfaces | QoS |
|---|---|
| `~/state`, `~/discover` | Reliable, transient local, depth 1 |
| Targets, pose feedback, driver commands | Reliable, volatile, depth 10 |
| `~/joint_states`, `~/odom`, extra feedback topics | Sensor data (best effort, volatile, depth 5) |
| Services | ROS defaults |

## Faults

- A fault or feedback loss disables motion modules; a fresh target rearms them. A module disabled through `~/M/enable` needs an explicit enable.
- Missing feedback gates the outputs. Fresh, complete feedback restores solving without replaying cleared targets.
- A latched engine or scheduler fault cannot be cleared by enable or home. Fix the cause, restart the node, then send fresh feedback and commands.
- `duration` completion keeps publishing state but stops commands; restart the node for another run.
