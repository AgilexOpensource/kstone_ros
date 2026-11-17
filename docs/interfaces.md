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

Driver outputs depend on real feedback, not a stale model initialization. Out-of-limit scalar synchronization uses actual Ruckig rather than an instantaneous position clip.

- `~/feedback/joint_states` carries only the joints whose feedback group is attached: it stays silent until fresh `~/joint_states` covers every unlocked joint of a group, and a group that goes stale (for example an unplugged hand in its own `feedback_groups` entry) drops out while the other groups keep publishing. `~/feedback/cmd_vel` stays silent until fresh `~/odom` arrives.
- Attaching takes `feedback_timeout_s` of continuously fresh feedback. During this settling window, synchronization tracks latest raw measurement; suppressed unattached internal writes do not advance correction. The first attached command for the synchronizing group equals raw positions (mimics derived) with zero commanded velocity. Only subsequent attached writes advance continuous `JointTrajectory` with guard-scaled velocity and default acceleration/jerk factors 2/10, without restarting on every fresh sample. Independent engine groups retain their own attachment lifecycle; suppression by an external relay after attachment remains the separate forwarding boundary below.
- Finite supported joint feedback is accepted and displayed unchanged even outside software position limits. Empty or unsupported-only names, mismatched lengths, duplicate names, NaN/Inf, invalid ages and out-of-order samples retain their rejection rules and atomicity. Modules driving/locking corrected joints stay enabled with constraints active, but their tasks and preparation are held and targets/FK/ramps rebase from current commanded state. Completion rebases affected modules to the legal posture. Unrelated modules/groups continue normally. In-range synchronization and command expiry rules remain unchanged; correction drops affected targets. Stale/reattach and generic pause/resume restart from latest raw feedback. Failed solve/guard/write cycles do not advance the profile; model limits are not relaxed.
- Normal IK, gaze, CoM/collision recovery and null-space resume after correction without a new target/home, subject to existing arbitration. No motion-intent gate or new service is introduced; feasibility and ordinary module takeover acceleration/jerk are not guaranteed by the startup profile. See kstone's `docs/robot.md`.
- **External relay boundary:** a closed relay can let correction finish without robot movement. Opening/reopening requires fresh synchronization at actual forwarding, coordinated with the first successful new command. Existing engine `set_paused(true)` then `set_paused(false)` provides this lifecycle; when enabled, the debug protocol exposes `pause` with `paused: true` then `false`. No new ROS service is added. Do not infer relay safety from cold-start completion or attachment; guardian gates and its temporary position-delta check remain untouched, and relay reopening is unverified.
- Coverage regression: `test_controller` runs the geometric fixture `test/fixtures/coverage.yaml` / `coverage.urdf`, which uses every built-in module type (chassis, frame_target, joint_target, null_space, gaze, center_of_mass, self_collision, world_collision) plus coupled mimics, locked wheels and per-module feedback groups, and depends on no robot description package. With joints outside their limits it verifies raw first engine commands at startup/reattach, unchanged measured FK, bounded position changes and recovery velocity/acceleration/jerk, and completion, with every automatic module still enabled. Core tests additionally compare actual Ruckig output and check module target/FK coherence, mimic coupling, rate changes and failure rollback. Robot packages check their own deployed configurations (for Lookie, lookie_bringup checks the driver bridge against its URDF).
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
- Messages that arrive within one control cycle are all applied, in order; a later message only overrides the joints it names. Each message is accepted or rejected as a whole.
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
| `~/driver_io` | `std_srvs/SetBool` | Pause or resume ROS-layer driver data exchange; the engine keeps running |

```bash
ros2 service call /kstone/tool_pose/home std_srvs/srv/Trigger "{}"
ros2 service call /kstone/tool_pose/enable std_srvs/srv/SetBool "{data: false}"
```

- `~/home` homes the modules in control, the ones that last received a target. Parts that nobody controls, or that a non-homeable module controls, stay put; `~/M/home` still homes them.
- Homing moves to the YAML home positions. Disabled modules may reject home, and `~/home` is not atomic.
- The response `accepted request_id=N` means queued, not finished. Follow the result in `~/state` as described below.
- Stop target publishers before homing; `outcomes` keeps 32 entries, and a target stream quickly pushes service results out.
- `~/driver_io` false drops driver feedback and stops joint commands at once; `cmd_vel` sends zero for one `command_timeout_s`, then goes silent. Feedback goes stale and the engine detaches after `feedback_timeout_s`.
- `~/driver_io` true fails while any feedback group is still attached. Once accepted, output restarts only after fresh feedback reattaches, and the first command equals that feedback.
- For a driver gate or motor enable change: pause, wait until `joints_attached` and `base_attached` are `false`, change the driver, make sure the motors are at rest, then resume. The node does not read driver status; a separate monitor node decides when to call this service.

## State fields

`~/state` is a JSON object with `schema_version` `1`, published at `state_rate_hz`.

| Field | Description |
|---|---|
| `fresh` | Every feedback group was within `feedback_timeout_s` in the last cycle |
| `driver_io` | `false` while `~/driver_io` paused driver data exchange |
| `joints_attached`, `base_attached` | Every joint feedback group / the chassis is attached; `base_attached` is `false` without a free root |
| `groups` | `[{name, fresh, attached}]` per engine feedback group, configured groups first, then `default` |
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
