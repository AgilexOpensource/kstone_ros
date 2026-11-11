# kstone_ros

[中文](README.zh-CN.md)

Control end-effector poses, joint targets and chassis velocities through ROS 2 with the [kstone](https://github.com/AgilexOpensource/kstone) engine.

## Install

Use **Ubuntu 22.04, ROS 2 Humble, C++17 and CMake ≥ 3.22** with kstone 0.0.1.
In a fresh terminal with ROS installed and rosdep configured:

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install ros-humble-pinocchio ros-humble-coal nlohmann-json3-dev
mkdir -p ~/kstone_ws/src
cd ~/kstone_ws
```

Place this package at `~/kstone_ws/src/kstone_ros`, then build:

```bash
cd ~/kstone_ws
git clone --recurse-submodules --branch v0.0.1 \
  https://github.com/AgilexOpensource/kstone.git src/kstone
rosdep install --from-paths src/kstone src/kstone_ros --ignore-src -r -y
colcon build --packages-up-to kstone_ros --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DKSTONE_PINOCCHIO_SOURCE=SYSTEM \
  -DKSTONE_BUILD_PYTHON=OFF -DKSTONE_BUILD_TESTS=OFF \
  -DKSTONE_BUILD_TOOLS=OFF -DKSTONE_BUILD_EXAMPLES=OFF
source install/setup.bash
```

Already have the engine source? Initialize its submodules with
`git -C src/kstone submodule update --init --recursive` instead of cloning.
Use one ROS/system dependency stack, not pip/Conda binary libraries; engine Python bindings are not needed.
For an installed engine, see [External engine installation](#external-engine-installation).

## Quick start — no hardware needed

The shipped [omni YAML](config/omni.yaml) and [URDF](config/omni.urdf) run a loopback omni base, three tool axes and a gripper.
Start the node in one terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/kstone_ws/install/setup.bash
ros2 launch kstone_ros kstone.launch.py
```

In **each additional terminal**, source `~/kstone_ws/install/setup.bash` first.
Run each command below separately; echo and `-r 10` publishers keep running until Ctrl+C.

Inspect state and available interfaces:

```bash
ros2 topic echo /kstone/state --qos-durability transient_local
ros2 topic echo /kstone/discover --qos-durability transient_local
```

Move the tool in `base_link`:

```bash
ros2 topic pub -r 10 /kstone/tool_pose/control/pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.1, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}"
```

Set the gripper position:

```bash
ros2 topic pub -r 10 /kstone/control/joint \
  sensor_msgs/msg/JointState "{name: [finger_joint], position: [0.03]}"
```

Drive the chassis in its body frame:

```bash
ros2 topic pub -r 10 /kstone/chassis/control/cmd_vel \
  geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.2}}"
```

## Interfaces

Names are node-private by default: `/left/kstone` resolves `~/state` to `/left/kstone/state`.
Remapping is supported; `~/discover` reports resolved names, types, frames and QoS.
`M` is the YAML module ID.

### Driver topics

| Endpoint | Direction / type | Purpose |
|---|---|---|
| `~/joint_states` | INPUT `JointState` | Real joint feedback from the driver; every unlocked joint must stay fresh to enable joint output |
| `~/odom` | INPUT `Odometry` | Real world/root odometry from the driver, subscribed only for a free root; must stay fresh to enable `cmd_vel` output |
| `~/feedback/joint_states` | OUTPUT `JointState` | Joint position commands in model (URDF tree) order, **not measured feedback**; silent until joint feedback is synced, and when unsafe |
| `~/feedback/cmd_vel` | OUTPUT `Twist` | Chassis body-frame command; silent until odometry is synced; zero when unsafe, stopped or completed |

Driver outputs are gated on real feedback, so a robot never jumps to a stale model target on connect:

- Nothing is published on `~/feedback/joint_states` until fresh `~/joint_states` covers every unlocked joint; nothing on `~/feedback/cmd_vel` until fresh `~/odom` arrives (free root only).
- Attach takes `feedback_timeout_s` of continuously fresh feedback. Throughout that window the measured state overwrites the model command every cycle (the first cycle also rebases module targets to it), so the first published command starts from a current measurement; enabled modules stay enabled.
- Joint and base gating are independent: the free root is its own engine feedback group `base` unless the YAML `feedback_groups` places it. When feedback goes stale (`feedback_timeout_s`) that output detaches: joint commands stop; `cmd_vel` sends zero for one `command_timeout_s`, then stops. Attach repeats on fresh feedback.
- The engine keeps running without feedback; there is no dry-run mode. To test without hardware, feed `~/joint_states`/`~/odom` from a simulator (for example a MuJoCo sim package) or use the model package's RViz launch.

### Engine topics

| Endpoint | Direction / type | Purpose |
|---|---|---|
| `~/state` | OUTPUT `String` (JSON) | Current snapshot, faults, accepted targets and asynchronous outcomes; per-module status, accepted targets and home state under `modules.M`; latched |
| `~/discover` | OUTPUT `String` (JSON) | Available modules and interfaces; latched |
| `~/control/joint` | INPUT `JointState` | Position targets for every `joint_target` module, routed by joint name |

### Module topics

| Endpoint | Direction / type | Purpose |
|---|---|---|
| `~/M/feedback/pose` | OUTPUT `PoseStamped` | Current frame-task FK or chassis control-center estimate, **not the target** |
| `~/M/control/pose` | INPUT `PoseStamped` | `frame_target` target; **omni-only** chassis planar pose target |
| `~/M/control/cmd_vel` | INPUT `Twist` | Omni/differential chassis body-frame velocity; overspeed is limited by the chassis with a throttled warning |

Joint targets use `name`/`position` in any order; frame IDs must be empty and velocity/effort are ignored.
Any subset of joints is accepted, and omitted joints keep their module's current target. Names outside every joint_target chain are ignored with a throttled warning, and values are clamped to the module's limits. Duplicate names, a name/position count mismatch, non-finite values or ambiguous ownership reject the whole message atomically, and the latest accepted target wins per module.
Pose feedback comes from current measured/estimated snapshots. Frame targets use their configured reference; chassis pose targets/feedback use `world_frame`.
Twist uses only planar x/y/yaw; other axes, and `linear.y` on a differential chassis, are ignored.

### Services

| Endpoint | Direction / type | Purpose |
|---|---|---|
| `~/home` | REQUEST / RESPONSE `Trigger` | Run kstone `home_all` (home the modules in control, i.e. the ones last given a target; parts nobody controls, or controlled by a non-homeable module such as gaze, stay put; `~/<id>/home` still homes them) and zero chassis velocity |
| `~/M/enable` | REQUEST / RESPONSE `SetBool` | Enable or disable one module |
| `~/M/home` | REQUEST / RESPONSE `Trigger` | Home one homeable module |

Stop target publishers before homing. Home uses the YAML home positions; it does not redefine them.
Disabled modules may reject home, and global home is not atomic.
Success (`accepted request_id=N`) means queued, **not completed**: check `~/state` outcomes and the `modules.M` home status in `~/state`.

```bash
ros2 service call /kstone/tool_pose/home std_srvs/srv/Trigger "{}"
ros2 service call /kstone/tool_pose/enable std_srvs/srv/SetBool "{data: false}"
```

### State fields

`~/state` is a JSON object (`schema_version` `1`) published at `state_rate_hz`:

| Field | Meaning |
|---|---|
| `fresh` | Every feedback group was within `feedback_timeout_s` in the last cycle |
| `joints_attached`, `base_attached` | Joint / chassis driver output is attached (see driver topics); `base_attached` is `false` without a free root |
| `written` | A safe command was written this cycle and the snapshot is not stale; when `false`, driver output is not trustworthy |
| `quality` | Solve quality: `nominal`, or `degraded` (a stage held or inexact, or the step clipped) |
| `fault` | Empty when healthy; otherwise the reason, e.g. `driver_stale_feedback`, `held_or_stale`, `cycle_error: ...`, `stopped`, `duration_complete` |
| `expired_modules` | Motion module IDs disabled by a fault or feedback loss that need a new target to reactivate |
| `modules.M` | `enabled`, `running` (in this cycle's solve), `held_by` (ID of the preempting module, else `null`), `home_state` (`null` if not homeable), `ports` (current accepted inputs), `outputs` (module outputs) |
| `outcomes` | Last 32 request results `{request_id, applied, error}`, from services and target topics; when `applied` is `false`, `error` gives the reason |
| `cycles`, `failed_cycles`, `engine_time` | Engine cycle count, failed cycles, logical time (s) |
| `snapshot_stamp` | Snapshot time (s, node clock) |
| `generation` | Increments on fault, recovery or reattach; requests from an older generation are discarded |
| `lifecycle` | `0` not started, `1` running, `2` thread exited with an error, `3` stopped |
| `rejected_requests`, `submitted_inputs` | Rejected requests (not running or queue full), submitted topic inputs |
| `dropped_logs`, `truncated_logs` | Dropped / truncated log entries |

`home_state` values: `idle`, `waiting` (queued), `executing`, `succeeded`, `timed_out` (no progress within `timeout_s`), `aborted` (disabled, skipped cycle or resync). A terminal state persists until the next home request.

To check whether homing finished:

1. The service returns `accepted request_id=N`.
2. Find `request_id == N` in `outcomes`: `applied` `false` means the request was rejected; see `error`.
3. Once applied, follow `modules.M.home_state`: `waiting` → `executing` → `succeeded` is success; `timed_out` / `aborted` is failure.

`~/home` homes only modules in control; other modules keep their previous terminal `home_state`, so watch only modules that passed through `waiting` / `executing` this time.
`outcomes` keeps only 32 entries, and a continuous target stream quickly pushes out service results, so stop target publishers before homing.

## Use your own model

Start from the shipped YAML/URDF, replace the model and configure modules, limits, rate and home positions.
Use descriptive IDs such as `left_tool`, `hand` or `mobile_platform`; supported module IDs must match `[A-Za-z_][A-Za-z0-9_]*`.
The sole chassis is selected automatically; no fixed module ID is required.

```bash
ros2 launch kstone_ros kstone.launch.py \
  config_file:=/absolute/path/control.yaml \
  config_variables:='{"urdf": "/absolute/path/model.urdf"}' \
  realtime:=true duration:=0
```

Replace the paths with your files. `config_variables` is optional JSON of string values; the example key applies only if your YAML uses `${urdf}`.
URDF paths may be relative to the YAML. Driver outputs stay silent until the driver publishes fresh `~/joint_states` (and `~/odom` for a free root).
For multiple nodes, use launch `namespace` and private/relative topic names; absolute overrides are shared unless remapped.

## Startup settings

All bridge parameters are startup-only. Defaults are in [bridge.yaml](config/bridge.yaml); launch arguments are in [kstone.launch.py](launch/kstone.launch.py).
Pass `params_file:=/absolute/path/bridge.yaml` for bridge settings not exposed as launch arguments.

| Setting | Default / meaning |
|---|---|
| `config_file`, `config_variables` | Shipped omni YAML; `{}` substitution variables |
| `realtime` | Launch `inherit` uses the YAML; `true` applies the YAML `thread` block (real-time scheduling), `false` ignores it. Execution is always paced to the clock. Direct node: optional boolean, omit to inherit. When realtime is in effect and the YAML `thread` block needs higher rtprio/memlock limits, the node runs `sudo prlimit` on itself at startup (password prompt on the terminal; warns and continues without one) |
| `duration` | `0`: until shutdown; positive finite seconds limit engine **logical time**, including paused/failed cycles. Completion keeps telemetry, stops commands and requires restart for another run |
| `topic_prefix`, `namespace` | `~`; empty launch namespace |
| `world_frame`, `base_frame` | `world`, `base_link`: world and model-root frames; chassis body frame is its YAML `base_frame` |
| `publish_tf` | `false`; optionally publish world → model-root TF, requiring a free root and distinct frames |
| `chassis_module` | Empty: automatic selection; if set, must name the root-owning chassis |
| `command_timeout_s` | `0.5`; a command older than this (or than a shorter positive port timeout) when the engine applies it is rejected; also rejects stale/future nonzero command stamps |
| `feedback_timeout_s` | `0.25`; driver feedback freshness required to keep driver outputs attached; also rejects stale/future nonzero feedback stamps and bounds chassis odometry age. Zero stamps use reception time |
| `state_rate_hz` | `30`; rate of the state topic. Feedback poses, driver commands (`feedback/joint_states`, `feedback/cmd_vel`) and TF follow the engine rate |
| `debug_port`, `debug_host`, `debug_token` | `0` (off), `127.0.0.1`, empty; start a kstone DebugServer for `kstone-viewer`. A non-loopback host requires a token; an empty token falls back to `$KSTONE_TOKEN`. Launch `debug_port:=7311` overrides the params file. Viewer motion edits yield to ROS commands on the same port |
| Driver topic overrides | `feedback_joint_topic`, `feedback_base_topic`, `joint_command_topic`, `base_command_topic`; see bridge defaults. Nonempty `feedback_base_topic` enables additional root `PoseStamped` sensing |

Pose inputs must use the discovered frame (empty means configured frame); there is **no implicit TF conversion**.
Quaternions must be finite and nonzero; they are normalized.
QoS: state/discovery reliable transient-local depth 1; pose feedback and commands reliable volatile depth 10; driver sensing best-effort volatile depth 5. Services use ROS defaults.

## Safety and faults

- An accepted target holds until replaced, as with `kstone run`; ROS does not add a lease. Only port timeouts (for example chassis velocity) lapse. A fault or feedback loss disables motion modules; a fresh target rearms them, but manual disable requires explicit enable.
- Missing sensing gates output; fresh complete sensing restores solving without replaying cleared targets. For latched engine/scheduler faults, fix the cause and restart, then send fresh sensing and commands; enable/home cannot clear the fault.
- An external driver must independently watchdog missing commands and enforce limits/emergency stop. ROS zero commands are **not a hardware fail-safe**; this example is not hardware safety validation.

## Troubleshooting

| Symptom | Check |
|---|---|
| Package not found | Source ROS and the workspace installation in this terminal |
| `find_package(kstone)` fails | Initialize engine submodules; build the engine first or use an installed prefix below |
| Binary library/ABI error | Do not mix pip/Conda and ROS libraries. Tested stack: Pinocchio 4.0.0, Coal 3.0.3, Boost 1.74, `nlohmann-json3-dev` (no Boost.JSON needed); rebuild engine and bridge with the same stack |
| Commands rejected / robot stationary | Inspect discovery types/frames, complete joint name sets and state faults; for silent driver outputs check `joints_attached`/`base_attached` in `~/state` and publish fresh `~/joint_states`/`~/odom` |

Inspect runtime libraries with `ldd "$(ros2 pkg prefix kstone_ros)/lib/kstone_ros/kstone_node"`.
An XML discovery workaround does not fix ABI mismatches. See the [engine installation guide](https://github.com/AgilexOpensource/kstone/blob/HEAD/docs/install.md) for other dependency setups.

### External engine installation

If kstone is already installed, skip the engine CMake/build/install commands below.
Otherwise initialize its submodules and use the system dependencies from [Install](#install).
Replace `/path/to/...` with your directories; this ROS workspace should contain only `kstone_ros`, not another engine source package.

```bash
source /opt/ros/humble/setup.bash
export KSTONE_PREFIX=/path/to/kstone/install
cmake -S /path/to/kstone -B /path/to/kstone/build-release \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$KSTONE_PREFIX" \
  -DKSTONE_PINOCCHIO_SOURCE=SYSTEM \
  -DKSTONE_BUILD_PYTHON=OFF -DKSTONE_BUILD_TESTS=OFF \
  -DKSTONE_BUILD_TOOLS=OFF -DKSTONE_BUILD_EXAMPLES=OFF
cmake --build /path/to/kstone/build-release --parallel 2
cmake --install /path/to/kstone/build-release
export CMAKE_PREFIX_PATH="$KSTONE_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
cd /path/to/ros_workspace
rosdep install --from-paths src/kstone_ros --ignore-src -r -y --skip-keys kstone
colcon build --packages-select kstone_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`find_package(kstone CONFIG REQUIRED)` needs installed headers, libraries and CMake exports, not just an engine build folder.
For a shared engine, make its library directory available to the runtime loader too.

### Validate the installation

From a sourced workspace, rebuild with testing enabled and run the package suites:

```bash
cd ~/kstone_ws
colcon build --packages-select kstone_ros --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select kstone_ros --event-handlers console_direct+
colcon test-result --test-result-base build/kstone_ros --verbose
```

These tests use ROS Python clients, not engine Python bindings.
