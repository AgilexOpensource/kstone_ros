# kstone_ros 接口

[English](interfaces.md)

`kstone_node` 的话题、服务、state 字段和参数参考。返回 [README](../README.zh-CN.md)。

名称为节点私有：节点位于 `/left/kstone` 时，`~/state` 解析为 `/left/kstone/state`。支持重映射，`~/discover` 报告解析后的名称、类型、坐标系和 QoS。`M` 是 YAML 中的模块 ID。只为 `frame_target`、`chassis` 和 `joint_target` 模块创建话题。

## 驱动话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `~/joint_states` | 订阅 `sensor_msgs/JointState` | 驱动测得的关节位置 |
| `~/odom` | 订阅 `nav_msgs/Odometry` | 测得的 world 到根位姿，或控制中心位姿（见 `center_frame`）；仅自由根 |
| `~/feedback/joint_states` | 发布 `sensor_msgs/JointState` | 按 URDF 树顺序的关节位置命令，不是测量反馈 |
| `~/feedback/cmd_vel` | 发布 `geometry_msgs/Twist` | 底盘机体坐标系中的底盘命令 |

驱动输出依赖真实反馈，而不是陈旧模型初值。越界标量同步使用真实 Ruckig，不再瞬时裁限。

- `~/feedback/joint_states` 只包含反馈组已接入的关节：某组的未锁定关节都有新鲜 `~/joint_states` 之前不发布；某组过期（例如手单独放在 `feedback_groups` 里，手掉线）时只去掉该组关节，其余组照常发布。新鲜的 `~/odom` 到达之前，`~/feedback/cmd_vel` 不发布。
- 接入需要持续 `feedback_timeout_s` 的新鲜反馈。等待窗口持续同步最新原始测量，未附着时被禁止下发的内部写入不推进修正。同步组首条已附着命令位置等于原始反馈（mimic 按模型派生），速度为零；只有之后的已附着写入才推进连续 `JointTrajectory`，使用守卫缩放速度限位及默认加速度/jerk 因子 2/10，不因每个新鲜样本重启。独立引擎反馈组保留各自接入生命周期；附着后外部转发器继续禁止下发仍属于下述独立转发边界。
- 合法有限的受支持反馈原样接收和展示，包括越界位置。名称为空或全部不支持、长度不匹配、重复名称、NaN/Inf、非法时间年龄及乱序样本仍原子拒绝。驱动或锁定修正关节的模块保持启用和约束，但挂起任务及 prepare，目标/FK/轨迹从当前命令重新基准。完成后受影响模块从合法姿态重新基准；无关模块及反馈组正常继续。限位内同步及命令过期规则不变；越界修正丢弃受影响目标。过期重附着及通用暂停恢复从最新原始反馈重启。求解、守卫或写入失败不推进轨迹，也不放宽模型限位。
- 修正后 IK、gaze、CoM/碰撞恢复及 null-space 无需新 target/home 即恢复，仍服从既有仲裁。不添加运动意图门控或新服务；启动轨迹不保证姿态可行，也不覆盖普通模块接管的加速度/jerk。参见 kstone 的 `docs/zh-CN/robot.md`。
- **外部转发边界：** 转发器关闭时，机器人可能未运动而核心已完成修正。实际打开或重新打开时必须用新鲜反馈重同步，并协调首条成功的新命令。已有引擎 `set_paused(true)` 后 `set_paused(false)` 提供该生命周期；启用调试协议时用 `pause`，先 `paused: true` 再 `false`。不新增 ROS 服务。冷启动完成或附着不等于转发安全；guardian 门控及临时位置差检查保持不变，重新打开仍未验证。
- 覆盖性回归：`test_controller` 使用几何测试模型 `test/fixtures/coverage.yaml` / `coverage.urdf`，覆盖全部内置模块类型（chassis、frame_target、joint_target、null_space、gaze、center_of_mass、self_collision、world_collision）以及耦合 mimic、锁定轮关节和按模块划分的反馈组，不依赖任何机器人描述包。在关节超出限位时，验证启动/重附着首条原始引擎命令、原始实测 FK、逐周期位置变化及恢复速度/加速度/jerk 和实际完成，且各自动模块保持启用。核心测试另外对比真实 Ruckig 输出，验证目标/FK 一致性、mimic、频率变化及失败回滚。机器人包自行检查各自的发布配置（Lookie 由 lookie_bringup 核对驱动桥与 URDF）。
- 关节和底盘各自独立接入。自由根是单独的引擎反馈组 `base`，除非 YAML 的 `feedback_groups` 另行安排。
- 反馈过期达 `feedback_timeout_s` 时，对应输出断开：关节命令停止；`cmd_vel` 发送零速度持续一个 `command_timeout_s` 后停止。新鲜反馈到达后重新接入。
- 没有反馈时引擎照常运行，没有空跑模式。无硬件测试时，用仿真器发布 `~/joint_states` 和 `~/odom`。

## 引擎话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `~/state` | 发布 `std_msgs/String` | JSON 快照、故障、已接受目标和请求结果；锁存 |
| `~/discover` | 发布 `std_msgs/String` | JSON 模块与接口列表；锁存 |
| `~/control/joint` | 订阅 `sensor_msgs/JointState` | 所有 `joint_target` 模块的位置目标，按关节名分发 |

`~/control/joint` 规则：

- 使用 `name` 和 `position`，顺序任意。`header.frame_id` 须为空；velocity 和 effort 被忽略。
- 可只发部分关节；未给出的关节保持所属模块的当前目标。
- 同一控制周期内到达的多条消息按顺序全部生效，后到的消息只覆盖它给出的关节。每条消息整体接受或整体拒绝。
- 不属于任何 `joint_target` 模块的名称被忽略，并输出限频警告。数值按模块限位截断。
- 名称重复、name 与 position 数量不一致、非有限值或归属不明确时，整条消息被拒绝。

## 模块话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `~/M/control/pose` | 订阅 `geometry_msgs/PoseStamped` | `frame_target` 目标，或全向底盘平面位姿目标 |
| `~/M/control/cmd_vel` | 订阅 `geometry_msgs/Twist` | 底盘机体坐标系速度（全向或差速） |
| `~/M/feedback/pose` | 发布 `geometry_msgs/PoseStamped` | 当前 frame 位姿或底盘控制中心估计，不是目标 |

- frame 目标和反馈使用模块配置的参考坐标系。底盘位姿目标和反馈使用 `world_frame`。
- 底盘位姿目标的 z 须为零，旋转只能绕 yaw。
- 位姿输入须使用 `~/discover` 列出的坐标系；`frame_id` 为空表示该坐标系。不做 TF 转换。
- 四元数须有限且非零，会被归一化。
- Twist 只使用 `linear.x`、`linear.y` 和 `angular.z`；差速底盘还忽略 `linear.y`。超出底盘限值的速度会被限制，并输出限频警告。
- 每个模块以最新接受的目标为准，保持到被替换，与 `kstone run` 相同。只有端口超时（如底盘速度）会失效。

## 服务

| 服务 | 类型 | 说明 |
|---|---|---|
| `~/home` | `std_srvs/Trigger` | 执行 kstone `home_all`，并将底盘速度置零 |
| `~/home_cancel` | `std_srvs/Trigger` | 在同一周期取消所有正在进行的回零，各模块在当前位置平滑停下，保持使能并停在原地；未在回零的模块不受影响 |
| `~/M/home` | `std_srvs/Trigger` | 让一个可回零模块回零 |
| `~/M/enable` | `std_srvs/SetBool` | 启用或禁用一个模块 |
| `~/driver_io` | `std_srvs/SetBool` | 暂停或恢复 ROS 层与驱动的数据收发；引擎继续运行 |

```bash
ros2 service call /kstone/tool_pose/home std_srvs/srv/Trigger "{}"
ros2 service call /kstone/tool_pose/enable std_srvs/srv/SetBool "{data: false}"
```

- `~/home` 让处于控制中的模块回零，即最近收到目标的模块。无人控制或由不可回零模块控制的部分保持不动；`~/M/home` 仍可让它们回零。
- 回零移动到 YAML 中的回零位置。已禁用的模块可能拒绝回零，`~/home` 不是原子操作。
- 响应 `accepted request_id=N` 表示已排队，不表示已完成。按下文在 `~/state` 中跟踪结果。
- 回零前停止目标发布；`outcomes` 只保留 32 条，持续的目标流会很快把服务结果挤出。
- `~/driver_io` 设为 false 后立即丢弃驱动反馈并停发关节命令；`cmd_vel` 先发一个 `command_timeout_s` 的零速，然后停发。反馈随之过期，引擎在 `feedback_timeout_s` 后断开接入。
- `~/driver_io` 设为 true 时，只要还有反馈组处于接入状态就会失败。接受后，要等新鲜反馈重新接入才恢复输出，首条命令等于该反馈。
- 驱动门控或电机使能变化时：先暂停，等 `joints_attached` 和 `base_attached` 都为 `false`，再操作驱动，确认电机静止后恢复。节点不读取驱动状态，何时调用此服务由单独的监控节点决定。

## State 字段

`~/state` 是 `schema_version` 为 `1` 的 JSON 对象，按 `state_rate_hz` 发布。

| 字段 | 说明 |
|---|---|
| `fresh` | 上一周期所有反馈组都在 `feedback_timeout_s` 之内 |
| `driver_io` | `~/driver_io` 暂停驱动数据收发时为 `false` |
| `joints_attached`、`base_attached` | 所有关节反馈组 / 底盘已接入；没有自由根时 `base_attached` 为 `false` |
| `groups` | 各引擎反馈组的 `[{name, fresh, attached}]`，先是配置的组，最后是 `default` |
| `written` | 本周期写出了安全命令，且快照未过期 |
| `quality` | `nominal`；某阶段保持、不精确或步长被截断时为 `degraded` |
| `fault` | 正常时为空，否则为原因，如 `driver_stale_feedback`、`held_or_stale`、`stopped`、`duration_complete` |
| `expired_modules` | 因故障或反馈丢失而禁用的运动模块；新目标会重新激活它们 |
| `modules.M` | `enabled`、`running`、`held_by`、`home_state`、`ports`、`outputs` |
| `outcomes` | 最近 32 条请求结果 `{request_id, applied, error}` |
| `cycles`、`failed_cycles` | 引擎周期数和失败周期数 |
| `engine_time` | 引擎逻辑时间（秒） |
| `snapshot_stamp` | 快照时间（秒），节点时钟 |
| `generation` | 故障、恢复或重新接入时递增；旧 generation 的请求被丢弃 |
| `lifecycle` | `0` 未启动，`1` 运行中，`2` 引擎线程失败，`3` 已停止 |
| `rejected_requests` | 因引擎未运行或队列已满而被拒绝的请求 |
| `submitted_inputs` | 提交给引擎的话题输入 |
| `dropped_logs`、`truncated_logs` | 丢弃和截断的日志条目 |

在 `modules.M` 中，`running` 表示模块参与了本周期求解，`held_by` 是抢占它的模块 ID 或 `null`，不可回零的模块 `home_state` 为 `null`。

`home_state` 取值为 `idle`、`waiting`（已排队）、`executing`、`succeeded`、`timed_out`（`timeout_s` 内无进展）、`aborted`（被禁用、跳过周期或重新同步）和 `cancelled`（`~/home_cancel`）。终止状态保持到下一次回零请求。

确认回零完成：

1. 服务返回 `accepted request_id=N`。
2. 在 `outcomes` 中找到 `request_id` 为 `N` 的条目。若 `applied` 为 `false`，`error` 给出原因。
3. 跟踪 `modules.M.home_state`：`waiting`、`executing`，然后 `succeeded`。`timed_out` 或 `aborted` 表示失败。

调用 `~/home` 后，只关注经历过 `waiting` 或 `executing` 的模块；其他模块保持之前的 `home_state`。

## 节点参数

所有参数在启动时读取。默认值见 [bridge.yaml](../config/bridge.yaml)；launch 参数见 [README](../README.zh-CN.md#接入自己的模型)。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `config_file` | 空（launch：`config/omni.yaml`） | kstone 引擎 YAML |
| `config_variables` | `{}` | YAML 中 `${name}` 的字符串值 JSON 对象 |
| `duration` | `0.0` | 引擎逻辑时间上限（秒）；`0` 运行到关闭 |
| `realtime` | 未设置 | 可选 bool；`true` 应用 YAML 的 `thread` 块，`false` 忽略它，未设置时沿用 YAML |
| `topic_prefix` | `~` | 所有话题和服务名的前缀 |
| `world_frame` | `world` | 世界坐标系 |
| `base_frame` | `base_link` | 模型根坐标系 |
| `frame_prefix` | 空 | 加在每个坐标系 ID 前，与 `robot_state_publisher` 相同 |
| `center_frame` | 空 | 表示底盘控制中心的 `~/odom` child frame |
| `chassis_module` | 空 | 底盘模块 ID；为空时自动选择 |
| `command_timeout_s` | `0.5` | 引擎应用命令时允许的最大命令时长 |
| `feedback_timeout_s` | `0.25` | 保持输出接入的最大驱动反馈时长 |
| `state_rate_hz` | `30.0` | `~/state` 发布频率 |
| `joint_command_topic` | `~/feedback/joint_states` | 关节命令的额外话题 |
| `base_command_topic` | `~/feedback/cmd_vel` | 底盘命令的额外话题 |
| `feedback_joint_topic` | `~/joint_states` | 额外的关节反馈话题 |
| `feedback_base_topic` | 空 | 额外的根位姿反馈话题，`geometry_msgs/PoseStamped` |
| `publish_tf` | `false` | 发布 world 到根的 TF |
| `debug_port` | `0` | 供 `kstone-viewer` 连接的 kstone DebugServer 端口；`0` 关闭 |
| `debug_host` | `127.0.0.1` | DebugServer 监听地址 |
| `debug_token` | 空 | DebugServer 令牌；为空时使用 `$KSTONE_TOKEN` |

- 反馈时间戳早于 `feedback_timeout_s` 或来自未来时被拒绝。
- 命令时间戳（`~/control/joint`、`M/control/pose`）按话题检查连续性，不与节点时钟比较，因此可回放录制的 bag，也接受其他时钟的发布者。第一条消息、或静默超过 `command_timeout_s` 后的第一条消息记为参考。之后的消息若时间戳回退，或时间戳的推进与距参考的实际经过时间相差超过 `command_timeout_s`（传输延迟、积压、时钟跳变），则被拒绝，同时丢弃参考，下一条消息重新记为参考。每个命令话题只应有一个发布者。
- 时间戳为零时使用接收时间。
- YAML 中更小的正端口超时会缩短 `command_timeout_s`。
- 设置 `center_frame` 后，`child_frame_id` 为该值的 `~/odom` 消息按底盘 YAML 的 `center_in_root` 转换；child 为 `base_frame` 的消息仍作为根位姿。
- 底盘机体坐标系是底盘模块 YAML 中的 `base_frame`。
- 驱动话题参数会增加第二个话题；`~/joint_states`、`~/feedback/joint_states` 和 `~/feedback/cmd_vel` 始终存在。
- `publish_tf` 需要自由根，且 `world_frame` 与 `base_frame` 不同。
- 开启实时且 YAML 的 `thread` 块需要更高的 rtprio 或 memlock 限制时，引擎对自身执行 `sudo prlimit`，并在终端请求密码。没有终端时输出警告并继续。
- 非 loopback 的 `debug_host` 需要令牌。launch 参数 `debug_port:=7311` 会覆盖参数文件。
- `~/M/feedback/pose`、驱动命令和 TF 按 YAML 中的引擎频率（`rate_hz`）发布。

## QoS

| 接口 | QoS |
|---|---|
| `~/state`、`~/discover` | Reliable，transient local，depth 1 |
| 目标、位姿反馈、驱动命令 | Reliable，volatile，depth 10 |
| `~/joint_states`、`~/odom`、额外反馈话题 | Sensor data（best effort，volatile，depth 5） |
| 服务 | ROS 默认值 |

## 故障

- 故障或反馈丢失会禁用运动模块；新鲜目标会重新启用它们。通过 `~/M/enable` 禁用的模块需要显式启用。
- 缺少反馈时输出被门控。新鲜完整的反馈恢复求解，不会重放已清除的目标。
- 锁存的引擎或调度器故障无法通过 enable 或 home 清除。排除原因后重启节点，再发送新鲜反馈和命令。
- `duration` 结束后继续发布 state，但停止命令；再次运行需重启节点。
