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

驱动输出依赖真实反馈，并使用其模型限位投影而不是陈旧的模型初值。第一条位置命令可能从越界实测位置跳到裁限后、经 IK 调整的状态。

- 新鲜的 `~/joint_states` 覆盖所有未锁定关节之前，`~/feedback/joint_states` 不发布。新鲜的 `~/odom` 到达之前，`~/feedback/cmd_vel` 不发布。
- 接入需要持续 `feedback_timeout_s` 的新鲜反馈。期间每个周期用裁限后的测量状态覆盖模型命令，第一个周期还把模块目标重置为裁限状态，因此第一条发布命令从裁限状态开始，且已允许正常 IK/自动纠偏。已启用的模块保持启用。
- 支持的关节反馈只要合法有限，即使越过软件位置限位也原样接收和展示。名称为空或全部不支持、长度不匹配、重复名称、NaN/Inf、非法时间年龄及乱序样本仍原子拒绝。启动/附着/重附着及暂停恢复同步时，先将 command/ideal 裁到模型限位，再初始化关节目标及位姿 FK/轨迹；不裁原始反馈。重附着丢弃失联目标；正常生命周期内的有效目标及既有命令过期规则保持不变。
- 采用旧 `ik_teleop_ros` 行为：同步后无需 fresh target/home，即正常运行 IK、自动 gaze、CoM/碰撞恢复及 null-space。不设运动意图门控、不添加自动恢复服务或新的全机输出抑制。第一条位置命令不保证等于原始反馈：裁限可以立即产生位置突变，随后仍有正常 IK/自动纠偏。原有轨迹限位、闭环碰撞/CoM 保护、终点检查和守卫失败处理仍适用。非独立驱动的 mimic 从关节遵循模型耦合；参见 kstone 的 `docs/zh-CN/robot.md`。
- 发布配置离线组合回归：运行 `test_controller` 时，把 `KSTONE_LOOKIE_CONFIG` 设为发布的 `kstone.yaml`，把 `KSTONE_LOOKIE_URDF` 设为碰撞 URDF。测试核对旁边 `driver_joint_bridge.yaml` 的全部 32 个独立命令关节，验证原始反馈不变、命令遵守限位、启动/重附着实测 FK 展示仍基于原始反馈，以及无 fresh target 的自动运动。核心引擎回归另行断言启动/重附着/恢复的关节目标及完整 FK/轨迹变换从裁限状态重置。离线执行只适配资源路径及实时权限；保留 gaze、CoM、自碰撞、null-space、三轮锁定及 mimic 耦合。 发布配置的 idle 关节占用可能保持静态启动姿态；组合测试在附着窗口加入小幅实测手臂变化，通过既有仲裁触发自动 gaze，全程不发送 target/home。
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
| `~/M/home` | `std_srvs/Trigger` | 让一个可回零模块回零 |
| `~/M/enable` | `std_srvs/SetBool` | 启用或禁用一个模块 |

```bash
ros2 service call /kstone/tool_pose/home std_srvs/srv/Trigger "{}"
ros2 service call /kstone/tool_pose/enable std_srvs/srv/SetBool "{data: false}"
```

- `~/home` 让处于控制中的模块回零，即最近收到目标的模块。无人控制或由不可回零模块控制的部分保持不动；`~/M/home` 仍可让它们回零。
- 回零移动到 YAML 中的回零位置。已禁用的模块可能拒绝回零，`~/home` 不是原子操作。
- 响应 `accepted request_id=N` 表示已排队，不表示已完成。按下文在 `~/state` 中跟踪结果。
- 回零前停止目标发布；`outcomes` 只保留 32 条，持续的目标流会很快把服务结果挤出。

## State 字段

`~/state` 是 `schema_version` 为 `1` 的 JSON 对象，按 `state_rate_hz` 发布。

| 字段 | 说明 |
|---|---|
| `fresh` | 上一周期所有反馈组都在 `feedback_timeout_s` 之内 |
| `joints_attached`、`base_attached` | 关节和底盘驱动输出已接入；没有自由根时 `base_attached` 为 `false` |
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

`home_state` 取值为 `idle`、`waiting`（已排队）、`executing`、`succeeded`、`timed_out`（`timeout_s` 内无进展）和 `aborted`（被禁用、跳过周期或重新同步）。终止状态保持到下一次回零请求。

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

- 过旧或来自未来的反馈与命令时间戳被拒绝。时间戳为零时使用接收时间。
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
