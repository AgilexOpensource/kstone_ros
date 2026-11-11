# kstone_ros

[English](README.md)

通过 ROS 2 使用 [kstone](https://github.com/AgilexOpensource/kstone) 引擎控制末端位姿、关节目标和底盘速度。

## 安装

使用 **Ubuntu 22.04、ROS 2 Humble、C++17、CMake ≥ 3.22** 和 kstone 0.0.1。
安装 ROS、配置 rosdep 后，在新终端执行：

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install ros-humble-pinocchio ros-humble-coal nlohmann-json3-dev
mkdir -p ~/kstone_ws/src
cd ~/kstone_ws
```

将本包放在 `~/kstone_ws/src/kstone_ros`，然后构建：

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

已有引擎源码时，用 `git -C src/kstone submodule update --init --recursive` 初始化子模块，无需重新克隆。
使用一致的 ROS/系统依赖，不要混入 pip/Conda 二进制库；不需要引擎 Python 绑定。
已有引擎安装时，见[外部引擎安装](#外部引擎安装)。

## 快速上手 — 无需硬件

随包提供的 [omni YAML](config/omni.yaml) 和 [URDF](config/omni.urdf) 以 loopback 运行全向底盘、三个工具平移轴及夹爪。
在一个终端启动节点：

```bash
source /opt/ros/humble/setup.bash
source ~/kstone_ws/install/setup.bash
ros2 launch kstone_ros kstone.launch.py
```

**每个新终端**先执行 `source ~/kstone_ws/install/setup.bash`。
以下每条命令分别运行；echo 和 `-r 10` 发布器会持续运行，按 Ctrl+C 停止。

查看状态与可用接口：

```bash
ros2 topic echo /kstone/state --qos-durability transient_local
ros2 topic echo /kstone/discover --qos-durability transient_local
```

在 `base_link` 坐标系中移动工具：

```bash
ros2 topic pub -r 10 /kstone/tool_pose/control/pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.1, y: 0.0, z: 0.4}, orientation: {w: 1.0}}}"
```

设置夹爪位置：

```bash
ros2 topic pub -r 10 /kstone/control/joint \
  sensor_msgs/msg/JointState "{name: [finger_joint], position: [0.03]}"
```

在底盘机体坐标系中控制速度：

```bash
ros2 topic pub -r 10 /kstone/chassis/control/cmd_vel \
  geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.2}}"
```

## 接口

默认使用节点私有名称：节点 `/left/kstone` 的 `~/state` 解析为 `/left/kstone/state`。
支持重映射；`~/discover` 报告实际名称、类型、坐标系及 QoS。
`M` 是 YAML 模块 ID。

### 驱动话题

| 端点 | 方向 / 类型 | 用途 |
|---|---|---|
| `~/joint_states` | 输入 INPUT `JointState` | 驱动的真机关节反馈；所有未锁定关节须持续新鲜才会开启关节输出 |
| `~/odom` | 输入 INPUT `Odometry` | 驱动的真机世界/模型根里程计，仅自由根时订阅；须持续新鲜才会开启 `cmd_vel` 输出 |
| `~/feedback/joint_states` | 输出 OUTPUT `JointState` | 按模型（URDF 树）顺序的关节位置命令，**不是实测反馈**；关节反馈同步前及不安全时不发布 |
| `~/feedback/cmd_vel` | 输出 OUTPUT `Twist` | 底盘机体坐标系命令；里程计同步前不发布；不安全、停止或完成时为零 |

驱动输出以真机反馈为前提，避免真机接入时跳向过期的模型目标：

- `~/joint_states` 覆盖所有未锁定关节且新鲜之前，`~/feedback/joint_states` 不发布；`~/odom` 新鲜之前（仅自由根），`~/feedback/cmd_vel` 不发布。
- 接入需要反馈连续新鲜 `feedback_timeout_s`。窗口内每个周期都用实测状态覆盖模型命令（第一个周期还把模块目标重置为实测值），因此发布的第一条命令从当前实测状态出发；已使能的模块保持使能。
- 关节和底盘分别门控：除非 YAML 的 `feedback_groups` 另行指定，自由根单独作为引擎反馈组 `base`。反馈过期（`feedback_timeout_s`）后对应输出断开：关节命令停发；`cmd_vel` 先发一个 `command_timeout_s` 的零速，再停发。反馈恢复新鲜后重新接入。
- 没有反馈时引擎照常运行；不提供 dry-run 模式。无真机测试时，可由仿真器（如 MuJoCo 仿真包）发布 `~/joint_states`/`~/odom`，或使用模型功能包的 RViz launch。

### 引擎话题

| 端点 | 方向 / 类型 | 用途 |
|---|---|---|
| `~/state` | 输出 OUTPUT `String`（JSON） | 当前快照、故障、已接受目标及异步结果；各模块状态、已接受目标及回零状态在 `modules.M` 下；锁存 |
| `~/discover` | 输出 OUTPUT `String`（JSON） | 可用模块与接口；锁存 |
| `~/control/joint` | 输入 INPUT `JointState` | 所有 `joint_target` 模块的位置目标，按关节名路由 |

### 模块话题

| 端点 | 方向 / 类型 | 用途 |
|---|---|---|
| `~/M/feedback/pose` | 输出 OUTPUT `PoseStamped` | frame task 当前 FK 或底盘控制中心估计，**不是目标** |
| `~/M/control/pose` | 输入 INPUT `PoseStamped` | `frame_target` 目标；**仅全向底盘**提供平面位姿目标 |
| `~/M/control/cmd_vel` | 输入 INPUT `Twist` | 全向/差速底盘的机体坐标系速度；超速由底盘限幅，并节流打印警告 |

关节目标使用任意顺序的 `name`/`position`，frame ID 须为空，忽略 velocity/effort。
可只发任意子集关节，未提及的关节保持所属模块的当前目标。不属于任何 joint_target 关节链的名称会被忽略并节流打印警告，数值按模块限位裁剪。名称重复、名称与位置数量不符、非有限值或归属有歧义时整条原子拒绝，同一模块以最后接受的目标为准。
位姿反馈来自当前实测/估计快照；frame_target 使用配置的参考坐标系，底盘位姿目标/反馈使用 `world_frame`。
Twist 只使用平面 x/y/yaw；其余轴忽略，差速底盘也忽略 `linear.y`。

### 服务

| 端点 | 方向 / 类型 | 用途 |
|---|---|---|
| `~/home` | 请求 / 响应 `Trigger` | 执行 kstone `home_all`（持有控制权、即最后收到目标的模块回零；无人控制或被注视等不可回零模块控制的部位保持不动，`~/<id>/home` 仍可单独回零），并清零底盘速度 |
| `~/M/enable` | 请求 / 响应 `SetBool` | 启用或禁用一个模块 |
| `~/M/home` | 请求 / 响应 `Trigger` | 一个可回零模块回零 |

回零前先停止目标发布器。home 使用 YAML 配置的回零位置，不重新定义它们。
禁用模块可能拒绝回零，全局回零不保证原子成功。
成功（`accepted request_id=N`）仅表示入队，**不是完成**；查看 `~/state` 异步结果及其中 `modules.M` 的回零状态。

```bash
ros2 service call /kstone/tool_pose/home std_srvs/srv/Trigger "{}"
ros2 service call /kstone/tool_pose/enable std_srvs/srv/SetBool "{data: false}"
```

### state 字段

`~/state` 是 JSON 对象（`schema_version` 为 `1`），按 `state_rate_hz` 发布：

| 字段 | 含义 |
|---|---|
| `fresh` | 上一周期所有反馈组都在 `feedback_timeout_s` 内 |
| `joints_attached`、`base_attached` | 关节 / 底盘驱动输出已接入（见驱动话题）；无自由根时 `base_attached` 为 `false` |
| `written` | 本周期已写出安全命令，且快照未过期；为 `false` 时驱动输出不可信 |
| `quality` | 求解质量：`nominal`，或 `degraded`（某阶段保持、不精确或步长被裁剪） |
| `fault` | 空串表示无故障；否则为原因，如 `driver_stale_feedback`、`held_or_stale`、`cycle_error: ...`、`stopped`、`duration_complete` |
| `expired_modules` | 因故障或反馈丢失被禁用、须新目标才能重新激活的运动模块 ID |
| `modules.M` | `enabled`、`running`（本周期参与求解）、`held_by`（抢占该模块的模块 ID，否则 `null`）、`home_state`（不可回零为 `null`）、`ports`（当前已接受的输入）、`outputs`（模块输出） |
| `outcomes` | 最近 32 条请求结果 `{request_id, applied, error}`，含服务和目标话题；`applied` 为 `false` 时 `error` 给出原因 |
| `cycles`、`failed_cycles`、`engine_time` | 引擎周期数、失败周期数、逻辑时间（秒） |
| `snapshot_stamp` | 快照时刻（秒，节点时钟） |
| `generation` | 故障、恢复或重新接入时递增；之前代次的请求被丢弃 |
| `lifecycle` | `0` 未启动、`1` 运行、`2` 线程异常退出、`3` 已停止 |
| `rejected_requests`、`submitted_inputs` | 被拒绝的请求数（未运行或队列满）、已提交的话题输入数 |
| `dropped_logs`、`truncated_logs` | 丢弃 / 截断的日志条数 |

`home_state` 取值：`idle`、`waiting`（已入队）、`executing`、`succeeded`、`timed_out`（`timeout_s` 内无进展）、`aborted`（被禁用、周期跳过或重新同步）。终态保持到下一次回零请求。

判断回零是否完成：

1. 服务返回 `accepted request_id=N`。
2. 在 `outcomes` 中找到 `request_id == N`：`applied` 为 `false` 表示请求被拒绝，看 `error`。
3. 被接受后跟踪 `modules.M.home_state`：`waiting` → `executing` → `succeeded` 为成功，`timed_out` / `aborted` 为失败。

`~/home` 只让持有控制权的模块回零，其他模块的 `home_state` 保留上一次的终态，只看本次进入过 `waiting` / `executing` 的模块。
`outcomes` 只保留 32 条，目标话题持续发布会很快挤掉服务结果，回零前须停止目标发布器。

## 接入自己的模型

从随包 YAML/URDF 开始，替换模型并配置模块、限位、频率及回零位置。
推荐使用 `left_tool`、`hand`、`mobile_platform` 等清晰 ID；支持的模块 ID 须匹配 `[A-Za-z_][A-Za-z0-9_]*`。
唯一底盘会自动选中，不要求固定模块 ID。

```bash
ros2 launch kstone_ros kstone.launch.py \
  config_file:=/absolute/path/control.yaml \
  config_variables:='{"urdf": "/absolute/path/model.urdf"}' \
  realtime:=true duration:=0
```

将路径替换为自己的文件。`config_variables` 是可选的字符串值 JSON；示例键仅适用于 YAML 使用 `${urdf}` 的情况。
URDF 路径可相对 YAML。驱动发布新鲜的 `~/joint_states`（自由根还需 `~/odom`）之前，驱动输出保持静默。
多实例使用 launch `namespace` 和私有/相对话题；绝对覆盖名称除非重映射，否则会共享。

## 启动设置

全部桥接参数仅在启动设置。默认值见 [bridge.yaml](config/bridge.yaml)，launch 参数见 [kstone.launch.py](launch/kstone.launch.py)。
未提供 launch 参数的桥接设置，通过 `params_file:=/absolute/path/bridge.yaml` 传入。

| 设置 | 默认值 / 含义 |
|---|---|
| `config_file`, `config_variables` | 随包 omni YAML；`{}` 替换变量 |
| `realtime` | launch 的 `inherit` 使用 YAML；`true` 应用 YAML 的 `thread` 块（实时调度），`false` 忽略它。运行总是按时钟节拍。直接运行节点时是可选布尔参数，省略即继承。realtime 生效且 YAML `thread` 块需要更高的 rtprio/memlock 限额时，启动时对节点执行 `sudo prlimit`（在终端提示输密码；无终端则 warn 后照常运行） |
| `duration` | `0` 运行到关闭；正的有限秒数限制引擎**逻辑时间**，含暂停/失败周期。完成后保留遥测、停止命令，再次运行须重启 |
| `topic_prefix`, `namespace` | `~`；launch 命名空间为空 |
| `world_frame`, `base_frame` | `world`、`base_link`：世界和模型根坐标系；底盘机体坐标系来自 YAML 的 `base_frame` |
| `publish_tf` | `false`；可选发布世界 → 模型根 TF，要求自由根且两个坐标系不同 |
| `chassis_module` | 空：自动选择；指定时须为拥有模型根的底盘 |
| `command_timeout_s` | `0.5`；引擎应用命令时，若命令已超过该时间（或更短的正值端口超时）则拒绝；也拒绝过期/未来的非零命令时间戳 |
| `feedback_timeout_s` | `0.25`；保持驱动输出接入所需的驱动反馈新鲜度；也拒绝过期/未来的非零反馈时间戳，并限定底盘里程计时效。零时间戳按接收时间处理 |
| `state_rate_hz` | `30`；state 话题的频率。feedback pose、驱动命令（`feedback/joint_states`、`feedback/cmd_vel`）和 TF 跟随引擎频率 |
| `debug_port`、`debug_host`、`debug_token` | `0`（关闭）、`127.0.0.1`、空；启动 kstone DebugServer 供 `kstone-viewer` 连接。非回环地址必须设置 token；token 为空时读取 `$KSTONE_TOKEN`。launch 参数 `debug_port:=7311` 可覆盖参数文件。同一端口上 ROS 命令优先于 viewer 的运动编辑 |
| 驱动话题覆盖 | `feedback_joint_topic`、`feedback_base_topic`、`joint_command_topic`、`base_command_topic`，见默认配置；非空 `feedback_base_topic` 启用额外的模型根 `PoseStamped` 感知 |

位姿输入须使用发现信息中的坐标系（空值使用配置坐标系），**不隐式转换 TF**。
四元数须有限、非零，接收后会归一化。
QoS：state/discover 可靠、transient-local、深度 1；位姿反馈及命令可靠、volatile、深度 10；驱动感知 best-effort、volatile、深度 5。服务使用 ROS 默认值。

## 安全与故障

- 已接受的目标会一直保持到被替换，与 `kstone run` 一致；ROS 层不额外加租约。只有端口超时（如底盘速度）会失效。故障或反馈丢失会禁用运动模块；新目标可重新激活，手动禁用则须显式 enable。
- 感知缺失会门控输出；完整新鲜感知恢复求解，但不重放已清除目标。引擎/调度故障锁存后，先排除原因再重启，提供新鲜感知和新命令；enable/home 不能清除故障。
- 外部驱动须独立处理命令缺失看门狗、限位和急停。ROS 零命令**不是硬件故障保护**；示例不构成硬件安全验证。

## 故障排查

| 现象 | 检查 |
|---|---|
| 找不到包 | 在当前终端加载 ROS 和工作空间安装环境 |
| `find_package(kstone)` 失败 | 初始化引擎子模块，先构建引擎或使用下面的安装前缀 |
| 二进制库/ABI 错误 | 不要混用 pip/Conda 和 ROS 库。已测试组合：Pinocchio 4.0.0、Coal 3.0.3、Boost 1.74、`nlohmann-json3-dev`（不需要 Boost.JSON）；使用相同组合重建引擎与桥接包 |
| 命令被拒 / 机器人不动 | 检查发现信息中的类型/坐标系、完整关节名称集合及状态故障；驱动输出静默时查看 `~/state` 中的 `joints_attached`/`base_attached`，并发布新鲜的 `~/joint_states`/`~/odom` |

用 `ldd "$(ros2 pkg prefix kstone_ros)/lib/kstone_ros/kstone_node"` 检查运行时库。
XML 发现绕过措施不能修复 ABI 不匹配；其他依赖安装方式见[引擎安装指南](https://github.com/AgilexOpensource/kstone/blob/HEAD/docs/zh-CN/install.md)。

### 外部引擎安装

已经安装 kstone 时，跳过下面的引擎 CMake/build/install 命令。
否则先初始化引擎子模块，并使用[安装](#安装)中的系统依赖。
将 `/path/to/...` 替换为自己的目录；此 ROS 工作空间只放 `kstone_ros`，不要再放另一份引擎源码包。

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

`find_package(kstone CONFIG REQUIRED)` 需要已安装的头文件、库及 CMake 导出配置，仅有引擎构建目录不够。
使用共享引擎库时，也须让运行时加载器找到其库目录。

### 验证安装

在已加载环境的工作空间中启用测试、重新构建并运行本包测试：

```bash
cd ~/kstone_ws
colcon build --packages-select kstone_ros --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select kstone_ros --event-handlers console_direct+
colcon test-result --test-result-base build/kstone_ros --verbose
```

测试使用 ROS Python 客户端，不需要引擎 Python 绑定。
