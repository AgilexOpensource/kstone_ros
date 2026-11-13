# kstone_ros 安装

[English](install.md)

[README](../README.zh-CN.md#安装) 中单工作空间安装之外的构建方式。

## 使用已安装的 kstone

若 kstone 已安装，跳过引擎的 CMake 命令。否则先初始化其子模块，并安装 README 中的系统依赖。此时 ROS 工作空间只包含 `kstone_ros`，不再放第二份 kstone 源码。将 `/path/to/...` 占位符替换为实际目录。

```bash
source /opt/ros/humble/setup.bash
export KSTONE_PREFIX=/path/to/kstone/install
cmake -S /path/to/kstone -B /path/to/kstone/build \
  -DCMAKE_INSTALL_PREFIX="$KSTONE_PREFIX" -DKSTONE_PINOCCHIO_SOURCE=SYSTEM \
  -DKSTONE_BUILD_TESTS=OFF -DKSTONE_BUILD_TOOLS=OFF -DKSTONE_BUILD_EXAMPLES=OFF
cmake --build /path/to/kstone/build --parallel 2
cmake --install /path/to/kstone/build
export CMAKE_PREFIX_PATH="$KSTONE_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
cd /path/to/ros_workspace
rosdep install --from-paths src/kstone_ros --ignore-src -r -y --skip-keys kstone
colcon build --packages-select kstone_ros
source install/setup.bash
```

- kstone 默认以 Release 构建。`KSTONE_PINOCCHIO_SOURCE=SYSTEM` 使用 ROS 的 Pinocchio 和 Coal，而不是 pip 包。
- `find_package(kstone CONFIG REQUIRED)` 需要已安装的头文件、库和 CMake 文件，仅有构建目录不够。
- 若 kstone 是共享库，还需将其库目录加入运行时加载路径。
- 只用一套依赖。混用 pip 或 Conda 的 Pinocchio/Coal 与 ROS 包会导致崩溃；不需要引擎的 Python 绑定。
- 其他依赖配置见 [kstone 安装指南](https://github.com/AgilexOpensource/kstone/blob/HEAD/docs/zh-CN/install.md)。

## 运行测试

开启测试重新构建，并在工作空间中运行本包测试：

```bash
cd ~/kstone_ws
colcon build --packages-select kstone_ros --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select kstone_ros --event-handlers console_direct+
colcon test-result --test-result-base build/kstone_ros --verbose
```

测试使用 ROS Python 客户端 `rclpy`，不使用引擎的 Python 绑定。
