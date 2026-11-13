# kstone_ros installation

[中文](install.zh-CN.md)

Build options beyond the one-workspace install in the [README](../README.md#install).

## Use an installed kstone

If kstone is already installed, skip the CMake commands for the engine. Otherwise initialize its submodules and install the system dependencies from the README. The ROS workspace then contains only `kstone_ros`, not a second kstone source tree. Replace the `/path/to/...` placeholders with your directories.

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

- kstone builds in Release by default. `KSTONE_PINOCCHIO_SOURCE=SYSTEM` uses the ROS Pinocchio and Coal instead of pip packages.
- `find_package(kstone CONFIG REQUIRED)` needs the installed headers, libraries and CMake files, not only a build folder.
- For a shared kstone library, add its library directory to the runtime loader path as well.
- Use one dependency stack. Mixing pip or Conda Pinocchio/Coal with the ROS packages causes crashes; the engine Python bindings are not needed.
- For other dependency setups, see the [kstone installation guide](https://github.com/AgilexOpensource/kstone/blob/HEAD/docs/install.md).

## Run the tests

Rebuild with testing on and run the package tests from the workspace:

```bash
cd ~/kstone_ws
colcon build --packages-select kstone_ros --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select kstone_ros --event-handlers console_direct+
colcon test-result --test-result-base build/kstone_ros --verbose
```

The tests use the ROS Python client `rclpy`, not the engine Python bindings.
