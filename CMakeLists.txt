cmake_minimum_required(VERSION 3.8)
project(kstone_ros)

find_package(ament_cmake REQUIRED)
find_package(kstone CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(kstone_ros_controller STATIC src/controller.cpp src/json.cpp)
target_compile_features(kstone_ros_controller PUBLIC cxx_std_17)
target_include_directories(kstone_ros_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(kstone_ros_controller PUBLIC kstone::kstone Threads::Threads
  PRIVATE nlohmann_json::nlohmann_json)
target_compile_options(kstone_ros_controller PRIVATE -Wall -Wextra -Wpedantic)
add_executable(kstone_node src/node.cpp)
target_link_libraries(kstone_node PRIVATE kstone_ros_controller)
ament_target_dependencies(kstone_node PUBLIC rclcpp rcl_interfaces geometry_msgs sensor_msgs nav_msgs std_msgs std_srvs tf2_ros)
target_compile_options(kstone_node PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS kstone_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md README.zh-CN.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_test REQUIRED)
  add_executable(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller PRIVATE kstone_ros_controller)
  target_compile_definitions(test_controller PRIVATE KSTONE_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/omni.yaml")
  ament_add_test(controller COMMAND "$<TARGET_FILE:test_controller>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO TIMEOUT 90)
  set(blackbox_env "KSTONE_ROS_EXECUTABLE=$<TARGET_FILE:kstone_node>"
    "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1" "PYTHONDONTWRITEBYTECODE=1"
    "ROS_LOCALHOST_ONLY=1" "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
  ament_add_pytest_test(config test/test_config.py TIMEOUT 60
    ENV ${blackbox_env} "ROS_DOMAIN_ID=93")
  ament_add_pytest_test(ros test/test_ros.py TIMEOUT 120
    ENV ${blackbox_env} "ROS_DOMAIN_ID=91")
  ament_add_pytest_test(launch test/test_launch.py TIMEOUT 90
    ENV ${blackbox_env} "ROS_DOMAIN_ID=92")
endif()

ament_package()
