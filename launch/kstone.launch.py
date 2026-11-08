"""Launch kstone_node with private driver inputs (joint_states, odom) and feedback/* outputs."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_node(context):
    realtime = LaunchConfiguration("realtime").perform(context).lower()
    if realtime not in ("inherit", "true", "false"):
        raise ValueError("realtime must be inherit, true or false")
    parameters = {
        "config_file": ParameterValue(LaunchConfiguration("config_file"), value_type=str),
        "config_variables": ParameterValue(LaunchConfiguration("config_variables"), value_type=str),
        "duration": ParameterValue(LaunchConfiguration("duration"), value_type=float),
        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
    }
    if realtime != "inherit":
        parameters["realtime"] = realtime == "true"
    debug_port = LaunchConfiguration("debug_port").perform(context)
    if debug_port != "inherit":
        parameters["debug_port"] = int(debug_port)
    return [Node(package="kstone_ros", executable="kstone_node", name="kstone",
                 namespace=LaunchConfiguration("namespace"), output="screen",
                 parameters=[LaunchConfiguration("params_file"), parameters])]


def generate_launch_description():
    share = FindPackageShare("kstone_ros")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=PathJoinSubstitution([share, "config", "omni.yaml"])),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([share, "config", "bridge.yaml"])),
        DeclareLaunchArgument("config_variables", default_value="{}"),
        DeclareLaunchArgument("duration", default_value="0.0"),
        DeclareLaunchArgument("realtime", default_value="inherit",
                              description="inherit/true/false; true applies thread.*, raising limits via sudo prlimit"),
        DeclareLaunchArgument("debug_port", default_value="inherit",
                              description="kstone DebugServer port for kstone-viewer; 0 = off"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=launch_node),
    ])
