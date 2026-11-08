"""Installed launch smoke test; execute from an unrelated working directory."""
import json
import os
import shutil
import signal
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


def test_installed_launch_round_trip(tmp_path):
    if not os.environ.get("AMENT_PREFIX_PATH") or not shutil.which("ros2"):
        pytest.fail("requires a built, sourced ROS workspace")
    from ament_index_python.packages import get_package_share_directory
    try:
        get_package_share_directory("kstone_ros")
    except LookupError:
        pytest.fail("requires an installed kstone_ros package")
    env = os.environ.copy()
    env["ROS_LOG_DIR"] = str(tmp_path / "ros-log")
    log_path = tmp_path / "launch.log"
    log = log_path.open("w")
    process = subprocess.Popen([shutil.which("ros2"), "launch", "kstone_ros", "kstone.launch.py", "realtime:=true"],
                               cwd=tmp_path, env=env, stdout=log,
                               stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    client = Node("installed_bridge_probe")
    discovery, poses, states = [], [], []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    client.create_subscription(String, "/kstone/discover", lambda msg: discovery.append(json.loads(msg.data)), latched)
    client.create_subscription(PoseStamped, "/kstone/tool_pose/feedback/pose", poses.append, 10)
    client.create_subscription(String, "/kstone/state", lambda msg: states.append(json.loads(msg.data)["modules"]["gripper_joint"]), latched)
    publisher = client.create_publisher(JointState, "/kstone/control/joint", 10)

    def wait_for(condition, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                pytest.fail(f"launch exited early: {log_path.read_text(errors='replace')}")
            rclpy.spin_once(client, timeout_sec=0.02)
            if condition():
                return
        pytest.fail("installed launch/DDS condition timed out")

    output = ""
    try:
        wait_for(lambda: discovery and poses and publisher.get_subscription_count())
        assert "mode" not in discovery[0]
        topics = {i["topic"]: i for i in discovery[0]["interfaces"] if "topic" in i}
        assert topics["/kstone/joint_states"]["direction"] == "subscribe"
        output = topics["/kstone/feedback/cmd_vel"]
        assert output["type"] == "geometry_msgs/msg/Twist" and output["frame"] == "base_link"
        assert topics["/kstone/feedback/joint_states"]["direction"] == "publish"
        assert poses[-1].header.frame_id == "base_link"
        publisher.publish(JointState(name=["finger_joint"], position=[0.025]))
        wait_for(lambda: states and states[-1]["ports"]["target"] == pytest.approx([0.025]))
        assert len(discovery[0]["modules"]) == 3
        assert "/kstone/discover" not in dict(client.get_service_names_and_types())
        service = client.create_client(Trigger, "/kstone/home")
        wait_for(service.service_is_ready)
        future = service.call_async(Trigger.Request())
        wait_for(future.done)
        assert future.result().success
        assert future.result().message.startswith("accepted request_id=")
    finally:
        client.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            # Include launch children in forced cleanup; the session is private.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5.0)
            pytest.fail(f"launch failed to shut down: {log_path.read_text(errors='replace')}")
        finally:
            log.close()
        output = log_path.read_text(errors="replace")
    assert process.returncode == 0, output
    assert "Traceback" not in output, output
    assert "process has died" not in output, output


@pytest.mark.parametrize("realtime", ["inherit", "false"])
def test_installed_launch_without_feedback_and_finite_inheritance(tmp_path, realtime):
    """The shipped unpaced YAML is inherited; launch flags must actually reach ROS."""
    env = os.environ.copy()
    env["ROS_LOG_DIR"] = str(tmp_path / "ros-log")
    path = tmp_path / "launch.log"
    log = path.open("w")
    process = subprocess.Popen(
        [shutil.which("ros2"), "launch", "kstone_ros", "kstone.launch.py",
         "namespace:=launch_feedback",
         "duration:=0.2", f"realtime:={realtime}"],
        cwd=tmp_path, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    client = Node("installed_feedback_launch_probe")
    discovery, states, velocities, joints = [], [], [], []
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    prefix = "/launch_feedback/kstone"
    client.create_subscription(String, prefix + "/discover",
                               lambda msg: discovery.append(json.loads(msg.data)), latched)
    client.create_subscription(String, prefix + "/state",
                               lambda msg: states.append(json.loads(msg.data)), 10)
    client.create_subscription(Twist, prefix + "/feedback/cmd_vel", velocities.append, 10)
    client.create_subscription(JointState, prefix + "/feedback/joint_states", joints.append, 10)

    def wait_for(predicate):
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                pytest.fail(f"launch exited early: {path.read_text(errors='replace')}")
            rclpy.spin_once(client, timeout_sec=0.02)
            if predicate():
                return
        pytest.fail(f"launch condition timed out: {path.read_text(errors='replace')}")

    try:
        wait_for(lambda: discovery and states
                 and states[-1]["fault"] == "duration_complete")
        assert not states[-1]["joints_attached"] and not states[-1]["base_attached"]
        assert any(endpoint["topic"] == prefix + "/odom"
                   for endpoint in discovery[-1]["interfaces"] if "topic" in endpoint)
        assert states[-1]["engine_time"] == pytest.approx(0.2)
        assert states[-1]["lifecycle"] == 3 and not states[-1]["written"]
        topics = {i["topic"]: i for i in discovery[-1]["interfaces"] if "topic" in i}
        assert topics[prefix + "/joint_states"]["direction"] == "subscribe"
        assert topics[prefix + "/feedback/cmd_vel"]["type"] == "geometry_msgs/msg/Twist"
        assert not joints and not velocities  # No driver output without real feedback.
        home = client.create_client(Trigger, prefix + "/home")
        wait_for(home.service_is_ready)
        future = home.call_async(Trigger.Request())
        wait_for(future.done)
        assert not future.result().success
    finally:
        client.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
            pytest.fail(f"launch did not stop: {path.read_text(errors='replace')}")
        finally:
            log.close()
    output = path.read_text(errors="replace")
    assert process.returncode == 0 and "Traceback" not in output, output
