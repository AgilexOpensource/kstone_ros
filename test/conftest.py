"""DDS clients and temporary subprocess fixtures for the kstone_node executable."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
import yaml

CONFIG = Path(__file__).resolve().parents[1] / "config" / "omni.yaml"


def config_variant(tmp_path, *, fixed=False, no_chassis=False):
    data = yaml.safe_load(CONFIG.read_text())
    data["model"]["file"] = str(CONFIG.with_suffix(".urdf"))
    if fixed:
        data["model"]["root"] = "fixed"
    if no_chassis:
        del data["modules"]["chassis"]
    path = tmp_path / "engine.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


@pytest.fixture
def executable():
    override = os.environ.get("KSTONE_ROS_EXECUTABLE")
    if override:
        path = Path(override).expanduser().resolve()
    else:
        from ament_index_python.packages import get_package_prefix
        try:
            prefix = get_package_prefix("kstone_ros")
        except LookupError:
            pytest.fail("requires installed kstone_ros or KSTONE_ROS_EXECUTABLE")
        path = Path(prefix) / "lib" / "kstone_ros" / "kstone_node"
    assert path.is_file() and os.access(path, os.X_OK), f"not an executable: {path}"
    return path


class NodeProcess:
    def __init__(self, executable, directory, options):
        directory.mkdir()
        namespace = options.pop("_namespace", "")
        remaps = options.pop("_remaps", {})
        requested_prefix = options.get("topic_prefix", "/kstone_test_" + uuid.uuid4().hex)
        self.prefix = (namespace.rstrip("/") + "/kstone") if requested_prefix == "~" else requested_prefix
        parameters = {
            "config_file": str(CONFIG), "topic_prefix": self.prefix, "realtime": True,
        }
        parameters.update(options)
        params = directory / "parameters.yaml"
        params.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
        self.log_path = directory / "process.log"
        self.log = self.log_path.open("w")
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = str(directory / "ros-log")
        try:
            command = [str(executable), "--ros-args", "--params-file", str(params)]
            if namespace:
                command += ["-r", "__ns:=" + namespace]
            for source, destination in remaps.items():
                command += ["-r", source + ":=" + destination]
            self.process = subprocess.Popen(
                command,
                cwd=directory, env=env, stdout=self.log, stderr=subprocess.STDOUT)
        except BaseException:
            self.log.close()
            raise

    def topic(self, suffix):
        return self.prefix + "/" + suffix

    def output(self):
        return self.log_path.read_text(errors="replace")

    def check_running(self):
        if self.process.poll() is not None:
            pytest.fail(f"kstone_node exited early:\n{self.output()}")

    def stop(self):
        try:
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
                pytest.fail(f"kstone_node did not shut down:\n{self.output()}")
        finally:
            self.log.close()


@pytest.fixture
def start_node(executable, tmp_path):
    processes = []

    def start(**options):
        process = NodeProcess(executable, tmp_path / f"node-{len(processes)}", options)
        processes.append(process)
        return process

    yield start
    for process in reversed(processes):
        process.stop()


class Probe:
    def __init__(self):
        self.node = Node("probe_" + uuid.uuid4().hex)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def wait(self, bridge, condition, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            bridge.check_running()
            self.executor.spin_once(timeout_sec=0.01)
            if condition():
                return
        pytest.fail(f"DDS condition timed out:\n{bridge.output()}")

    def spin_for(self, bridge, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            bridge.check_running()
            self.executor.spin_once(timeout_sec=0.01)

    def subscribe_json(self, topic, qos=10):
        messages = []
        self.node.create_subscription(
            String,
            topic, lambda msg: messages.append(json.loads(msg.data)), qos)
        return messages

    def call(self, bridge, service_type, suffix, request):
        service = self.node.create_client(service_type, bridge.topic(suffix))
        self.wait(bridge, service.service_is_ready)
        future = service.call_async(request)
        self.wait(bridge, future.done)
        response = future.result()
        assert response is not None
        return response


class EchoDriver:
    """Test-only driver: replays joint output and integrates cmd_vel output as real feedback."""

    def __init__(self, probe, bridge, names, root=True, period=0.02):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        self.joints = dict.fromkeys(names, 0.0)
        self.x = self.y = self.yaw = 0.0
        self.twist = None
        self.period = period
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        node = probe.node
        node.create_subscription(JointState, bridge.topic("feedback/joint_states"),
                                 lambda m: self.joints.update(zip(m.name, m.position)), 10)
        self.joint_pub = node.create_publisher(JointState, bridge.topic("joint_states"), qos)
        self.odom_pub = None
        if root:
            node.create_subscription(Twist, bridge.topic("feedback/cmd_vel"), self._twist, 10)
            self.odom_pub = node.create_publisher(Odometry, bridge.topic("odom"), qos)
        self._joint_type, self._odom_type = JointState, Odometry
        self.timer = node.create_timer(period, self.publish)

    def _twist(self, msg):
        self.twist = msg

    def publish(self):
        import math
        self.joint_pub.publish(self._joint_type(name=list(self.joints), position=list(self.joints.values())))
        if self.odom_pub is None:
            return
        if self.twist is not None:
            t, c, s = self.twist, math.cos(self.yaw), math.sin(self.yaw)
            self.x += (t.linear.x * c - t.linear.y * s) * self.period
            self.y += (t.linear.x * s + t.linear.y * c) * self.period
            self.yaw += t.angular.z * self.period
        odom = self._odom_type()
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2)
        self.odom_pub.publish(odom)

    def stop(self):
        self.timer.cancel()


@pytest.fixture
def probe():
    rclpy.init()
    client = Probe()
    try:
        yield client
    finally:
        client.executor.remove_node(client.node)
        client.node.destroy_node()
        client.executor.shutdown()
        rclpy.shutdown()
