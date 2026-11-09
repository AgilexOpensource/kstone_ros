"""Blackbox DDS contracts against the kstone_node subprocess."""
import json
import time

import pytest
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, SetBool
from tf2_msgs.msg import TFMessage

from conftest import EchoDriver, config_variant

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
JOINT_NAMES = ["slide_x", "slide_y", "slide_z", "finger_joint"]
INITIAL_POSITIONS = [0.0, 0.0, 0.0, 0.0]


def subscribe(probe, bridge, message_type, suffix, qos=10):
    messages = []
    probe.node.create_subscription(message_type, bridge.topic(suffix), messages.append, qos)
    return messages


def target_message(x=0.08, z=0.35):
    target = PoseStamped()
    target.header.frame_id = "base_link"
    target.pose.position.x = x
    target.pose.position.z = z
    target.pose.orientation.w = 1.0
    return target


def driver_twist(message):
    assert isinstance(message, Twist)
    assert not hasattr(message, "header")
    assert message.linear.z == message.angular.x == message.angular.y == 0


def assert_no_legacy_driver_topics(bridge, metadata, probe):
    obsolete = {bridge.topic(suffix) for suffix in (
        "driver/feedback/joint_states", "driver/feedback/odometry", "driver/control/joint_states",
        "driver/control/cmd_vel", "feedback/odometry", "odometry",
        "control/joint_states", "control/twist", "driver/control/twist", "twist", "twist_stamped",
        "joint_commands", "base_commands", "feedback_joints",
        "cmd_vel", "cmd_vel_stamped", "extra_joint_commands", "base_pose")}
    advertised = {i["topic"] for i in metadata["interfaces"] if "topic" in i}
    obsolete.update(bridge.topic(m["id"] + "/control/twist") for m in metadata["modules"])
    assert not obsolete & advertised
    graph = {name for name, _ in probe.node.get_topic_names_and_types()}
    assert not obsolete & graph
    assert not any(i["direction"] == "publish" and i.get("type") == "geometry_msgs/msg/TwistStamped"
                   for i in metadata["interfaces"])


def test_driver_io_private_remaps_qos_and_feedback_gating(start_node, probe, tmp_path):
    path = config_variant(tmp_path)
    config = yaml.safe_load(path.read_text())
    config["modules"]["chassis"]["chassis"]["base_frame"] = "body_center"
    path.write_text(yaml.safe_dump(config))
    namespace = "/driver_contract"
    prefix = namespace + "/kstone"
    remaps = {prefix + "/" + suffix: namespace + "/external/" + suffix
              for suffix in ("joint_states", "odom", "feedback/joint_states", "feedback/cmd_vel")}
    bridge = start_node(config_file=str(path), topic_prefix="~", _namespace=namespace, _remaps=remaps)
    discovery = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    states = probe.subscribe_json(bridge.topic("state"))
    commands, velocities = [], []
    joint_output = remaps[prefix + "/feedback/joint_states"]
    twist_output = remaps[prefix + "/feedback/cmd_vel"]
    probe.node.create_subscription(JointState, joint_output, commands.append, 10)
    probe.node.create_subscription(Twist, twist_output, velocities.append, 10)
    twist = probe.node.create_publisher(Twist, bridge.topic("chassis/control/cmd_vel"), 10)
    probe.wait(bridge, lambda: discovery and states and twist.get_subscription_count())
    interfaces = {i["topic"]: i for i in discovery[-1]["interfaces"] if "topic" in i}
    for topic, expected_type in ((joint_output, "sensor_msgs/msg/JointState"),
                                 (twist_output, "geometry_msgs/msg/Twist")):
        info = interfaces[topic]
        assert (info["direction"], info["type"], info["depth"], info["reliability"], info["durability"]) == (
            "publish", expected_type, 10, "reliable", "volatile")
    assert interfaces[twist_output]["frame"] == "body_center"
    assert_no_legacy_driver_topics(bridge, discovery[-1], probe)
    for topic in (joint_output, twist_output):
        info = probe.node.get_publishers_info_by_topic(topic)
        assert len(info) == 1
        assert info[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert info[0].qos_profile.durability == DurabilityPolicy.VOLATILE
    joint_input = remaps[prefix + "/joint_states"]
    odom_input = remaps[prefix + "/odom"]
    for topic, expected_type in ((joint_input, "sensor_msgs/msg/JointState"), (odom_input, "nav_msgs/msg/Odometry")):
        info = interfaces[topic]
        assert (info["direction"], info["type"], info["depth"], info["reliability"], info["durability"]) == (
            "subscribe", expected_type, 5, "best_effort", "volatile")
    sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    sensing = probe.node.create_publisher(JointState, joint_input, sensor_qos)
    odometry = probe.node.create_publisher(Odometry, odom_input, sensor_qos)
    probe.wait(bridge, lambda: sensing.get_subscription_count() and odometry.get_subscription_count())
    probe.spin_for(bridge, 0.2)
    assert not commands and not velocities  # No driver output before real feedback.
    assert not states[-1]["fault"] and not states[-1]["joints_attached"] and not states[-1]["base_attached"]
    velocity = Twist()
    velocity.linear.x = 0.08
    measured = Odometry()
    measured.header.frame_id = "world"
    measured.child_frame_id = "base_link"  # Root sensing frame, not chassis body_center.
    measured.pose.pose.orientation.w = 1.0
    def joints_only():
        sensing.publish(JointState(name=JOINT_NAMES, position=INITIAL_POSITIONS))
        return commands and states[-1]["joints_attached"]
    probe.wait(bridge, joints_only)
    assert not velocities and not states[-1]["base_attached"]  # Joint and base gating are separate.
    def moving():
        sensing.publish(JointState(name=JOINT_NAMES, position=INITIAL_POSITIONS))
        odometry.publish(measured)
        twist.publish(velocity)
        return commands and states[-1]["base_attached"] and velocities and velocities[-1].linear.x > 0.005
    probe.wait(bridge, moving)
    assert set(commands[-1].name) == set(JOINT_NAMES)
    assert len(commands[-1].position) == len(JOINT_NAMES)
    assert not states[-1]["fault"]
    driver_twist(velocities[-1])
    assert_no_legacy_driver_topics(bridge, discovery[-1], probe)
    probe.wait(bridge, lambda: not states[-1]["joints_attached"] and not states[-1]["base_attached"])
    probe.wait(bridge, lambda: (velocities[-1].linear.x, velocities[-1].linear.y, velocities[-1].angular.z) == (0, 0, 0))
    driver_twist(velocities[-1])
    probe.spin_for(bridge, 0.6)  # Past one command_timeout_s of zero hold.
    joints_before, twists_before = len(commands), len(velocities)
    probe.spin_for(bridge, 0.3)
    assert len(commands) == joints_before and len(velocities) == twists_before


def test_driver_io_explicit_topic_overrides_keep_bare_twist_output_type(start_node, probe, tmp_path):
    path = config_variant(tmp_path)
    config = yaml.safe_load(path.read_text())
    config["modules"]["chassis"]["chassis"]["base_frame"] = "body_center"
    path.write_text(yaml.safe_dump(config))
    prefix = "/driver_explicit/kstone"
    bridge = start_node(config_file=str(path), topic_prefix="~", _namespace="/driver_explicit",
                          joint_command_topic=prefix + "/joint_commands",
                          base_command_topic=prefix + "/base_commands",
                          feedback_joint_topic=prefix + "/feedback_joints")
    discovery = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    states = probe.subscribe_json(bridge.topic("state"))
    commands = subscribe(probe, bridge, JointState, "joint_commands")
    canonical = subscribe(probe, bridge, Twist, "feedback/cmd_vel")
    alias = subscribe(probe, bridge, Twist, "base_commands")
    sensing = probe.node.create_publisher(JointState, bridge.topic("feedback_joints"), 10)
    odometry = probe.node.create_publisher(Odometry, bridge.topic("odom"), 10)
    velocity = probe.node.create_publisher(Twist, bridge.topic("chassis/control/cmd_vel"), 10)
    probe.wait(bridge, lambda: discovery and states and sensing.get_subscription_count() and
               odometry.get_subscription_count() and velocity.get_subscription_count())
    endpoints = {i["topic"]: i for i in discovery[-1]["interfaces"] if "topic" in i}
    for suffix in ("feedback/cmd_vel", "base_commands"):
        info = endpoints[bridge.topic(suffix)]
        assert info["type"] == "geometry_msgs/msg/Twist" and info["frame"] == "body_center"
    assert endpoints[bridge.topic("joint_commands")]["type"] == "sensor_msgs/msg/JointState"
    assert endpoints[bridge.topic("feedback_joints")]["direction"] == "subscribe"
    assert endpoints[bridge.topic("joint_states")]["direction"] == "subscribe"
    message = Odometry()
    message.header.frame_id = "world"
    message.child_frame_id = "base_link"
    message.pose.pose.orientation.w = 1.0
    command = Twist()
    command.linear.x = 0.05
    def ready():
        sensing.publish(JointState(name=JOINT_NAMES, position=INITIAL_POSITIONS))
        odometry.publish(message)
        return commands and states[-1]["joints_attached"] and states[-1]["base_attached"]
    probe.wait(bridge, ready)
    def moving():
        sensing.publish(JointState(name=JOINT_NAMES, position=INITIAL_POSITIONS))
        odometry.publish(message)
        velocity.publish(command)
        return commands and canonical and alias and alias[-1].linear.x > 0.005
    probe.wait(bridge, moving)
    driver_twist(canonical[-1])
    driver_twist(alias[-1])
    values = lambda messages: {(m.linear.x, m.linear.y, m.angular.z) for m in messages}
    assert values(canonical) & values(alias)  # Both publications copy the same bare command.
    assert not any(i["direction"] == "publish" and i.get("type") == "geometry_msgs/msg/TwistStamped"
                   for i in discovery[-1]["interfaces"])
    probe.wait(bridge, lambda: not states[-1]["base_attached"])
    probe.spin_for(bridge, 0.15)
    for messages in (canonical, alias):
        assert (messages[-1].linear.x, messages[-1].linear.y, messages[-1].angular.z) == (0, 0, 0)
        driver_twist(messages[-1])


def test_dds_discovery_task_chassis_aggregate_and_services(start_node, probe):
    bridge = start_node()
    discovery = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    EchoDriver(probe, bridge, JOINT_NAMES)
    states = probe.subscribe_json(bridge.topic("state"))
    poses = subscribe(probe, bridge, PoseStamped, "tool_pose/feedback/pose")
    bases = subscribe(probe, bridge, PoseStamped, "chassis/feedback/pose")
    commands = subscribe(probe, bridge, JointState, "feedback/joint_states")
    target_pub = probe.node.create_publisher(PoseStamped, bridge.topic("tool_pose/control/pose"), 10)
    twist_pub = probe.node.create_publisher(Twist, bridge.topic("chassis/control/cmd_vel"), 10)
    aggregate_pub = probe.node.create_publisher(JointState, bridge.topic("control/joint"), 10)
    probe.wait(bridge, lambda: discovery and poses and states and commands and bases
               and all(pub.get_subscription_count() for pub in (target_pub, twist_pub, aggregate_pub)))
    assert {m["id"] for m in discovery[0]["modules"]} == {"chassis", "tool_pose", "gripper_joint"}
    assert "mode" not in discovery[0]
    assert_no_legacy_driver_topics(bridge, discovery[-1], probe)
    assert poses[-1].header.frame_id == "base_link"
    velocity = Twist()
    velocity.linear.x = 0.1
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        target_pub.publish(target_message())
        twist_pub.publish(velocity)
        aggregate_pub.publish(JointState(name=["finger_joint"], position=[0.02]))
        probe.spin_for(bridge, 0.02)
    probe.wait(bridge, lambda: poses[-1].pose.position.x > 0.03
               and bases[-1].pose.position.x > 0.03
               and dict(zip(commands[-1].name, commands[-1].position)).get("finger_joint", 0) > 0.005)
    assert not states[-1]["fault"]
    assert probe.call(bridge, Trigger, "home", Trigger.Request()).success


def test_private_short_services_and_latched_discovery(start_node, probe):
    namespace = "/short_names"
    prefix = namespace + "/kstone"
    remaps = {prefix + "/" + suffix: namespace + "/remapped/" + suffix
              for suffix in ("discover", "home", "tool_pose/home", "tool_pose/enable")}
    bridge = start_node(topic_prefix="~", _namespace=namespace, _remaps=remaps)
    states = probe.subscribe_json(bridge.topic("state"))
    probe.wait(bridge, lambda: states)
    # Subscribe only after startup publication: transient-local history is required.
    discovery = probe.subscribe_json(remaps[prefix + "/discover"], LATCHED)
    probe.wait(bridge, lambda: discovery)
    metadata = discovery[-1]
    interfaces = {i.get("topic", i.get("service")): i for i in metadata["interfaces"]}
    for endpoint in (remaps[prefix + "/discover"], prefix + "/state"):
        info = interfaces[endpoint]
        assert (info["type"], info["depth"], info["reliability"], info["durability"]) == (
            "std_msgs/msg/String", 1, "reliable", "transient_local")
        publishers = probe.node.get_publishers_info_by_topic(endpoint)
        assert len(publishers) == 1
        assert publishers[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
    for suffix, expected in (("home", "std_srvs/srv/Trigger"),
                              ("tool_pose/home", "std_srvs/srv/Trigger"),
                              ("tool_pose/enable", "std_srvs/srv/SetBool"),
                              ("gripper_joint/home", "std_srvs/srv/Trigger"),
                              ("gripper_joint/enable", "std_srvs/srv/SetBool"),
                              ("chassis/enable", "std_srvs/srv/SetBool")):
        endpoint = remaps.get(prefix + "/" + suffix, prefix + "/" + suffix)
        assert (interfaces[endpoint]["direction"], interfaces[endpoint]["type"]) == ("service", expected)
    graph_services = dict(probe.node.get_service_names_and_types())
    graph_topics = dict(probe.node.get_topic_names_and_types())
    assert remaps[prefix + "/discover"] not in graph_services
    obsolete = {prefix + "/" + suffix for suffix in (
        "modules", "modules/discover", "reset_home", "chassis/home",
        "tool_pose/set_enabled", "modules/tool_pose/enable", "modules/tool_pose/home", "modules/tool_pose/state",
        "modules/tool_pose/set_enabled", "reset_target", "recover", "chassis/state", "tool_pose/state")}
    assert not obsolete & (set(interfaces) | set(graph_services) | set(graph_topics))
    assert set(metadata["modules"][i]["id"] for i in range(len(metadata["modules"]))) == {
        "chassis", "tool_pose", "gripper_joint"}
    assert_no_legacy_driver_topics(bridge, metadata, probe)

    for suffix, service_type, request in (
            ("tool_pose/enable", SetBool, SetBool.Request(data=False)),
            ("tool_pose/enable", SetBool, SetBool.Request(data=True)),
            ("tool_pose/home", Trigger, Trigger.Request()),
            ("home", Trigger, Trigger.Request())):
        endpoint = remaps[prefix + "/" + suffix]
        client = probe.node.create_client(service_type, endpoint)
        probe.wait(bridge, client.service_is_ready)
        future = client.call_async(request)
        probe.wait(bridge, future.done)
        response = future.result()
        assert response.success and response.message.startswith("accepted request_id=")
        request_id = int(response.message.split("=")[-1])
        probe.wait(bridge, lambda: any(o["request_id"] == request_id for o in states[-1]["outcomes"]))
        outcome = next(o for o in states[-1]["outcomes"] if o["request_id"] == request_id)
        assert outcome["applied"], outcome


def test_dds_feedback_gates_commands_and_resumes_on_fresh_sensing(start_node, probe, tmp_path):
    config = config_variant(tmp_path, fixed=True, no_chassis=True)
    bridge = start_node(config_file=str(config))
    commands = subscribe(probe, bridge, JointState, "feedback/joint_states")
    states = probe.subscribe_json(bridge.topic("state"))
    feedback = probe.node.create_publisher(JointState, bridge.topic("joint_states"), 10)
    probe.wait(bridge, lambda: states and feedback.get_subscription_count())
    probe.spin_for(bridge, 0.2)
    assert not commands and not states[-1]["joints_attached"] and not states[-1]["fault"]
    # A single scalar joint must not make an incomplete measurement fresh.
    feedback.publish(JointState(name=["slide_x"], position=[0.0]))
    probe.spin_for(bridge, 0.1)
    assert not commands
    measured = [0.05, -0.02, 0.1, 0.01]
    def attached(position):
        # Attach needs feedback that stays fresh for the whole feedback_timeout_s window.
        feedback.publish(JointState(name=JOINT_NAMES, position=position))
        return commands and states[-1]["joints_attached"]
    probe.wait(bridge, lambda: attached(measured))
    # First output after attach equals the measured state: no jump.
    assert dict(zip(commands[0].name, commands[0].position)) == pytest.approx(dict(zip(JOINT_NAMES, measured)), abs=1e-6)
    probe.wait(bridge, lambda: not states[-1]["joints_attached"])
    # Drain DDS samples already in flight before testing publication suppression.
    probe.spin_for(bridge, 0.15)
    before = len(commands)
    probe.spin_for(bridge, 0.2)
    assert len(commands) == before and not states[-1]["fault"]
    probe.wait(bridge, lambda: attached(INITIAL_POSITIONS) and len(commands) > before)


def test_feedback_chassis_uses_base_odometry_and_gates_stale_base(start_node, probe):
    # Driver feedback must reach the chassis odometry API, not only joint q.
    base_topic = "/ns_base_feedback_" + str(time.monotonic_ns())
    bridge = start_node(feedback_base_topic=base_topic, feedback_timeout_s=0.5)
    poses = subscribe(probe, bridge, PoseStamped, "chassis/feedback/pose")
    commands = subscribe(probe, bridge, JointState, "feedback/joint_states")
    velocities = subscribe(probe, bridge, Twist, "feedback/cmd_vel")
    states = probe.subscribe_json(bridge.topic("state"))
    joints = probe.node.create_publisher(JointState, bridge.topic("joint_states"), 10)
    bases = probe.node.create_publisher(PoseStamped, base_topic, 10)
    probe.wait(bridge, lambda: states and joints.get_subscription_count() and bases.get_subscription_count())
    measured = PoseStamped()
    measured.header.frame_id = "world"
    measured.pose.position.x = 1.2
    measured.pose.position.y = 0.3
    measured.pose.orientation.w = 1.0
    message = JointState(name=JOINT_NAMES, position=INITIAL_POSITIONS)
    def attached():
        joints.publish(message)
        bases.publish(measured)
        return commands and poses and velocities and states[-1]["joints_attached"] and states[-1]["base_attached"]
    probe.wait(bridge, attached)
    probe.wait(bridge, lambda: poses[-1].pose.position.x == pytest.approx(1.2, abs=1e-3))
    assert poses[-1].pose.position.y == pytest.approx(0.3, abs=1e-3)
    # Keep scalar joints fresh but deliberately let the configured base expire.
    deadline = time.monotonic() + 0.9
    while time.monotonic() < deadline:
        joints.publish(message)
        probe.spin_for(bridge, 0.02)
    assert states[-1]["joints_attached"] and not states[-1]["base_attached"]
    before = len(commands)
    for _ in range(10):
        joints.publish(message)
        probe.spin_for(bridge, 0.02)
    assert len(commands) > before  # Base staleness only gates cmd_vel.
    assert [velocities[-1].linear.x, velocities[-1].linear.y,
            velocities[-1].angular.z] == pytest.approx([0, 0, 0])


@pytest.mark.parametrize("option,value", [
    ("feedback_joint_topic", "/kstone/feedback/joint_states"),
    ("feedback_base_topic", "/kstone/tool_pose/feedback/pose"),
])
def test_startup_rejects_feedback_output_alias(start_node, tmp_path, option, value):
    config = config_variant(tmp_path, no_chassis=True)
    bridge = start_node(config_file=str(config), topic_prefix="/kstone",
                          **{option: value})
    code = bridge.process.wait(timeout=10)
    assert code != 0, bridge.output()
    assert "alias" in bridge.output().lower(), bridge.output()


@pytest.mark.parametrize("no_chassis", [False, True])
def test_dds_root_tf_and_invalid_pose_rejection(start_node, probe, tmp_path, no_chassis):
    config = config_variant(tmp_path, no_chassis=no_chassis)
    bridge = start_node(config_file=str(config), publish_tf=True, command_timeout_s=0.2)
    transforms = []
    probe.node.create_subscription(TFMessage, "/tf", transforms.append, 10)
    states = probe.subscribe_json(bridge.topic("state"))
    publisher = probe.node.create_publisher(PoseStamped, bridge.topic("tool_pose/control/pose"), 10)
    probe.wait(bridge, lambda: transforms and states and publisher.get_subscription_count())
    discovery = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    probe.wait(bridge, lambda: discovery)
    assert_no_legacy_driver_topics(bridge, discovery[-1], probe)
    assert any(t.header.frame_id == "world" and t.child_frame_id == "base_link"
               for msg in transforms for t in msg.transforms)
    # Neither a stale stamped target nor an unrelated frame is accepted.
    submitted = states[-1]["submitted_inputs"]
    for invalid in ("stale", "frame"):
        target = target_message()
        if invalid == "stale":
            target.header.stamp = probe.node.get_clock().now().to_msg()
            target.header.stamp.sec -= 5
        else:
            target.header.frame_id = "unrelated_frame"
        publisher.publish(target)
        probe.spin_for(bridge, 0.25)
        assert states[-1]["submitted_inputs"] == submitted
    target = target_message()
    target.header.stamp = probe.node.get_clock().now().to_msg()
    publisher.publish(target)
    probe.wait(bridge, lambda: states[-1]["submitted_inputs"] == submitted + 1)
    # An accepted target holds without a refresh; the module stays enabled.
    probe.spin_for(bridge, 0.5)
    assert states[-1]["modules"]["tool_pose"]["enabled"] and not states[-1]["expired_modules"]



def test_private_namespaces_resolved_discovery_and_chassis(start_node, probe):
    bridge = start_node(topic_prefix="~", _namespace="/ns_a",
                          _remaps={"/ns_a/kstone/chassis/control/cmd_vel": "/ns_a/input/body_twist",
                                   "/ns_a/kstone/tool_pose/control/pose": "/ns_a/input/tool_pose"})
    other = start_node(topic_prefix="~", _namespace="/ns_b")
    EchoDriver(probe, bridge, JOINT_NAMES)
    discovery = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    other_discovery = probe.subscribe_json(other.topic("discover"), LATCHED)
    states = probe.subscribe_json(bridge.topic("state"))
    bases = subscribe(probe, bridge, PoseStamped, "chassis/feedback/pose")
    controls = subscribe(probe, bridge, Twist, "feedback/cmd_vel")
    pub = probe.node.create_publisher(Twist, "/ns_a/input/body_twist", 10)
    probe.wait(bridge, lambda: discovery and other_discovery and bases and controls and pub.get_subscription_count())
    interfaces = discovery[-1]["interfaces"]
    assert any(i.get("topic") == "/ns_a/input/body_twist" and i["type"] == "geometry_msgs/msg/Twist"
               and i["reliability"] == "reliable" for i in interfaces)
    assert all(i.get("topic", i.get("service", "")).startswith("/ns_a/") for i in interfaces)
    assert all(i.get("topic", i.get("service", "")).startswith("/ns_b/") for i in other_discovery[-1]["interfaces"])
    assert_no_legacy_driver_topics(bridge, discovery[-1], probe)
    assert_no_legacy_driver_topics(other, other_discovery[-1], probe)
    tool = next(m for m in discovery[-1]["modules"] if m["id"] == "tool_pose")
    port = next(p for p in tool["ports"] if p["name"] == "target")
    assert port["control_topic"] == "/ns_a/input/tool_pose"
    assert "command_topic" not in port and "value_topic" not in port
    assert any(i["topic"] == bridge.topic("tool_pose/feedback/pose") and i["semantics"] == "feedback"
               and i["source"] == "fk" for i in tool["ros_interfaces"])
    assert port["ros_frame"] == "base_link"
    assert not any("tune" in i.get("topic", "") for i in interfaces)
    message = Twist()
    message.linear.x = 0.1
    for _ in range(20):
        pub.publish(message)
        probe.spin_for(bridge, 0.02)
    probe.wait(bridge, lambda: bases[-1].pose.position.x > 0.005 and any(m.linear.x > 0.02 for m in controls))
    response = probe.call(bridge, SetBool, "chassis/enable", SetBool.Request(data=False))
    assert response.success and response.message.startswith("accepted request_id=")
    request_id = int(response.message.split("=")[-1])
    probe.wait(bridge, lambda: states and any(o["request_id"] == request_id and o["applied"] for o in states[-1]["outcomes"]))
    assert not states[-1]["modules"]["chassis"]["enabled"]
    probe.wait(bridge, lambda: controls[-1].linear.x == controls[-1].linear.y == controls[-1].angular.z == 0)
    driver_twist(controls[-1])


def test_progress_and_expiry_between_slow_ros_snapshot_consumptions(start_node, probe):
    bridge = start_node(state_rate_hz=0.5, command_timeout_s=0.1)
    states = probe.subscribe_json(bridge.topic("state"))
    pub = probe.node.create_publisher(Twist, bridge.topic("chassis/control/cmd_vel"), 10)
    probe.wait(bridge, lambda: states and pub.get_subscription_count())
    initial = states[-1]["cycles"]
    command = Twist()
    command.linear.x = 0.1
    pub.publish(command)
    probe.wait(bridge, lambda: states[-1]["cycles"] > initial + 70, timeout=6)
    # The port velocity timeout stops motion; ROS does not disable the module.
    assert not states[-1]["expired_modules"]
    assert states[-1]["modules"]["chassis"]["enabled"]
    assert not states[-1]["fault"]


def test_canonical_odometry_and_joint_feedback_route_to_integrated_chassis(start_node, probe):
    bridge = start_node()
    states = probe.subscribe_json(bridge.topic("state"))
    bases = subscribe(probe, bridge, PoseStamped, "chassis/feedback/pose")
    joints = probe.node.create_publisher(JointState, bridge.topic("joint_states"), 10)
    odom = probe.node.create_publisher(Odometry, bridge.topic("odom"), 10)
    probe.wait(bridge, lambda: states and joints.get_subscription_count() and odom.get_subscription_count())
    message = Odometry()
    message.header.frame_id = "world"
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = 1.0
    message.pose.pose.orientation.w = 1.0
    for _ in range(5):
        joints.publish(JointState(name=JOINT_NAMES, position=INITIAL_POSITIONS))
        odom.publish(message)
        probe.spin_for(bridge, 0.03)
    probe.wait(bridge, lambda: states[-1]["joints_attached"] and states[-1]["base_attached"]
               and bases[-1].pose.position.x == pytest.approx(1.0))
    probe.wait(bridge, lambda: not states[-1]["joints_attached"] and not states[-1]["base_attached"])
    assert not states[-1]["fault"]


@pytest.mark.parametrize("realtime", [False, True])
def test_finite_duration_safe_completion_and_rejected_restart(start_node, probe, realtime):
    bridge = start_node(duration=0.2, realtime=realtime)
    states = probe.subscribe_json(bridge.topic("state"))
    controls = subscribe(probe, bridge, Twist, "feedback/cmd_vel")
    probe.wait(bridge, lambda: states and states[-1]["fault"] == "duration_complete")
    assert states[-1]["engine_time"] == pytest.approx(0.2)
    assert not states[-1]["written"]
    probe.spin_for(bridge, 0.25)
    assert not controls  # Never attached to real feedback, so no driver output at all.
    assert states[-1]["engine_time"] == pytest.approx(0.2)
    response = probe.call(bridge, Trigger, "home", Trigger.Request())
    assert not response.success and "not running" in response.message


def test_three_type_topics_arbitrary_ids_and_removed_interfaces(start_node, tmp_path, probe):
    config = yaml.safe_load(config_variant(tmp_path).read_text())
    original = config["modules"]
    config["modules"] = {
        "rolling_base": original["chassis"],
        "pose_alpha": original["tool_pose"],
        "joint_beta": original["gripper_joint"],
        # Module IDs may contain a dash; unsupported types need no ROS token restrictions.
        "guard-only": {
            "type": "world_collision", "urdf": config["model"]["file"],
            "obstacles": [{"name": "obstacle", "shape": {"type": "sphere", "size": [0.1, 0, 0]}}],
        },
    }
    path = tmp_path / "arbitrary_topics.yaml"
    path.write_text(yaml.safe_dump(config))
    bridge = start_node(config_file=str(path), chassis_module="rolling_base")
    EchoDriver(probe, bridge, JOINT_NAMES)
    discover = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    states = probe.subscribe_json(bridge.topic("state"))
    outputs = subscribe(probe, bridge, Twist, "feedback/cmd_vel")
    probe.wait(bridge, lambda: discover and states)
    metadata = discover[-1]
    modules = {m["id"]: m for m in metadata["modules"]}
    for ident, port, suffix, message_type in [
        ("pose_alpha", "target", "control/pose", "geometry_msgs/msg/PoseStamped"),
        ("joint_beta", "target", None, "sensor_msgs/msg/JointState"),
        ("rolling_base", "velocity", "control/cmd_vel", "geometry_msgs/msg/Twist"),
        ("rolling_base", "pose_target", "control/pose", "geometry_msgs/msg/PoseStamped")]:
        assert modules[ident]["ros_topics_supported"] is True
        info = next(p for p in modules[ident]["ports"] if p["name"] == port)
        assert info["ros_exposed"] is True
        assert info["control_topic"] == bridge.topic(f"{ident}/{suffix}" if suffix else "control/joint")
        assert info["control_type"] == message_type
        assert "command_topic" not in info and "value_topic" not in info
        assert all(i["depth"] == 10 and i["durability"] == "volatile" and i["reliability"] == "reliable"
                   for i in modules[ident]["ros_interfaces"])
    guard_only = modules["guard-only"]
    assert guard_only["ros_topics_supported"] is False
    assert guard_only["ros_interfaces"] == []
    assert all(not p["ros_exposed"] for p in guard_only["ports"])
    assert {p["name"] for p in guard_only["ports"]} == {"obstacle", "obstacle/enabled"}
    assert all("command_topic" not in p and "value_topic" not in p for p in guard_only["ports"])
    interfaces = metadata["interfaces"]
    names = {i.get("topic", i.get("service")) for i in interfaces}
    assert_no_legacy_driver_topics(bridge, metadata, probe)
    assert bridge.topic("feedback/cmd_vel") in names
    assert not any(name.endswith(("/recover", "/reset_target")) for name in names)
    assert not any("guard-only" in name or "/ports/" in name or "/tasks/" in name for name in names)
    graph_services = {name for name, _ in probe.node.get_service_names_and_types()}
    assert not any(name.startswith(bridge.prefix + "/") and name.endswith(("/recover", "/reset_target"))
                   for name in graph_services)
    graph_topics = {name for name, _ in probe.node.get_topic_names_and_types()}
    assert not any(name.startswith(bridge.prefix + "/") and
                   ("guard-only" in name or "/ports/" in name or "/tasks/" in name) for name in graph_topics)
    velocity = probe.node.create_publisher(Twist, bridge.topic("rolling_base/control/cmd_vel"), 10)
    command = Twist(); command.linear.x = 0.04
    def moving():
        velocity.publish(command)
        return outputs and outputs[-1].linear.x > 0.005
    probe.wait(bridge, moving)
    before = states[-1]
    assert "guard-only" in before["modules"] and not before["fault"]
    probe.wait(bridge, lambda: states[-1]["cycles"] > before["cycles"])


@pytest.mark.parametrize("remapped", [False, True])
def test_typed_current_fk_and_named_joint_input_without_joint_telemetry(start_node, probe, tmp_path, remapped):
    path = config_variant(tmp_path, fixed=True, no_chassis=True)
    config = yaml.safe_load(path.read_text())
    original = config["modules"]
    original["gripper_joint"]["joints"] = ["slide_y", "finger_joint", "slide_x"]
    config["modules"] = {"pose_alpha": original["tool_pose"], "joint_beta": original["gripper_joint"]}
    path.write_text(yaml.safe_dump(config))
    joint_endpoint = "/typed_current/input/joint_target" if remapped else "/typed_current/kstone/control/joint"
    bridge = start_node(config_file=str(path), topic_prefix="~",
                          _namespace="/typed_current", command_timeout_s=2.0, state_rate_hz=60.0,
                          feedback_timeout_s=0.5,
                          _remaps={"~/control/joint": joint_endpoint} if remapped else {})
    poses = subscribe(probe, bridge, PoseStamped, "pose_alpha/feedback/pose")
    states = probe.subscribe_json(bridge.topic("state"))
    sensing = probe.node.create_publisher(JointState, bridge.topic("joint_states"), 10)
    pose = probe.node.create_publisher(PoseStamped, bridge.topic("pose_alpha/control/pose"), 10)
    joint = probe.node.create_publisher(JointState, joint_endpoint, 10)
    probe.wait(bridge, lambda: poses and states and
               all(p.get_subscription_count() for p in (sensing, pose, joint)))
    measured = JointState(name=JOINT_NAMES, position=[0.01, 0.02, 0.03, 0.004])
    expected = [0.08, 0.03, 0.12]
    def accepted_with_current_feedback():
        sensing.publish(measured)
        pose.publish(target_message(x=0.12))
        joint.publish(JointState(name=["slide_x", "slide_y", "finger_joint"],
                                 position=[0.12, 0.08, 0.03],
                                 velocity=[float("nan")], effort=[float("inf")]))
        return (states[-1]["written"] and
                states[-1]["modules"]["joint_beta"]["ports"]["target"] == pytest.approx(expected) and
                states[-1]["modules"]["pose_alpha"]["ports"]["target"]["position"][0] == pytest.approx(0.12) and
                poses[-1].pose.position.x == pytest.approx(0.01) and
                poses[-1].pose.position.y == pytest.approx(0.02) and
                poses[-1].pose.position.z == pytest.approx(0.38))
    probe.wait(bridge, accepted_with_current_feedback)
    assert poses[-1].header.frame_id == "base_link"
    discovery = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    probe.wait(bridge, lambda: discovery)
    module = next(m for m in discovery[-1]["modules"] if m["id"] == "joint_beta")
    assert module["ports"][0]["labels"] == ["slide_y", "finger_joint", "slide_x"]
    assert len(module["ros_interfaces"]) == 1 and module["ros_interfaces"][0]["direction"] == "subscribe"
    interface = module["ros_interfaces"][0]
    assert (interface["topic"], interface["type"]) == (joint_endpoint, "sensor_msgs/msg/JointState")
    assert module["ports"][0]["control_topic"] == joint_endpoint
    assert any(i.get("topic") == joint_endpoint and i["type"] == "sensor_msgs/msg/JointState"
               for i in discovery[-1]["interfaces"])
    assert dict(probe.node.get_topic_names_and_types())[joint_endpoint] == ["sensor_msgs/msg/JointState"]
    if remapped:
        assert bridge.topic("control/joint") not in dict(probe.node.get_topic_names_and_types())

    # Drain valid commands, then prove every malformed command is rejected atomically.
    sensing.publish(measured)
    probe.spin_for(bridge, 0.3)
    submitted = states[-1]["submitted_inputs"]
    for invalid in ("empty", "duplicate", "mismatch", "extra_position", "nan", "inf",
                    "frame", "stale", "future"):
        message = JointState(name=["slide_x", "finger_joint", "slide_y"], position=[0.07, 0.02, 0.05])
        if invalid == "empty":
            message.name = []
        elif invalid == "duplicate":
            message.name[-1] = "slide_x"
        elif invalid == "mismatch":
            message.position.pop()
        elif invalid == "extra_position":
            message.position.append(0.0)
        elif invalid in ("nan", "inf"):
            message.position[-1] = float(invalid)
        elif invalid == "frame":
            message.header.frame_id = "base_link"
        else:
            message.header.stamp = probe.node.get_clock().now().to_msg()
            message.header.stamp.sec += 5 if invalid == "future" else -5
        sensing.publish(measured)
        joint.publish(message)
        probe.spin_for(bridge, 0.08)
        assert states[-1]["submitted_inputs"] == submitted, invalid
        assert states[-1]["modules"]["joint_beta"]["ports"]["target"] == pytest.approx(expected), invalid

    current = JointState(name=["finger_joint", "slide_x", "slide_y"], position=[0.02, 0.1, 0.06])
    def accept_current_stamp():
        sensing.publish(measured)
        current.header.stamp = probe.node.get_clock().now().to_msg()
        joint.publish(current)
        return states[-1]["modules"]["joint_beta"]["ports"]["target"] == pytest.approx([0.06, 0.02, 0.1])
    probe.wait(bridge, accept_current_stamp)
    # Invalid traffic is rejected and the accepted target holds.
    bad = JointState(name=["slide_x", "slide_x", "slide_y"], position=[0.04, 0.02, 0.03])
    for _ in range(10):
        sensing.publish(measured)
        joint.publish(bad)
        probe.spin_for(bridge, 0.05)
    assert states[-1]["modules"]["joint_beta"]["enabled"]
    assert states[-1]["modules"]["joint_beta"]["ports"]["target"] == pytest.approx([0.06, 0.02, 0.1])
    assert poses[-1].pose.position.x != pytest.approx(0.12)
    graph = {name for name, _ in probe.node.get_topic_names_and_types()}
    assert bridge.topic("joint_beta/feedback/joint") not in graph
    assert not any(name.startswith(bridge.prefix + "/") and
                   ("/ports/" in name or "/tasks/" in name) for name in graph)


@pytest.mark.parametrize("kinematics", ["omni", "differential"])
def test_typed_chassis_planar_pose_validation_and_differential_twist_only(start_node, probe, tmp_path, kinematics):
    path = config_variant(tmp_path)
    config = yaml.safe_load(path.read_text())
    base = config["modules"].pop("chassis")
    base["chassis"]["kinematics"] = kinematics
    base["chassis"]["base_frame"] = "body_center"
    base["center_in_root"] = {"position": [0.2, 0.0, 0.0]}
    config["modules"]["rolling_base"] = base
    path.write_text(yaml.safe_dump(config))
    bridge = start_node(config_file=str(path), chassis_module="rolling_base",
                          topic_prefix="~", _namespace=f"/typed_{kinematics}", publish_tf=True)
    EchoDriver(probe, bridge, JOINT_NAMES)
    discovery = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    states = probe.subscribe_json(bridge.topic("state"))
    poses = subscribe(probe, bridge, PoseStamped, "rolling_base/feedback/pose")
    roots = []
    probe.node.create_subscription(TFMessage, "/tf",
        lambda msg: roots.extend(t for t in msg.transforms if t.child_frame_id == "base_link"), 10)
    controls = subscribe(probe, bridge, Twist, "feedback/cmd_vel")
    twist = probe.node.create_publisher(Twist, bridge.topic("rolling_base/control/cmd_vel"), 10)
    probe.wait(bridge, lambda: discovery and states and poses and roots and twist.get_subscription_count())
    module = next(m for m in discovery[-1]["modules"] if m["id"] == "rolling_base")
    assert module["chassis_kinematics"] == kinematics
    assert module["chassis_frame"] == "body_center"
    endpoints = {i["topic"]: i for i in module["ros_interfaces"]}
    assert endpoints[bridge.topic("rolling_base/control/cmd_vel")]["frame"] == "body_center"
    assert endpoints[bridge.topic("rolling_base/feedback/pose")]["source"] == "odometry"
    assert poses[-1].header.frame_id == "world"
    # Pose feedback runs at state_rate_hz and TF every tick, so compare once both settle.
    probe.wait(bridge, lambda: poses[-1].pose.position.x == pytest.approx(roots[-1].transform.translation.x + 0.2))
    pose_info = next(p for p in module["ports"] if p["name"] == "pose_target")
    pose_topic = bridge.topic("rolling_base/control/pose")
    graph = {name for name, _ in probe.node.get_topic_names_and_types()}
    assert not any(name.startswith(bridge.prefix + "/") and
                   ("/ports/" in name or "/tasks/" in name) for name in graph)
    assert (pose_topic in graph) == (kinematics == "omni")
    assert (pose_topic in endpoints) == (kinematics == "omni")
    assert pose_info["ros_exposed"] == (kinematics == "omni")
    target = probe.node.create_publisher(PoseStamped, pose_topic, 10)
    before = states[-1]["modules"]["rolling_base"]["ports"]["pose_target"]
    if kinematics == "omni":
        probe.wait(bridge, lambda: target.get_subscription_count())
        for invalid in ["z", "tilt", "frame", "stale", "future", "nan", "zero_quaternion", "nan_quaternion"]:
            message = PoseStamped()
            message.header.frame_id = "world"
            message.pose.position.x = 0.08
            message.pose.orientation.w = 1.0
            if invalid == "z":
                message.pose.position.z = 0.01
            elif invalid == "tilt":
                message.pose.orientation.x = 0.1
            elif invalid == "frame":
                message.header.frame_id = "body_center"
            elif invalid in ("stale", "future"):
                message.header.stamp = probe.node.get_clock().now().to_msg()
                message.header.stamp.sec += 5 if invalid == "future" else -5
            elif invalid == "nan":
                message.pose.position.x = float("nan")
            elif invalid == "zero_quaternion":
                message.pose.orientation.w = 0.0
            else:
                message.pose.orientation.w = float("nan")
            target.publish(message)
            probe.spin_for(bridge, 0.05)
        probe.spin_for(bridge, 0.15)
        assert states[-1]["modules"]["rolling_base"]["ports"]["pose_target"] == before
        assert "rolling_base" not in states[-1]["expired_modules"]
        import math
        message = PoseStamped()
        message.header.frame_id = "world"
        message.pose.position.x, message.pose.position.y = 0.08, -0.02
        # Non-unit yaw quaternion is normalized before projecting x/y/yaw.
        message.pose.orientation.w = 2 * math.cos(0.1)
        message.pose.orientation.z = 2 * math.sin(0.1)
        target.publish(message)
        probe.wait(bridge, lambda: states[-1]["modules"]["rolling_base"]["ports"]["pose_target"] ==
                   pytest.approx([0.08, -0.02, 0.2]))
    else:
        assert "control_topic" not in pose_info
        probe.spin_for(bridge, 0.15)
        assert target.get_subscription_count() == 0
        target.publish(target_message())
        probe.spin_for(bridge, 0.15)
        assert states[-1]["modules"]["rolling_base"]["ports"]["pose_target"] == before
    old = probe.node.create_publisher(Float64MultiArray,
                                     bridge.topic("modules/rolling_base/ports/pose_target/command"), 10)
    probe.spin_for(bridge, 0.1)
    assert old.get_subscription_count() == 0
    velocity = Twist()
    velocity.linear.x = 0.04
    def moving():
        twist.publish(velocity)
        return controls and controls[-1].linear.x > 0.005
    probe.wait(bridge, moving)
    assert not states[-1]["fault"]


@pytest.mark.parametrize("remapped", [False, True])
def test_aggregate_joint_targets_routing_validation_order_and_lease(start_node, probe, tmp_path, remapped):
    path = config_variant(tmp_path, fixed=True, no_chassis=True)
    config = yaml.safe_load(path.read_text())
    hand = dict(config["modules"].pop("gripper_joint"))
    hand["joints"] = ["finger_joint", "slide_x"]
    config["modules"]["hand_target"] = hand
    config["modules"]["arm_joint"] = dict(hand, joints=["slide_y"])
    path.write_text(yaml.safe_dump(config))
    endpoint = "/aggregate_targets/input/whole_body" if remapped else "/aggregate_targets/kstone/control/joint"
    bridge = start_node(config_file=str(path), topic_prefix="~", _namespace="/aggregate_targets",
                          command_timeout_s=2.0, state_rate_hz=60.0,
                          _remaps={"~/control/joint": endpoint} if remapped else {})
    states = probe.subscribe_json(bridge.topic("state"))
    discovery = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    aggregate = probe.node.create_publisher(JointState, endpoint, 10)
    probe.wait(bridge, lambda: states and discovery and aggregate.get_subscription_count())
    interfaces = {i["topic"]: i for i in discovery[-1]["interfaces"] if "topic" in i}
    info = interfaces[endpoint]
    assert (info["direction"], info["type"], info["routing"]) == (
        "subscribe", "sensor_msgs/msg/JointState", "by_joint_name")
    assert {r["module"]: r["joint_names"] for r in info["module_routes"]} == {
        "hand_target": ["finger_joint", "slide_x"], "arm_joint": ["slide_y"]}
    assert interfaces[bridge.topic("feedback/joint_states")]["direction"] == "publish"
    assert interfaces[bridge.topic("joint_states")]["direction"] == "subscribe"
    assert_no_legacy_driver_topics(bridge, discovery[-1], probe)
    assert dict(probe.node.get_topic_names_and_types())[endpoint] == ["sensor_msgs/msg/JointState"]
    if remapped:
        assert bridge.topic("control/joint") not in dict(probe.node.get_topic_names_and_types())
    before_pose = states[-1]["modules"]["tool_pose"]["ports"]["target"]

    def values():
        modules = states[-1]["modules"]
        return modules["hand_target"]["ports"]["target"], modules["arm_joint"]["ports"]["target"]
    def accept(pub, names, positions, expected):
        message = JointState(name=names, position=positions, velocity=[float("nan")], effort=[float("inf")])
        def applied():
            message.header.stamp = probe.node.get_clock().now().to_msg()
            pub.publish(message)
            a, b = values()
            return a == pytest.approx(expected[0]) and b == pytest.approx(expected[1])
        probe.wait(bridge, applied)
        probe.spin_for(bridge, 0.2)

    accept(aggregate, ["slide_y", "slide_x", "finger_joint"], [0.08, 0.12, 0.02], ([0.02, 0.12], [0.08]))
    assert states[-1]["modules"]["tool_pose"]["ports"]["target"] == before_pose
    # Any subset is valid; omitted joints keep the module's current target.
    accept(aggregate, ["slide_y"], [0.05], ([0.02, 0.12], [0.05]))
    accept(aggregate, ["slide_x"], [0.1], ([0.02, 0.1], [0.05]))
    accept(aggregate, ["slide_x"], [0.12], ([0.02, 0.12], [0.05]))
    # Names outside every joint_target chain are ignored; the rest still applies.
    accept(aggregate, ["slide_x", "slide_z"], [0.11, 0.3], ([0.02, 0.11], [0.05]))
    accept(aggregate, ["slide_x", ""], [0.12, 0.0], ([0.02, 0.12], [0.05]))
    submitted = states[-1]["submitted_inputs"]
    for invalid in ("empty", "duplicate", "mismatch", "nan", "inf", "frame", "stale", "future"):
        message = JointState(name=["finger_joint", "slide_x", "slide_y"], position=[0.03, 0.07, 0.06])
        if invalid == "empty":
            message.name = []; message.position = []
        elif invalid == "duplicate":
            message.name[-1] = "finger_joint"
        elif invalid == "mismatch":
            message.position.pop()
        elif invalid in ("nan", "inf"):
            message.position[-1] = float(invalid)
        elif invalid == "frame":
            message.header.frame_id = "base_link"
        else:
            message.header.stamp = probe.node.get_clock().now().to_msg()
            message.header.stamp.sec += 5 if invalid == "future" else -5
        aggregate.publish(message)
        probe.spin_for(bridge, 0.08)
        assert values()[0] == pytest.approx([0.02, 0.12]), invalid
        assert values()[1] == pytest.approx([0.05]), invalid
        assert states[-1]["submitted_inputs"] == submitted, invalid

    assert bridge.topic("hand_target/control/joint") not in dict(probe.node.get_topic_names_and_types())
    accept(aggregate, ["slide_x", "finger_joint"], [0.09, 0.025], ([0.025, 0.09], [0.05]))
    accept(aggregate, ["slide_y", "finger_joint", "slide_x"], [0.07, 0.015, 0.06], ([0.015, 0.06], [0.07]))
    accept(aggregate, ["finger_joint", "slide_x"], [0.03, 0.11], ([0.03, 0.11], [0.07]))
    bad = JointState(name=["finger_joint", "slide_y", "finger_joint"], position=[0.01, 0.02, 0.01])
    for _ in range(10):
        aggregate.publish(bad)
        probe.spin_for(bridge, 0.05)
    assert states[-1]["modules"]["hand_target"]["enabled"] and states[-1]["modules"]["arm_joint"]["enabled"]
    assert values()[0] == pytest.approx([0.03, 0.11]) and values()[1] == pytest.approx([0.07])
    assert states[-1]["modules"]["tool_pose"]["ports"]["target"] == before_pose


def test_aggregate_joint_targets_ambiguous_ownership_is_rejected(start_node, probe, tmp_path):
    path = config_variant(tmp_path, fixed=True, no_chassis=True)
    config = yaml.safe_load(path.read_text())
    hand = dict(config["modules"].pop("gripper_joint"), joints=["finger_joint", "slide_x"])
    config["modules"]["hand_target"] = hand
    config["modules"]["overlapping"] = dict(hand, joints=["slide_x", "slide_y"])
    path.write_text(yaml.safe_dump(config))
    bridge = start_node(config_file=str(path), state_rate_hz=60.0)
    states = probe.subscribe_json(bridge.topic("state"))
    aggregate = probe.node.create_publisher(JointState, bridge.topic("control/joint"), 10)
    probe.wait(bridge, lambda: states and aggregate.get_subscription_count())
    submitted = states[-1]["submitted_inputs"]
    aggregate.publish(JointState(name=["finger_joint", "slide_x", "slide_y"], position=[0.02, 0.1, 0.05]))
    probe.spin_for(bridge, 0.2)
    assert states[-1]["submitted_inputs"] == submitted
    assert states[-1]["modules"]["hand_target"]["ports"]["target"] == pytest.approx([0, 0])
    assert states[-1]["modules"]["overlapping"]["ports"]["target"] == pytest.approx([0, 0])
    assert bridge.topic("hand_target/control/joint") not in dict(probe.node.get_topic_names_and_types())


def test_discovery_presentation_driver_engine_module_topics_then_services(start_node, probe):
    bridge = start_node(topic_prefix="~", _namespace="/ordered_contract")
    discovery = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    probe.wait(bridge, lambda: discovery)
    info = discovery[-1]
    assert_no_legacy_driver_topics(bridge, info, probe)
    driver = ["joint_states", "odom", "feedback/joint_states", "feedback/cmd_vel"]
    engine = ["state", "discover", "control/joint"]
    ranks = {bridge.topic(name): i for i, name in enumerate(driver + engine)}
    module_names = ["state", "feedback/pose", "control/pose", "control/cmd_vel", "enable", "home"]
    for module in info["modules"]:
        for i, name in enumerate(module_names):
            ranks[bridge.topic(module["id"] + "/" + name)] = 11 + i if i < 4 else 13 + i
    ranks[bridge.topic("home")] = 16
    def check_order(records):
        values = [ranks[r.get("topic", r.get("service"))] for r in records]
        assert values == sorted(values)
        services_seen = False
        for record in records:
            if record["direction"] == "service":
                services_seen = True
            else:
                assert not services_seen
    check_order(info["interfaces"])
    for module in info["modules"]:
        check_order(module["ros_interfaces"])
    actual = {r.get("topic", r.get("service")) for r in info["interfaces"]}
    assert {bridge.topic(name) for name in engine + driver}.issubset(actual)


@pytest.mark.parametrize("kinematics", ["omni", "differential"])
@pytest.mark.parametrize("remapped", [False, True])
def test_module_cmd_vel_bare_type_validation_and_receipt_lease(start_node, probe, tmp_path, kinematics, remapped):
    path = config_variant(tmp_path)
    config = yaml.safe_load(path.read_text())
    chassis = config["modules"].pop("chassis")
    chassis["chassis"]["kinematics"] = kinematics
    chassis["chassis"]["base_frame"] = "control_center"
    config["modules"]["mobile_platform"] = chassis
    path.write_text(yaml.safe_dump(config))
    namespace = f"/cmd_vel_{kinematics}_{remapped}"
    original = namespace + "/kstone/mobile_platform/control/cmd_vel"
    endpoint = namespace + "/input/cmd_vel" if remapped else original
    # No chassis_module override: an arbitrary module ID is selected by default.
    bridge = start_node(config_file=str(path), topic_prefix="~", _namespace=namespace,
                          command_timeout_s=0.3, state_rate_hz=60.0,
                          _remaps={original: endpoint} if remapped else {})
    EchoDriver(probe, bridge, JOINT_NAMES)
    states = probe.subscribe_json(bridge.topic("state"))
    discovery = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    controls = subscribe(probe, bridge, Twist, "feedback/cmd_vel")
    publisher = probe.node.create_publisher(Twist, endpoint, 10)
    probe.wait(bridge, lambda: states and discovery and controls and publisher.get_subscription_count())
    metadata = discovery[-1]
    interfaces = {i["topic"]: i for i in metadata["interfaces"] if "topic" in i}
    assert (interfaces[endpoint]["type"], interfaces[endpoint]["direction"], interfaces[endpoint]["frame"]) == (
        "geometry_msgs/msg/Twist", "subscribe", "control_center")
    info = interfaces[bridge.topic("feedback/cmd_vel")]
    assert (info["type"], info["direction"], info["frame"]) == (
        "geometry_msgs/msg/Twist", "publish", "control_center")
    module = next(m for m in metadata["modules"] if m["id"] == "mobile_platform")
    velocity_port = next(p for p in module["ports"] if p["name"] == "velocity")
    assert (velocity_port["control_topic"], velocity_port["control_type"], velocity_port["ros_frame"]) == (
        endpoint, "geometry_msgs/msg/Twist", "control_center")
    assert dict(probe.node.get_topic_names_and_types())[endpoint] == ["geometry_msgs/msg/Twist"]
    if remapped:
        assert original not in dict(probe.node.get_topic_names_and_types())
    assert_no_legacy_driver_topics(bridge, metadata, probe)
    submitted = states[-1]["submitted_inputs"]
    invalid = []
    for part, axis in (("linear", "x"), ("linear", "y"), ("linear", "z"),
                       ("angular", "x"), ("angular", "y"), ("angular", "z")):
        for value in (float("nan"), float("inf")):
            message = Twist()
            setattr(getattr(message, part), axis, value)
            invalid.append(message)
    for part, axis in (("linear", "z"), ("angular", "x"), ("angular", "y")):
        message = Twist()
        setattr(getattr(message, part), axis, 0.01)
        invalid.append(message)
    if kinematics == "differential":
        message = Twist(); message.linear.y = 0.01
        invalid.append(message)
    for message in invalid:
        publisher.publish(message)
        probe.spin_for(bridge, 0.015)
    probe.spin_for(bridge, 0.1)
    assert states[-1]["submitted_inputs"] == submitted
    assert "mobile_platform" not in states[-1]["expired_modules"]
    command = Twist(); command.linear.x = 0.05
    def moving():
        publisher.publish(command)
        return controls[-1].linear.x > 0.005 and states[-1]["submitted_inputs"] > submitted
    probe.wait(bridge, moving)
    probe.spin_for(bridge, 0.05)
    accepted = states[-1]["submitted_inputs"]
    # Rejected messages cannot keep motion alive past the port velocity timeout.
    deadline = time.monotonic() + 0.45
    while time.monotonic() < deadline:
        publisher.publish(invalid[0])
        probe.spin_for(bridge, 0.015)
    probe.wait(bridge, lambda: controls[-1].linear.x == controls[-1].linear.y == controls[-1].angular.z == 0)
    assert states[-1]["submitted_inputs"] == accepted
    assert states[-1]["modules"]["mobile_platform"]["enabled"]
    driver_twist(controls[-1])
