"""YAML/configuration regressions through the installed ROS executable."""
import json
from pathlib import Path
import re
import shlex
import subprocess
from urllib.parse import unquote, urlsplit

import pytest
import yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from conftest import CONFIG, EchoDriver, config_variant
from test_ros import JOINT_NAMES, LATCHED, subscribe, assert_no_legacy_driver_topics


def test_include_and_variable_rate(start_node, probe, tmp_path):
    inner = config_variant(tmp_path, fixed=True, no_chassis=True)
    data = yaml.safe_load(inner.read_text())
    data["rate_hz"] = "${rate}"
    data["modules"]["tool_pose"]["home"] = {
        "position": ["${home_x}", 0, 0.2], "quaternion": [1, 0, 0, 0]}
    inner.write_text(yaml.safe_dump(data))
    outer = tmp_path / "include.yaml"
    outer.write_text(f"!include {inner.name}\n")
    bridge = start_node(config_file=str(outer),
                          config_variables=json.dumps({"rate": "50", "home_x": "0.03"}))
    discovery = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    states = probe.subscribe_json(bridge.topic("state"))
    probe.wait(bridge, lambda: discovery and states)
    home = next(m for m in discovery[-1]["modules"] if m["id"] == "tool_pose")["ports"][0]["home_pose"]
    assert len(discovery[0]["modules"]) == 2
    assert home["position"][0] == pytest.approx(0.03)
    assert home["position"][2] == pytest.approx(0.2)
    assert home["orientation"][0] == pytest.approx(1.0)
    assert not states[-1]["fault"] and not states[-1]["joints_attached"]


@pytest.mark.parametrize("rotation,expected", [
    ({"quaternion": [1, 0, 0, 0]}, [1, 0, 0, 0]),
    ({"rpy": [0, 0, 3.141592653589793]}, [0, 0, 0, 1]),
    ({}, [1, 0, 0, 0]),
])
def test_literal_start_home_schema(start_node, probe, tmp_path, rotation, expected):
    path = config_variant(tmp_path, fixed=True, no_chassis=True)
    data = yaml.safe_load(path.read_text())
    data["modules"]["tool_pose"]["home"] = {"position": [0.03, 0.02, 0.3], **rotation}
    path.write_text(yaml.safe_dump(data))
    bridge = start_node(config_file=str(path))
    discovery = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    probe.wait(bridge, lambda: discovery)
    target = next(m for m in discovery[-1]["modules"] if m["id"] == "tool_pose")["ports"][0]
    assert target["home_frame"] == "base_link"
    home = target["home_pose"]
    assert home["position"] == pytest.approx([0.03, 0.02, 0.3])
    # Immutable configured home metadata preserves YAML schema validation.
    actual = home["orientation"]
    assert actual == pytest.approx(expected, abs=1e-9) or actual == pytest.approx([-v for v in expected], abs=1e-9)


def test_manually_disabled_module_requires_explicit_enable(start_node, probe):
    bridge = start_node(command_timeout_s=2.0)
    states = []
    probe.node.create_subscription(String, bridge.topic("state"),
                                   lambda m: states.append(json.loads(m.data)["modules"]["gripper_joint"]), 10)
    publisher = probe.node.create_publisher(
        JointState, bridge.topic("control/joint"), 10)
    probe.wait(bridge, lambda: states and publisher.get_subscription_count())
    assert probe.call(bridge, SetBool, "gripper_joint/enable", SetBool.Request(data=False)).success
    probe.wait(bridge, lambda: not states[-1]["enabled"])
    before = states[-1]["ports"]["target"]
    publisher.publish(JointState(name=["finger_joint"], position=[0.02]))
    probe.spin_for(bridge, 0.2)
    assert not states[-1]["enabled"]
    assert states[-1]["ports"]["target"] == before
    assert probe.call(bridge, SetBool, "gripper_joint/enable", SetBool.Request(data=True)).success
    publisher.publish(JointState(name=["finger_joint"], position=[0.02]))
    probe.wait(bridge, lambda: states[-1]["enabled"]
               and states[-1]["ports"]["target"] == pytest.approx([0.02]))


_DOC_KINDS = {"pub": "pub", "发布": "pub", "sub": "sub", "订阅": "sub",
              "srv": "srv", "服务": "srv"}
_SRC_KINDS = {"publisher": "pub", "subscribe": "sub", "service": "srv"}


def _ros_type(cpp):
    """`sensor_msgs::msg::JointState` -> `sensor_msgs/JointState`."""
    parts = cpp.split("::")
    return f"{parts[0]}/{parts[-1]}"


def documented_interfaces(text):
    """(name, kind, type) rows of the README interface table, layout-independent."""
    rows = []
    for line in text.splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 2:
            continue
        names = re.findall(r"`([^`]+)`", cells[0])
        if not names or not all(n.startswith("~/") for n in names):
            continue
        kind = re.match(r"\s*([^\s`]+)", cells[1])
        types = re.findall(r"`([^`]+)`", cells[1])
        assert kind and kind[1].lower() in _DOC_KINDS and len(types) == 1, line
        msg = types[0].replace("/msg/", "/").replace("/srv/", "/")
        rows += [(n, _DOC_KINDS[kind[1].lower()], msg) for n in names]
    return rows


def source_interfaces(source):
    """Default (`~/...`) endpoints the node registers; per-module ids become `M`."""
    aliases = {a: _ros_type(t) for a, t in
               re.findall(r"^using (\w+) = ([\w:]+);", source, re.M)}
    variables = {}

    def resolve(expr):
        expr = " ".join(expr.split())
        if expr in variables:
            return variables[expr]
        joined = re.fullmatch(r'(\w+) ?\+ ?"(/[^"]+)"', expr)
        if joined and joined[1] in variables:
            return variables[joined[1]] + joined[2]
        call = re.fullmatch(r"topic\((.*)\)", expr)
        if not call:
            return None  # parameter-configured endpoint, no fixed default name
        inner = call[1].replace(" ", "")
        if inner == "id":
            return "~/M"
        literal = re.fullmatch(r'(id\+)?"/?([^"]+)"', inner)
        if not literal:
            return None
        return ("~/M/" if literal[1] else "~/") + literal[2]

    found = set()
    # Assignments and registrations in source order, since local names are reused.
    pattern = (r"\b(?P<var>\w+) ?= ?(?P<value>topic\([^;,()]*\))"
               r"|\b(?P<kind>publisher|subscribe|service)<(?P<cpp>[\w:]+)>\(\s*"
               r"(?P<arg>(?:[^,()]|\([^()]*\))+)")
    for match in re.finditer(pattern, source):
        if match["var"]:
            variables[match["var"]] = resolve(match["value"])
            continue
        name = resolve(match["arg"])
        if name:
            cpp = match["cpp"]
            found.add((name, _SRC_KINDS[match["kind"]], aliases.get(cpp) or _ros_type(cpp)))
    return found


def test_bilingual_quickstart_contract():
    package = Path(__file__).resolve().parents[1]
    documents = [(package / name).read_text()
                 for name in ("README.md", "README.zh-CN.md")]
    provided = source_interfaces((package / "src" / "node.cpp").read_text())
    assert ("~/M/enable", "srv", "std_srvs/SetBool") in provided
    tables = [documented_interfaces(text) for text in documents]
    assert tables[0] == tables[1]
    assert len(set(tables[0])) == len(tables[0])
    assert set(tables[0]) == provided
    blocks = []
    for text in documents:
        for obsolete in ("~/cmd_vel", "~/cmd_vel_stamped",
                         "~/extra_joint_commands", "~/base_pose", "~/driver/feedback/joint_states"):
            assert f"`{obsolete}`" not in text
        assert "```yaml" not in text
        shell = re.findall(r"```bash\n(.*?)```", text, re.S)
        assert shell
        blocks.append(shell)
        for block in shell:
            subprocess.run(["bash", "-n"], input=block, text=True, check=True)
        # Local links and heading fragments must remain navigable in both languages.
        anchors = {re.sub(r"[^\w\s-]", "", heading.lower()).replace(" ", "-")
                   for heading in re.findall(r"^#+ (.+)$", text, re.M)}
        for target in re.findall(r"\[[^\]]+\]\(([^)]+)\)", text):
            link = urlsplit(target)
            if link.scheme or link.netloc:
                continue
            if link.path:
                assert (package / unquote(link.path)).is_file(), target
            elif link.fragment:
                assert unquote(link.fragment) in anchors, target
    assert blocks[0] == blocks[1]
    assert "realtime" not in blocks[0][2]
    config = yaml.safe_load(CONFIG.read_text())
    assert "realtime" not in config and "thread" not in config
    publications = [shlex.split(block.replace("\\\n", ""))
                    for block in blocks[0] if block.startswith("ros2 topic pub")]
    assert len(publications) == 3
    assert [command[5] for command in publications] == [
        "/kstone/tool_pose/control/pose", "/kstone/control/joint",
        "/kstone/chassis/control/cmd_vel"]
    assert [command[6] for command in publications] == [
        "geometry_msgs/msg/PoseStamped", "sensor_msgs/msg/JointState",
        "geometry_msgs/msg/Twist"]
    pose, joint, _twist = [yaml.safe_load(command[7]) for command in publications]
    assert pose["header"]["frame_id"] == config["modules"]["tool_pose"]["reference"]
    assert joint["name"] == config["modules"]["gripper_joint"]["joints"]
    assert joint["position"] == [0.03]


def test_bilingual_documented_omni_config_runs_with_ports(start_node, probe, tmp_path):
    package = Path(__file__).resolve().parents[1]
    for name in ("README.md", "README.zh-CN.md"):
        text = (package / name).read_text()
        for endpoint in ("~/control/joint", "~/M/control/pose",
                         "~/M/control/cmd_vel", "~/M/feedback/pose",
                         "/kstone/control/joint", "/kstone/chassis/control/cmd_vel"):
            assert endpoint in text
        assert "```yaml" not in text  # Docs deliberately reference the YAML config, not a duplicate mapping.
    path = config_variant(tmp_path)
    config = yaml.safe_load(path.read_text())
    config["model"]["file"] = "${urdf}"
    path.write_text(yaml.safe_dump(config))
    bridge = start_node(config_file=str(path),
                          config_variables=json.dumps({"urdf": str(CONFIG.with_suffix(".urdf"))}))
    EchoDriver(probe, bridge, JOINT_NAMES)
    discovery = probe.subscribe_json(bridge.topic("discover"), LATCHED)
    states = probe.subscribe_json(bridge.topic("state"))
    values = subscribe(probe, bridge, JointState, "feedback/joint_states")
    target = probe.node.create_publisher(JointState,
                                        bridge.topic("control/joint"), 10)
    probe.wait(bridge, lambda: discovery and states and target.get_subscription_count())
    modules = {m["id"]: m for m in discovery[-1]["modules"]}
    assert set(modules) == {"chassis", "tool_pose", "gripper_joint"}
    assert_no_legacy_driver_topics(bridge, discovery[-1], probe)
    assert {p["name"] for p in modules["chassis"]["ports"]} == {"velocity", "pose_target"}
    for ident in ["tool_pose", "gripper_joint"]:
        assert {p["name"] for p in modules[ident]["ports"]} == {"target"}
    target.publish(JointState(name=["finger_joint"], position=[0.03]))
    probe.wait(bridge, lambda: values and "finger_joint" in values[-1].name
               and states[-1]["modules"]["gripper_joint"]["ports"]["target"] == pytest.approx([0.03]))
    assert not states[-1]["fault"]
