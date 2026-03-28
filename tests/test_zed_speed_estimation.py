"""Regression tests for ZED VIO speed estimation.

Tests the position-differentiation speed computation, QoS compatibility,
and dashboard integration — the three bugs found during initial deployment.

Run locally (no ROS needed):
    python -m pytest tests/test_zed_speed_estimation.py -v
"""

import math
import sys
import types

# ---------------------------------------------------------------------------
# Minimal ROS 2 stubs (same pattern as test_steering_gain.py)
# ---------------------------------------------------------------------------
_rclpy = types.ModuleType("rclpy")
_rclpy.init = lambda *a, **kw: None
_rclpy.spin = lambda *a, **kw: None
_rclpy.shutdown = lambda *a, **kw: None

_node_mod = types.ModuleType("rclpy.node")


class _FakeNode:
    def __init__(self, *a, **kw):
        self._params = {}

    def declare_parameter(self, name, default):
        self._params[name] = default

    def get_parameter(self, name):
        class _Val:
            def __init__(self, v):
                self.value = v
        return _Val(self._params[name])

    def create_publisher(self, *a, **kw):
        return _FakePublisher()

    def create_subscription(self, *a, **kw):
        return None

    def create_timer(self, *a, **kw):
        return None

    def get_logger(self):
        class _L:
            info = staticmethod(lambda *a, **kw: None)
            error = staticmethod(lambda *a, **kw: None)
            warn = staticmethod(lambda *a, **kw: None)
        return _L()

    def get_clock(self):
        class _C:
            def now(self):
                class _T:
                    nanoseconds = 0
                return _T()
        return _C()


class _FakePublisher:
    def __init__(self):
        self.last_msg = None

    def publish(self, msg):
        self.last_msg = msg


_node_mod.Node = _FakeNode
_rclpy.node = _node_mod

_qos_mod = types.ModuleType("rclpy.qos")
_qos_mod.QoSProfile = type("QoSProfile", (), {"__init__": lambda self, **kw: None})
_qos_mod.DurabilityPolicy = type("DurabilityPolicy", (), {"VOLATILE": 0})
_qos_mod.ReliabilityPolicy = type("ReliabilityPolicy", (), {"BEST_EFFORT": 0, "RELIABLE": 1})

_geo_mod = types.ModuleType("geometry_msgs")
_geo_msg = types.ModuleType("geometry_msgs.msg")


class _Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


_geo_msg.Twist = _Twist

_nav_mod = types.ModuleType("nav_msgs")
_nav_msg = types.ModuleType("nav_msgs.msg")
_nav_msg.Odometry = type("Odometry", (), {})

_std_mod = types.ModuleType("std_msgs")
_std_msg = types.ModuleType("std_msgs.msg")


class _Float32:
    def __init__(self, data=0.0):
        self.data = data


class _String:
    def __init__(self, data=""):
        self.data = data


_std_msg.Float32 = _Float32
_std_msg.String = _String

_vis_mod = types.ModuleType("vision_msgs")
_vis_msg = types.ModuleType("vision_msgs.msg")
_vis_msg.Detection3DArray = type("Detection3DArray", (), {})

for _name, _mod in [
    ("rclpy", _rclpy),
    ("rclpy.node", _node_mod),
    ("rclpy.qos", _qos_mod),
    ("geometry_msgs", _geo_mod),
    ("geometry_msgs.msg", _geo_msg),
    ("nav_msgs", _nav_mod),
    ("nav_msgs.msg", _nav_msg),
    ("std_msgs", _std_mod),
    ("std_msgs.msg", _std_msg),
    ("vision_msgs", _vis_mod),
    ("vision_msgs.msg", _vis_msg),
]:
    if _mod is not None:
        sys.modules[_name] = _mod

sys.path.insert(0, "src/kart_control/scripts")
from cone_follower_node import ConeFollowerNode  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_node() -> ConeFollowerNode:
    """Instantiate ConeFollowerNode with default params for speed testing."""
    node = ConeFollowerNode.__new__(ConeFollowerNode)
    _FakeNode.__init__(node)
    node.declare_parameter("detections_topic", "/perception/cones_3d")
    node.declare_parameter("cmd_vel_topic", "/kart/cmd_vel")
    node.declare_parameter("odom_topic", "/zed/zed_node/odom")
    node.declare_parameter("no_cone_timeout", 1.0)
    node.declare_parameter("controller_type", "geometric")
    node.declare_parameter("weights_json", "")
    node.declare_parameter("steering_gain", 3.0)
    node.declare_parameter("max_steer", 1.047)
    node.declare_parameter("max_speed", 2.625)
    node.declare_parameter("min_speed", 0.5)
    node.declare_parameter("lookahead_max", 15.0)
    node.declare_parameter("half_track_width", 1.5)
    node.declare_parameter("speed_curve_factor", 0.0)
    node.steering_gain = 3.0
    node.max_steer = 1.047
    node.max_speed = 2.625
    node.min_speed = 0.5
    node.lookahead_max = 15.0
    node.half_track_width = 1.5
    node.speed_curve_factor = 0.0
    node.controller_type = "geometric"
    node._last_steer = 0.0
    node._actual_speed = 0.0
    node._prev_odom_pos = None
    node._prev_odom_time = None
    node._nn_W1 = None
    node.speed_pub = _FakePublisher()
    return node


def _make_odom(x, y, z, sec, nanosec=0):
    """Build a fake Odometry message with position and timestamp."""
    msg = types.SimpleNamespace()
    msg.header = types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=sec, nanosec=nanosec)
    )
    msg.pose = types.SimpleNamespace(
        pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=x, y=y, z=z)
        )
    )
    msg.twist = types.SimpleNamespace(
        twist=types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
        )
    )
    return msg


# ---------------------------------------------------------------------------
# Tests: position differentiation speed computation
# ---------------------------------------------------------------------------

class TestSpeedFromPositionDiff:
    """Regression: ZED odom twist is always zero; speed must come from dp/dt."""

    def test_first_message_speed_is_zero(self):
        """First odom message cannot compute speed (no previous position)."""
        node = _make_node()
        node._on_odom(_make_odom(1.0, 2.0, 0.0, sec=100))
        assert node._actual_speed == 0.0

    def test_stationary_speed_is_zero(self):
        """Same position at different times -> speed = 0."""
        node = _make_node()
        node._on_odom(_make_odom(1.0, 0.0, 0.0, sec=100))
        node._on_odom(_make_odom(1.0, 0.0, 0.0, sec=101))
        assert node._actual_speed == 0.0

    def test_constant_velocity_x(self):
        """Moving 2m in X over 1s -> speed = 2 m/s."""
        node = _make_node()
        node._on_odom(_make_odom(0.0, 0.0, 0.0, sec=100))
        node._on_odom(_make_odom(2.0, 0.0, 0.0, sec=101))
        assert abs(node._actual_speed - 2.0) < 1e-9

    def test_constant_velocity_y(self):
        """Moving 3m in Y over 1s -> speed = 3 m/s."""
        node = _make_node()
        node._on_odom(_make_odom(0.0, 0.0, 0.0, sec=100))
        node._on_odom(_make_odom(0.0, 3.0, 0.0, sec=101))
        assert abs(node._actual_speed - 3.0) < 1e-9

    def test_diagonal_velocity(self):
        """Moving (3, 4, 0) over 1s -> speed = 5 m/s."""
        node = _make_node()
        node._on_odom(_make_odom(0.0, 0.0, 0.0, sec=100))
        node._on_odom(_make_odom(3.0, 4.0, 0.0, sec=101))
        assert abs(node._actual_speed - 5.0) < 1e-9

    def test_3d_velocity(self):
        """3D displacement: speed = sqrt(1+4+4)/1 = 3 m/s."""
        node = _make_node()
        node._on_odom(_make_odom(0.0, 0.0, 0.0, sec=100))
        node._on_odom(_make_odom(1.0, 2.0, 2.0, sec=101))
        assert abs(node._actual_speed - 3.0) < 1e-9

    def test_high_frequency_updates(self):
        """At 100 Hz (dt=0.01s), 0.01m displacement -> 1 m/s."""
        node = _make_node()
        node._on_odom(_make_odom(0.0, 0.0, 0.0, sec=100, nanosec=0))
        node._on_odom(_make_odom(0.01, 0.0, 0.0, sec=100, nanosec=10_000_000))
        assert abs(node._actual_speed - 1.0) < 1e-6

    def test_nanosecond_precision(self):
        """Timestamps with nanoseconds are handled correctly."""
        node = _make_node()
        node._on_odom(_make_odom(0.0, 0.0, 0.0, sec=100, nanosec=500_000_000))
        node._on_odom(_make_odom(1.0, 0.0, 0.0, sec=101, nanosec=500_000_000))
        assert abs(node._actual_speed - 1.0) < 1e-9

    def test_speed_updates_each_message(self):
        """Speed reflects latest displacement, not cumulative."""
        node = _make_node()
        node._on_odom(_make_odom(0.0, 0.0, 0.0, sec=100))
        node._on_odom(_make_odom(5.0, 0.0, 0.0, sec=101))  # 5 m/s
        assert abs(node._actual_speed - 5.0) < 1e-9
        node._on_odom(_make_odom(5.0, 0.0, 0.0, sec=102))  # stopped
        assert abs(node._actual_speed - 0.0) < 1e-9

    def test_zero_dt_does_not_divide_by_zero(self):
        """Two messages with identical timestamps must not crash."""
        node = _make_node()
        node._on_odom(_make_odom(0.0, 0.0, 0.0, sec=100))
        node._on_odom(_make_odom(1.0, 0.0, 0.0, sec=100))
        # Speed should remain 0 (dt guard prevents division)
        assert node._actual_speed == 0.0


class TestSpeedPublishing:
    """Speed is published to /kart/speed as Float32."""

    def test_publishes_on_every_odom(self):
        """Each odom callback publishes a Float32 speed."""
        node = _make_node()
        node._on_odom(_make_odom(0.0, 0.0, 0.0, sec=100))
        assert node.speed_pub.last_msg is not None
        assert node.speed_pub.last_msg.data == 0.0

    def test_publishes_correct_value(self):
        """Published speed matches computed speed."""
        node = _make_node()
        node._on_odom(_make_odom(0.0, 0.0, 0.0, sec=100))
        node._on_odom(_make_odom(2.0, 0.0, 0.0, sec=101))
        assert abs(node.speed_pub.last_msg.data - 2.0) < 1e-9

    def test_speed_feeds_neural_v2(self):
        """_actual_speed is used by neural_v2 controller for speed feedback."""
        node = _make_node()
        node._on_odom(_make_odom(0.0, 0.0, 0.0, sec=100))
        node._on_odom(_make_odom(3.0, 4.0, 0.0, sec=101))
        assert abs(node._actual_speed - 5.0) < 1e-9


class TestOdomTopicParam:
    """Regression: odom_topic must be parameterized (ZED vs sim)."""

    def test_default_odom_topic_is_zed(self):
        """Default odom_topic is ZED's /zed/zed_node/odom for real kart."""
        node = _make_node()
        assert node.get_parameter("odom_topic").value == "/zed/zed_node/odom"

    def test_odom_topic_overridable(self):
        """Sim launch overrides to /model/kart/odom_gt."""
        node = _make_node()
        node._params["odom_topic"] = "/model/kart/odom_gt"
        assert node.get_parameter("odom_topic").value == "/model/kart/odom_gt"


class TestQoSCompatibility:
    """Regression: BEST_EFFORT subscriber is invisible to RELIABLE ZED publisher."""

    def test_odom_qos_is_reliable(self):
        """The odom subscription must use RELIABLE QoS to match ZED."""
        # Read the source directly to verify the QoS setting
        import inspect
        source = inspect.getsource(ConeFollowerNode.__init__)
        assert "ReliabilityPolicy.RELIABLE" in source, (
            "Odom subscription must use RELIABLE QoS. "
            "ZED publishes with RELIABLE and count_subscribers() ignores "
            "BEST_EFFORT subscribers, causing odom to never be published."
        )
        assert "ReliabilityPolicy.BEST_EFFORT" not in source, (
            "Odom subscription must NOT use BEST_EFFORT. "
            "See QoS mismatch bug: ZED only publishes when it sees RELIABLE subscribers."
        )


class TestZedConfig:
    """Regression: ZED config must have positional tracking enabled."""

    def test_pos_tracking_enabled_in_config(self):
        """zed_overrides.yaml must have pos_tracking_enabled: true."""
        import yaml
        with open("src/kart_bringup/config/zed_overrides.yaml") as f:
            cfg = yaml.safe_load(f)
        params = cfg["/**"]["ros__parameters"]
        assert params["pos_tracking"]["pos_tracking_enabled"] is True, (
            "pos_tracking_enabled must be true for ZED VIO speed estimation"
        )

    def test_publish_tf_enabled(self):
        """publish_tf must be true for ZED to compute and publish odom."""
        import yaml
        with open("src/kart_bringup/config/zed_overrides.yaml") as f:
            cfg = yaml.safe_load(f)
        params = cfg["/**"]["ros__parameters"]
        assert params["pos_tracking"]["publish_tf"] is True


class TestDashboardSpeedIntegration:
    """Dashboard must subscribe to /kart/speed and update esp32_speed state."""

    def test_dashboard_node_subscribes_to_kart_speed(self):
        """dashboard_node.py must subscribe to /kart/speed."""
        with open("src/kb_dashboard/kb_dashboard/dashboard_node.py") as f:
            src = f.read()
        assert '"/kart/speed"' in src, (
            "Dashboard must subscribe to /kart/speed for ZED VIO speed"
        )

    def test_dashboard_html_has_speed_element(self):
        """index.html must have an uncommented vSpeed element."""
        with open("src/kb_dashboard/kb_dashboard/index.html") as f:
            src = f.read()
        # Check the element exists and is NOT inside a comment
        assert 'id="vSpeed"' in src, "Speed display element missing"
        # Find the line with vSpeed and check it's not commented
        for line in src.split("\n"):
            if 'id="vSpeed"' in line:
                assert not line.strip().startswith("<!--"), (
                    "Speed element is commented out in HTML"
                )
                break

    def test_dashboard_js_updates_speed(self):
        """Default skin update() must write to vSpeed (not commented out)."""
        with open("src/kb_dashboard/kb_dashboard/index.html") as f:
            src = f.read()
        # Find the line that updates vSpeed
        for line in src.split("\n"):
            if "vSpeed" in line and "esp32_speed" in line and ".innerHTML" in line:
                assert not line.strip().startswith("//"), (
                    "Speed JS update is commented out — dashboard won't show speed"
                )
                return
        raise AssertionError("No vSpeed update line found in default skin JS")


class TestSimLaunchOdomOverride:
    """Sim launch must override odom_topic to Gazebo ground truth."""

    def test_sim_launch_has_odom_override(self):
        with open("src/kart_sim/launch/simulation_gz_jazzy.launch.py") as f:
            src = f.read()
        assert '"/model/kart/odom_gt"' in src, (
            "Sim launch must override odom_topic to /model/kart/odom_gt"
        )
        assert '"odom_topic"' in src, (
            "Sim launch must pass odom_topic parameter to cone_follower"
        )
