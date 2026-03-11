"""Unit tests for the dashboard WebSocket server manual control."""

import unittest
from unittest.mock import MagicMock

from kb_dashboard.server import handle_command
from kb_dashboard.protocol import DashboardState


class TestManualControlWS(unittest.TestCase):
    def setUp(self):
        self.state = DashboardState()
        self.node = MagicMock()

    def test_manual_control_action_dispatches_to_node(self):
        """When the frontend sends action: 'manual_control', it should call publish_manual_control."""

        # This is exactly the payload the JS Gamepad loop sends:
        payload = {
            "action": "manual_control",
            "steering": -0.85,
            "steer_type": "pwm",
            "throttle": 1.0,
            "brake": 0.0,
        }

        # Handle the WebSocket JSON command
        handle_command(payload, self.state, self.node)

        # Assert that the node method was called with correctly extracted kwargs
        self.node.publish_manual_control.assert_called_once_with(
            steer=-0.85, steer_type="pwm", throttle=1.0, brake=0.0
        )

    def test_manual_control_with_missing_fields(self):
        """Missing fields should safely default to zero floats and 'angle' string."""
        payload = {
            "action": "manual_control",
            "steering": 0.5,
            # Missing steer_type, throttle, brake
        }

        handle_command(payload, self.state, self.node)

        self.node.publish_manual_control.assert_called_once_with(
            steer=0.5, steer_type="angle", throttle=0.0, brake=0.0
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
