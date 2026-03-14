#!/usr/bin/env python3
"""State machine node — gates cmd_vel based on mission and AS state.

Subscribes to dashboard commands and muxes autonomous/manual cmd_vel
to /kart/cmd_vel_muxed, which cmd_vel_bridge reads.

States follow Formula Student AS (Autonomous System) conventions:
  AS_OFF(0) → AS_READY(1) → AS_DRIVING(2) → AS_FINISHED(3)
  Any state (except AS_OFF) → AS_EMERGENCY(4)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from kb_interfaces.msg import Frame

# AS states
AS_OFF = 0
AS_READY = 1
AS_DRIVING = 2
AS_FINISHED = 3
AS_EMERGENCY = 4

STATE_NAMES = {
    AS_OFF: "AS_OFF",
    AS_READY: "AS_READY",
    AS_DRIVING: "AS_DRIVING",
    AS_FINISHED: "AS_FINISHED",
    AS_EMERGENCY: "AS_EMERGENCY",
}

# Missions that count as "autonomous"
AUTONOMOUS_MISSIONS = {"acceleration", "skidpad", "autocross", "trackdrive", "ebs_test", "inspection"}


class StateMachineNode(Node):
    """@brief State machine node that gates cmd_vel based on mission and AS state.

    Implements Formula Student AS state transitions and muxes autonomous/manual
    cmd_vel to /kart/cmd_vel_muxed at 100 Hz. Publishes state heartbeat at 10 Hz.
    """

    def __init__(self):
        """@brief Initialize the state machine in AS_OFF with subscriptions, publishers, and timers."""
        super().__init__("state_machine")

        self._state = AS_OFF
        self._mission = "manual"
        self._last_auto_cmd = Twist()
        self._last_manual_cmd = Twist()

        # Subscriptions
        self.create_subscription(String, "/dashboard/mission", self._on_mission, 10)
        self.create_subscription(String, "/dashboard/state_cmd", self._on_state_cmd, 10)
        self.create_subscription(Twist, "/kart/cmd_vel", self._on_auto_cmd, 10)
        self.create_subscription(Twist, "/kart/cmd_vel_manual", self._on_manual_cmd, 10)

        # Publishers
        self._muxed_pub = self.create_publisher(Twist, "/kart/cmd_vel_muxed", 10)
        self._state_pub = self.create_publisher(String, "/kart/state", 10)
        self._machine_state_pub = self.create_publisher(Frame, "/orin/machine_state", 10)
        self._mission_pub = self.create_publisher(Frame, "/orin/mission", 10)

        # 100 Hz mux timer
        self.create_timer(0.01, self._mux_tick)
        # 10 Hz state heartbeat
        self.create_timer(0.1, self._publish_state)

        self.get_logger().info("StateMachine: started in AS_OFF")

    # ── Subscriptions ──────────────────────────────────────────────────

    def _on_mission(self, msg: String):
        """@brief Callback for mission selection from the dashboard. Triggers state transitions.

        @param msg String message with the mission name.
        """
        old = self._mission
        self._mission = msg.data
        if old == self._mission:
            return
        self.get_logger().info(f"Mission: {old} → {self._mission}")
        self._publish_mission_frame()

        # Auto-transition: selecting autonomous mission → AS_READY
        if self._mission in AUTONOMOUS_MISSIONS and self._state == AS_OFF:
            self._set_state(AS_READY)
        # Selecting manual/remote_control while in AS_READY → back to AS_OFF
        elif self._mission not in AUTONOMOUS_MISSIONS and self._state == AS_READY:
            self._set_state(AS_OFF)

    def _on_state_cmd(self, msg: String):
        """@brief Callback for state commands (start, stop, ebs, finish, reset).

        @param msg String message with the command.
        """
        cmd = msg.data
        s = self._state

        if cmd == "start" and s == AS_READY:
            self._set_state(AS_DRIVING)
        elif cmd == "stop" and s in (AS_READY, AS_DRIVING):
            self._set_state(AS_OFF)
        elif cmd == "ebs" and s != AS_OFF:
            self._set_state(AS_EMERGENCY)
        elif cmd == "finish" and s == AS_DRIVING:
            self._set_state(AS_FINISHED)
        elif cmd == "reset" and s in (AS_FINISHED, AS_EMERGENCY):
            self._set_state(AS_OFF)
        else:
            self.get_logger().warn(f"Ignored cmd '{cmd}' in state {STATE_NAMES[s]}")

    def _on_auto_cmd(self, msg: Twist):
        """@brief Callback for autonomous cmd_vel. Stores latest command for muxing."""
        self._last_auto_cmd = msg

    def _on_manual_cmd(self, msg: Twist):
        """@brief Callback for manual (remote control) cmd_vel. Stores latest command for muxing."""
        self._last_manual_cmd = msg

    # ── State transitions ──────────────────────────────────────────────

    def _set_state(self, new_state: int):
        """@brief Transition to a new AS state, logging and publishing the change.

        @param new_state Target AS state constant (AS_OFF, AS_READY, etc.).
        """
        old = self._state
        self._state = new_state
        self.get_logger().info(f"State: {STATE_NAMES[old]} → {STATE_NAMES[new_state]}")
        self._publish_state()
        self._publish_state_frame()

    # ── Muxing (100 Hz) ───────────────────────────────────────────────

    def _mux_tick(self):
        """@brief Timer callback (100 Hz): mux autonomous or manual cmd_vel based on mission and state."""
        out = Twist()

        if self._mission == "remote_control":
            out = self._last_manual_cmd
        elif self._mission in AUTONOMOUS_MISSIONS:
            if self._state == AS_DRIVING:
                out = self._last_auto_cmd
            # else: zero Twist (default)

        self._muxed_pub.publish(out)

    # ── Publishers ─────────────────────────────────────────────────────

    def _publish_state(self):
        """@brief Publish current AS state name to /kart/state as a String."""
        msg = String()
        msg.data = STATE_NAMES[self._state]
        self._state_pub.publish(msg)

    def _publish_state_frame(self):
        """@brief Publish current AS state as a Frame to /orin/machine_state for the ESP32."""
        frame = Frame()
        frame.type = Frame.ORIN_MACHINE_STATE
        frame.payload = [self._state]
        self._machine_state_pub.publish(frame)

    def _publish_mission_frame(self):
        """@brief Publish current mission ID as a Frame to /orin/mission for the ESP32."""
        from kb_dashboard.protocol import MISSIONS
        mission_id = MISSIONS.get(self._mission, 0)
        frame = Frame()
        frame.type = Frame.ORIN_MISION
        frame.payload = [mission_id]
        self._mission_pub.publish(frame)


def main():
    """@brief Entrypoint for the state machine node."""
    rclpy.init()
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
