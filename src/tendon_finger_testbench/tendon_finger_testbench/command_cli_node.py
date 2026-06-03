"""Simple terminal command interface for the testbench."""

from __future__ import annotations

import json
import math
import shlex
import sys
import threading
from typing import Any, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from tendon_finger_testbench.defaults import CLI_DEFAULTS
from tendon_finger_testbench.ros_helpers import declare_parameters, read_parameters


HELP_TEXT = """
Commands:
  pos <rad>          command finger joint target in radians
  posdeg <deg>      command finger joint target in degrees
  motor <rad>       command equivalent motor angle target in radians
  motordeg <deg>    command equivalent motor angle target in degrees
  tip <mm>          command fingertip displacement target in millimeters
  vel <rad/s>       command velocity feedforward/setpoint
  torque <N*m>      publish direct motor torque command
  load <N*m>        set simulated external load torque
  reset             reset the simulated plant
  help              print this help
  quit              stop the CLI
""".strip()


class CommandCliNode(Node):
    def __init__(self) -> None:
        super().__init__("command_cli_node")
        declare_parameters(self, CLI_DEFAULTS)
        self.params = read_parameters(self, CLI_DEFAULTS)

        self.position_pub = self.create_publisher(Float64, "/finger/command_position", 10)
        self.velocity_pub = self.create_publisher(Float64, "/finger/command_velocity", 10)
        self.torque_pub = self.create_publisher(Float64, "/finger/command_torque", 10)
        self.test_pub = self.create_publisher(String, "/finger/test_command", 10)

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("Command CLI ready. Type 'help' for commands.")

    def _read_loop(self) -> None:
        print(HELP_TEXT, flush=True)
        while rclpy.ok():
            try:
                line = input("finger> ")
            except EOFError:
                break
            except KeyboardInterrupt:
                break
            self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        try:
            parts = shlex.split(line)
        except ValueError as exc:
            print(f"Parse error: {exc}")
            return
        if not parts:
            return

        command = parts[0].lower()
        if command in ("quit", "exit"):
            rclpy.shutdown()
            return
        if command == "help":
            print(HELP_TEXT)
            return
        if command == "reset":
            self._publish_test_command({"reset": True})
            return

        try:
            value = self._parse_value(parts)
        except ValueError as exc:
            print(exc)
            return

        if command == "pos":
            self._publish_float(self.position_pub, value)
        elif command == "posdeg":
            self._publish_float(self.position_pub, math.radians(value))
        elif command == "motor":
            self._publish_float(self.position_pub, self._motor_to_finger_angle(value))
        elif command == "motordeg":
            self._publish_float(self.position_pub, self._motor_to_finger_angle(math.radians(value)))
        elif command == "tip":
            self._publish_float(self.position_pub, (value * 0.001) / self._finger_length())
        elif command == "vel":
            self._publish_float(self.velocity_pub, value)
        elif command == "torque":
            self._publish_float(self.torque_pub, value)
            self.get_logger().warn(
                "Direct torque published. If the controller is running it may overwrite this topic."
            )
        elif command == "load":
            self._publish_test_command({"external_load": value})
        else:
            print(f"Unknown command: {command}. Type 'help'.")

    def _parse_value(self, parts: List[str]) -> float:
        if len(parts) < 2:
            raise ValueError("Command requires a numeric value.")
        return float(parts[1])

    def _finger_length(self) -> float:
        return max(float(self.params["finger.length_m"]), 1.0e-9)

    def _motor_to_finger_angle(self, motor_angle: float) -> float:
        gear = max(float(self.params["transmission.gear_ratio"]), 1.0e-9)
        spool_radius = max(float(self.params["transmission.spool_radius_m"]), 1.0e-9)
        moment_arm = max(float(self.params["transmission.tendon_moment_arm_m"]), 1.0e-9)
        return motor_angle / gear * spool_radius / moment_arm

    def _publish_float(self, publisher: Any, value: float) -> None:
        msg = Float64()
        msg.data = value
        publisher.publish(msg)

    def _publish_test_command(self, payload: Any) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self.test_pub.publish(msg)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = CommandCliNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)

