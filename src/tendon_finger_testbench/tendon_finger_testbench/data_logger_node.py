"""CSV logger for tendon finger testbench topics."""

from __future__ import annotations

import csv
import os
from typing import Any, Dict

from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from tendon_finger_testbench.defaults import LOGGER_DEFAULTS
from tendon_finger_testbench.ros_helpers import (
    declare_parameters,
    joint_value,
    parameters_to_dict,
    read_parameters,
)


CSV_COLUMNS = [
    "time",
    "command_position",
    "command_velocity",
    "command_torque_or_current",
    "true_motor_position",
    "true_motor_velocity",
    "measured_motor_encoder_position",
    "true_finger_position",
    "true_finger_velocity",
    "measured_output_position",
    "cable_tension",
    "controller_error_position",
    "controller_error_velocity",
    "torque_command",
    "current_command",
    "applied_torque_after_saturation",
    "external_load",
    "temperature",
    "safety_status",
]


class DataLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("data_logger_node")
        declare_parameters(self, LOGGER_DEFAULTS)
        self.params = read_parameters(self, LOGGER_DEFAULTS)
        self.values: Dict[str, Any] = {column: "" for column in CSV_COLUMNS}
        self.rows_since_flush = 0

        self.file_path = self._make_file_path()
        self.file_handle = open(self.file_path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file_handle, fieldnames=CSV_COLUMNS)
        self.writer.writeheader()

        self.create_subscription(Float64, "/finger/command_position", self._command_position, 10)
        self.create_subscription(Float64, "/finger/command_velocity", self._command_velocity, 10)
        self.create_subscription(Float64, "/finger/command_torque", self._command_torque, 10)
        self.create_subscription(JointState, "/finger/true_state", self._true_state, 10)
        self.create_subscription(JointState, "/finger/sensor_state", self._sensor_state, 10)
        self.create_subscription(JointState, "/finger/controller_state", self._controller_state, 10)
        self.create_subscription(DiagnosticArray, "/finger/safety_status", self._safety_status, 10)
        self.add_on_set_parameters_callback(self._on_parameters)

        self.get_logger().info(f"Logging CSV data to {self.file_path}")

    def _make_file_path(self) -> str:
        directory = os.path.abspath(str(self.params["output_directory"]))
        os.makedirs(directory, exist_ok=True)
        prefix = str(self.params["file_prefix"])
        stamp = self.get_clock().now().nanoseconds
        return os.path.join(directory, f"{prefix}_{stamp}.csv")

    def _on_parameters(self, parameters: Any) -> SetParametersResult:
        self.params.update(parameters_to_dict(parameters))
        return SetParametersResult(successful=True)

    def _command_position(self, msg: Float64) -> None:
        self.values["command_position"] = msg.data

    def _command_velocity(self, msg: Float64) -> None:
        self.values["command_velocity"] = msg.data

    def _command_torque(self, msg: Float64) -> None:
        self.values["command_torque_or_current"] = msg.data

    def _true_state(self, msg: JointState) -> None:
        self.values["true_motor_position"] = joint_value(msg, "motor_shaft", "position", 0.0)
        self.values["true_motor_velocity"] = joint_value(msg, "motor_shaft", "velocity", 0.0)
        self.values["true_finger_position"] = joint_value(msg, "finger_joint", "position", 0.0)
        self.values["true_finger_velocity"] = joint_value(msg, "finger_joint", "velocity", 0.0)
        self.values["applied_torque_after_saturation"] = joint_value(
            msg, "motor_shaft", "effort", 0.0
        )

    def _sensor_state(self, msg: JointState) -> None:
        self.values["measured_motor_encoder_position"] = joint_value(
            msg, "motor_encoder", "position", 0.0
        )
        self.values["measured_output_position"] = joint_value(
            msg, "output_sensor", "position", 0.0
        )
        self.values["cable_tension"] = joint_value(msg, "cable_tension", "position", 0.0)
        self.values["temperature"] = joint_value(msg, "temperature", "position", 0.0)
        self.values["external_load"] = joint_value(msg, "external_load", "position", 0.0)
        self._write_row()

    def _controller_state(self, msg: JointState) -> None:
        self.values["controller_error_position"] = joint_value(
            msg, "position_error", "position", 0.0
        )
        self.values["controller_error_velocity"] = joint_value(
            msg, "velocity_error", "position", 0.0
        )
        self.values["torque_command"] = joint_value(msg, "torque_command", "position", 0.0)
        self.values["current_command"] = joint_value(msg, "current_command", "position", 0.0)

    def _safety_status(self, msg: DiagnosticArray) -> None:
        if not msg.status:
            self.values["safety_status"] = ""
            return
        status = msg.status[0]
        self.values["safety_status"] = status.message

    def _write_row(self) -> None:
        self.values["time"] = self.get_clock().now().nanoseconds * 1.0e-9
        self.writer.writerow(self.values)
        self.rows_since_flush += 1
        if self.rows_since_flush >= int(self.params["flush_every_n_rows"]):
            self.file_handle.flush()
            self.rows_since_flush = 0

    def destroy_node(self) -> bool:
        try:
            self.file_handle.flush()
            self.file_handle.close()
        finally:
            return super().destroy_node()


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

