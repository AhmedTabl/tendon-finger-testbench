"""Cascaded position/velocity/torque-current controller node."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from tendon_finger_testbench.defaults import CONTROLLER_DEFAULTS
from tendon_finger_testbench.model import PID, clamp
from tendon_finger_testbench.ros_helpers import (
    declare_parameters,
    joint_value,
    parameters_to_dict,
    read_parameters,
)


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("controller_node")
        declare_parameters(self, CONTROLLER_DEFAULTS)
        self.params = read_parameters(self, CONTROLLER_DEFAULTS)

        self.position_pid = PID()
        self.velocity_pid = PID()
        self._configure_pids()

        self.target_position = float(self.params["target_position_rad"])
        self.target_velocity = float(self.params["target_velocity_rad_s"])
        self.feedback_position = 0.0
        self.feedback_velocity = 0.0
        self.motor_encoder_position = 0.0
        self.motor_encoder_velocity = 0.0
        self.output_sensor_position = float("nan")
        self.output_sensor_velocity = float("nan")
        self.last_velocity_command = 0.0
        self.position_error = 0.0
        self.velocity_error = 0.0
        self.torque_command = 0.0
        self.current_command = 0.0
        self.last_sensor_time: Optional[Any] = None

        self.torque_pub = self.create_publisher(Float64, "/finger/command_torque", 10)
        self.controller_state_pub = self.create_publisher(
            JointState, "/finger/controller_state", 10
        )

        self.create_subscription(Float64, "/finger/command_position", self._position_callback, 10)
        self.create_subscription(Float64, "/finger/command_velocity", self._velocity_callback, 10)
        self.create_subscription(JointState, "/finger/sensor_state", self._sensor_callback, 10)

        self.add_on_set_parameters_callback(self._on_parameters)

        self.update_rate_hz = max(float(self.params["update_rate_hz"]), 1.0)
        self.dt = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(self.dt, self._timer_callback)
        self.get_logger().info(f"Cascaded controller running at {self.update_rate_hz:.1f} Hz.")

    def _configure_pids(self) -> None:
        self.position_pid.configure(
            float(self.params["position.kp"]),
            float(self.params["position.ki"]),
            float(self.params["position.kd"]),
            float(self.params["position.integral_limit"]),
            float(self.params["position.velocity_limit_rad_s"]),
        )
        torque_limit = self._effective_torque_limit()
        self.velocity_pid.configure(
            float(self.params["velocity.kp"]),
            float(self.params["velocity.ki"]),
            float(self.params["velocity.kd"]),
            float(self.params["velocity.integral_limit"]),
            torque_limit,
        )

    def _effective_torque_limit(self) -> float:
        kt = max(float(self.params["motor.torque_constant_nm_per_a"]), 1.0e-9)
        return min(
            abs(float(self.params["velocity.torque_limit_nm"])),
            abs(float(self.params["safety.max_torque_nm"])),
            kt * abs(float(self.params["safety.max_current_a"])),
        )

    def _on_parameters(self, parameters: Any) -> SetParametersResult:
        updates = parameters_to_dict(parameters)
        self.params.update(updates)
        self._configure_pids()
        if "target_position_rad" in updates:
            self.target_position = float(updates["target_position_rad"])
        if "target_velocity_rad_s" in updates:
            self.target_velocity = float(updates["target_velocity_rad_s"])
        if "update_rate_hz" in updates:
            self.get_logger().warn("update_rate_hz changed; restart node to rebuild the timer.")
        return SetParametersResult(successful=True)

    def _position_callback(self, msg: Float64) -> None:
        self.target_position = msg.data

    def _velocity_callback(self, msg: Float64) -> None:
        self.target_velocity = msg.data

    def _sensor_callback(self, msg: JointState) -> None:
        self.motor_encoder_position = joint_value(msg, "motor_encoder", "position", 0.0)
        self.motor_encoder_velocity = joint_value(msg, "motor_encoder", "velocity", 0.0)
        self.output_sensor_position = joint_value(msg, "output_sensor", "position", float("nan"))
        self.output_sensor_velocity = joint_value(msg, "output_sensor", "velocity", float("nan"))

        source = str(self.params.get("feedback_source", "output_sensor"))
        if source == "motor_encoder" or not math.isfinite(self.output_sensor_position):
            self.feedback_position = self._motor_to_finger_angle(self.motor_encoder_position)
            self.feedback_velocity = self._motor_to_finger_velocity(self.motor_encoder_velocity)
        else:
            self.feedback_position = self.output_sensor_position
            self.feedback_velocity = (
                self.output_sensor_velocity
                if math.isfinite(self.output_sensor_velocity)
                else self.feedback_velocity
            )

    def _motor_to_finger_angle(self, motor_angle: float) -> float:
        gear = max(float(self.params["transmission.gear_ratio"]), 1.0e-9)
        spool_radius = max(float(self.params["transmission.spool_radius_m"]), 1.0e-9)
        moment_arm = max(float(self.params["transmission.tendon_moment_arm_m"]), 1.0e-9)
        return motor_angle / gear * spool_radius / moment_arm

    def _motor_to_finger_velocity(self, motor_velocity: float) -> float:
        gear = max(float(self.params["transmission.gear_ratio"]), 1.0e-9)
        spool_radius = max(float(self.params["transmission.spool_radius_m"]), 1.0e-9)
        moment_arm = max(float(self.params["transmission.tendon_moment_arm_m"]), 1.0e-9)
        return motor_velocity / gear * spool_radius / moment_arm

    def _finger_torque_to_motor_torque(self, finger_torque: float) -> float:
        gear = max(float(self.params["transmission.gear_ratio"]), 1.0e-9)
        spool_radius = max(float(self.params["transmission.spool_radius_m"]), 1.0e-9)
        moment_arm = max(float(self.params["transmission.tendon_moment_arm_m"]), 1.0e-9)
        return finger_torque * spool_radius / (gear * moment_arm)

    def _timer_callback(self) -> None:
        dt = self.dt

        self.position_error = self.target_position - self.feedback_position
        velocity_from_position = self.position_pid.update(self.position_error, dt)
        velocity_limit = abs(float(self.params["position.velocity_limit_rad_s"]))
        velocity_command = clamp(
            velocity_from_position + self.target_velocity,
            -velocity_limit,
            velocity_limit,
        )

        self.velocity_error = velocity_command - self.feedback_velocity
        torque_command = self.velocity_pid.update(self.velocity_error, dt)

        acceleration_command = (velocity_command - self.last_velocity_command) / dt
        self.last_velocity_command = velocity_command

        torque_command += float(self.params["feedforward.velocity_gain_nm_per_rad_s"]) * velocity_command
        torque_command += (
            float(self.params["feedforward.acceleration_gain_nm_per_rad_s2"])
            * acceleration_command
        )

        if bool(self.params["feedforward.gravity_compensation"]):
            finger_gravity = (
                float(self.params["model.finger_mass_kg"])
                * 9.80665
                * float(self.params["model.finger_com_length_m"])
                * math.sin(self.feedback_position)
            )
            torque_command += self._finger_torque_to_motor_torque(finger_gravity)

        if bool(self.params["feedforward.friction_compensation"]):
            friction = float(self.params["model.finger_coulomb_friction_nm"])
            if abs(velocity_command) > 1.0e-4:
                torque_command += self._finger_torque_to_motor_torque(
                    friction * math.copysign(1.0, velocity_command)
                )

        torque_limit = self._effective_torque_limit()
        torque_command = clamp(torque_command, -torque_limit, torque_limit)
        kt = max(float(self.params["motor.torque_constant_nm_per_a"]), 1.0e-9)
        self.torque_command = torque_command
        self.current_command = torque_command / kt

        torque_msg = Float64()
        torque_msg.data = self.torque_command
        self.torque_pub.publish(torque_msg)
        self.controller_state_pub.publish(self._make_controller_state(velocity_command))

    def _make_controller_state(self, velocity_command: float) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "target_position",
            "target_velocity",
            "feedback_position",
            "feedback_velocity",
            "position_error",
            "velocity_error",
            "torque_command",
            "current_command",
        ]
        msg.position = [
            self.target_position,
            self.target_velocity,
            self.feedback_position,
            self.feedback_velocity,
            self.position_error,
            self.velocity_error,
            self.torque_command,
            self.current_command,
        ]
        msg.velocity = [
            velocity_command,
            self.target_velocity,
            self.feedback_velocity,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        msg.effort = [
            self.torque_command,
            self.current_command,
            self._effective_torque_limit(),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        return msg


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

