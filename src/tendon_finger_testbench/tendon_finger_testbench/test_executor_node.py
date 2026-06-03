"""Automated command-profile executor for actuator characterization tests."""

from __future__ import annotations

import json
import math
import os
from statistics import mean
from typing import Any, Dict, List, Optional

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from tendon_finger_testbench.defaults import TEST_DEFAULTS
from tendon_finger_testbench.ros_helpers import (
    declare_parameters,
    joint_value,
    parameters_to_dict,
    read_parameters,
)


class TestExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("test_executor_node")
        declare_parameters(self, TEST_DEFAULTS)
        self.params = read_parameters(self, TEST_DEFAULTS)

        self.position_pub = self.create_publisher(Float64, "/finger/command_position", 10)
        self.velocity_pub = self.create_publisher(Float64, "/finger/command_velocity", 10)
        self.test_pub = self.create_publisher(String, "/finger/test_command", 10)

        self.create_subscription(JointState, "/finger/sensor_state", self._sensor_callback, 10)
        self.create_subscription(JointState, "/finger/true_state", self._true_callback, 10)
        self.create_subscription(JointState, "/finger/controller_state", self._controller_callback, 10)
        self.create_subscription(String, "/finger/test_command", self._test_command_callback, 10)

        self.add_on_set_parameters_callback(self._on_parameters)

        self.running = False
        self.started_at: Optional[float] = None
        self.samples: List[Dict[str, float]] = []
        self.latest_sensor: Dict[str, float] = {}
        self.latest_true: Dict[str, float] = {}
        self.latest_controller: Dict[str, float] = {}

        if bool(self.params["auto_start"]):
            self._start_test(str(self.params["mode"]))

        rate = max(float(self.params["publish_rate_hz"]), 1.0)
        self.timer = self.create_timer(1.0 / rate, self._timer_callback)
        self.get_logger().info("Test executor ready. Set auto_start true or publish a start_test command.")

    def _on_parameters(self, parameters: Any) -> SetParametersResult:
        self.params.update(parameters_to_dict(parameters))
        return SetParametersResult(successful=True)

    def _start_test(self, mode: str) -> None:
        self.params["mode"] = mode
        self.running = True
        self.started_at = self._now_s()
        self.samples.clear()
        self._publish_test_command({"reset": True, "external_load": 0.0})
        self.get_logger().info(f"Starting {mode} test.")

    def _stop_test(self) -> None:
        mode = str(self.params["mode"])
        self.running = False
        self._publish_float(self.velocity_pub, 0.0)
        if mode == "load":
            self._publish_test_command({"external_load": 0.0})
        path = self._write_summary(mode)
        self.get_logger().info(f"{mode} test complete. Summary: {path}")

    def _test_command_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if "start_test" in payload:
            self._start_test(str(payload["start_test"]))
        if payload.get("stop_test"):
            self._stop_test()

    def _sensor_callback(self, msg: JointState) -> None:
        self.latest_sensor = {
            "measured_motor_encoder_position": joint_value(msg, "motor_encoder", "position", 0.0),
            "measured_motor_encoder_velocity": joint_value(msg, "motor_encoder", "velocity", 0.0),
            "measured_output_position": joint_value(msg, "output_sensor", "position", 0.0),
            "measured_output_velocity": joint_value(msg, "output_sensor", "velocity", 0.0),
            "cable_tension": joint_value(msg, "cable_tension", "position", 0.0),
            "temperature": joint_value(msg, "temperature", "position", 0.0),
            "external_load": joint_value(msg, "external_load", "position", 0.0),
        }

    def _true_callback(self, msg: JointState) -> None:
        self.latest_true = {
            "true_motor_position": joint_value(msg, "motor_shaft", "position", 0.0),
            "true_motor_velocity": joint_value(msg, "motor_shaft", "velocity", 0.0),
            "true_finger_position": joint_value(msg, "finger_joint", "position", 0.0),
            "true_finger_velocity": joint_value(msg, "finger_joint", "velocity", 0.0),
            "applied_torque_after_saturation": joint_value(msg, "motor_shaft", "effort", 0.0),
        }

    def _controller_callback(self, msg: JointState) -> None:
        self.latest_controller = {
            "command_position": joint_value(msg, "target_position", "position", 0.0),
            "command_velocity": joint_value(msg, "target_velocity", "position", 0.0),
            "controller_error_position": joint_value(msg, "position_error", "position", 0.0),
            "controller_error_velocity": joint_value(msg, "velocity_error", "position", 0.0),
            "torque_command": joint_value(msg, "torque_command", "position", 0.0),
            "current_command": joint_value(msg, "current_command", "position", 0.0),
        }

    def _timer_callback(self) -> None:
        if not self.running or self.started_at is None:
            return
        elapsed = self._now_s() - self.started_at
        mode = str(self.params["mode"]).lower()
        duration = self._mode_duration(mode)

        command_position = self._profile(mode, elapsed)
        self._publish_float(self.position_pub, command_position)
        self._record_sample(elapsed, command_position)

        if elapsed >= duration:
            self._stop_test()

    def _mode_duration(self, mode: str) -> float:
        delay = max(float(self.params["start_delay_s"]), 0.0)
        if mode == "step":
            return delay + max(float(self.params["step.hold_s"]), 0.1)
        if mode == "backlash":
            return delay + max(float(self.params["backlash.cycles"]), 1.0) * max(
                float(self.params["backlash.period_s"]), 0.1
            )
        return max(float(self.params["duration_s"]), 0.1)

    def _profile(self, mode: str, elapsed: float) -> float:
        delay = max(float(self.params["start_delay_s"]), 0.0)
        active_t = max(0.0, elapsed - delay)
        if elapsed < delay:
            return 0.0

        if mode == "step":
            return float(self.params["step.target_rad"])

        if mode == "backlash":
            amplitude = float(self.params["backlash.amplitude_rad"])
            period = max(float(self.params["backlash.period_s"]), 0.1)
            phase = (active_t / period) % 1.0
            if phase < 0.25:
                return 4.0 * amplitude * phase
            if phase < 0.75:
                return amplitude - 4.0 * amplitude * (phase - 0.25)
            return -amplitude + 4.0 * amplitude * (phase - 0.75)

        if mode == "friction":
            amplitude = float(self.params["friction.amplitude_rad"])
            period = max(float(self.params["friction.period_s"]), 0.1)
            return amplitude * math.sin(2.0 * math.pi * active_t / period)

        if mode == "sine":
            return self._sine(active_t)

        if mode == "chirp":
            duration = max(float(self.params["duration_s"]) - delay, 0.1)
            f0 = float(self.params["chirp.start_frequency_hz"])
            f1 = float(self.params["chirp.end_frequency_hz"])
            k = (f1 - f0) / duration
            phase = 2.0 * math.pi * (f0 * active_t + 0.5 * k * active_t * active_t)
            amplitude = float(self.params["sine.amplitude_rad"])
            bias = float(self.params["sine.bias_rad"])
            return bias + amplitude * math.sin(phase)

        if mode == "load":
            if abs(active_t) < 1.0 / max(float(self.params["publish_rate_hz"]), 1.0):
                self._publish_test_command(
                    {"external_load": float(self.params["load.external_torque_nm"])}
                )
            return float(self.params["load.target_rad"])

        if mode == "thermal":
            amplitude = float(self.params["thermal.amplitude_rad"])
            bias = float(self.params["thermal.bias_rad"])
            frequency = float(self.params["thermal.frequency_hz"])
            return bias + amplitude * math.sin(2.0 * math.pi * frequency * active_t)

        return self._sine(active_t)

    def _sine(self, active_t: float) -> float:
        amplitude = float(self.params["sine.amplitude_rad"])
        bias = float(self.params["sine.bias_rad"])
        frequency = float(self.params["sine.frequency_hz"])
        return bias + amplitude * math.sin(2.0 * math.pi * frequency * active_t)

    def _record_sample(self, elapsed: float, command_position: float) -> None:
        sample: Dict[str, float] = {"time": elapsed, "command_position": command_position}
        sample.update(self.latest_sensor)
        sample.update(self.latest_true)
        sample.update(self.latest_controller)
        self.samples.append(sample)

    def _write_summary(self, mode: str) -> str:
        directory = os.path.abspath(str(self.params["output_directory"]))
        os.makedirs(directory, exist_ok=True)
        path = os.path.join(directory, f"{mode}_summary_{self.get_clock().now().nanoseconds}.md")
        lines = [
            f"# {mode.title()} Test Summary",
            "",
            f"Samples collected: {len(self.samples)}",
        ]

        if mode == "step":
            lines.extend(self._step_summary_lines())
        elif mode == "backlash":
            lines.extend(self._backlash_summary_lines())
        elif mode == "thermal":
            temperatures = [s.get("temperature", 0.0) for s in self.samples]
            if temperatures:
                lines.append(f"Peak temperature: {max(temperatures):.2f} C")

        with open(path, "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")
        return path

    def _step_summary_lines(self) -> List[str]:
        if not self.samples:
            return ["No samples available."]
        target = float(self.params["step.target_rad"])
        values = [
            (s["time"], s.get("measured_output_position", s.get("true_finger_position", 0.0)))
            for s in self.samples
        ]
        start_value = values[0][1]
        final_values = [position for _, position in values[-max(1, len(values) // 10) :]]
        final_value = mean(final_values)
        amplitude = target - start_value
        if abs(amplitude) < 1.0e-9:
            return ["Step amplitude too small for metrics."]

        t10 = self._crossing_time(values, start_value + 0.1 * amplitude)
        t90 = self._crossing_time(values, start_value + 0.9 * amplitude)
        rise = float("nan") if t10 is None or t90 is None else t90 - t10
        peak = max(position for _, position in values) if amplitude > 0.0 else min(position for _, position in values)
        overshoot = (peak - target) / abs(amplitude) * 100.0 if amplitude > 0.0 else (target - peak) / abs(amplitude) * 100.0
        steady_error = target - final_value
        peak_torque = max(abs(s.get("torque_command", 0.0)) for s in self.samples)
        peak_current = max(abs(s.get("current_command", 0.0)) for s in self.samples)

        return [
            f"Rise time 10-90%: {rise:.3f} s",
            f"Overshoot: {overshoot:.2f} %",
            f"Steady-state error: {steady_error:.4f} rad",
            f"Peak torque command: {peak_torque:.4f} N m",
            f"Peak current command: {peak_current:.3f} A",
        ]

    def _backlash_summary_lines(self) -> List[str]:
        estimates = []
        threshold = 0.01
        last_direction = 0.0
        reversal_motor = None
        reversal_output = None
        for sample in self.samples:
            motor_v = sample.get("measured_motor_encoder_velocity", 0.0)
            direction = math.copysign(1.0, motor_v) if abs(motor_v) > threshold else last_direction
            if last_direction and direction != last_direction:
                reversal_motor = sample.get("measured_motor_encoder_position", 0.0)
                reversal_output = sample.get("measured_output_position", 0.0)
            if reversal_motor is not None and reversal_output is not None:
                output_delta = abs(sample.get("measured_output_position", 0.0) - reversal_output)
                if output_delta > threshold:
                    estimates.append(abs(sample.get("measured_motor_encoder_position", 0.0) - reversal_motor))
                    reversal_motor = None
                    reversal_output = None
            last_direction = direction
        if not estimates:
            return ["Backlash estimate unavailable from collected samples."]
        motor_backlash = mean(estimates)
        gear = max(float(self.params["transmission.gear_ratio"]), 1.0e-9)
        spool_radius = max(float(self.params["transmission.spool_radius_m"]), 1.0e-9)
        moment_arm = max(float(self.params["transmission.tendon_moment_arm_m"]), 1.0e-9)
        finger_length = max(float(self.params["finger.length_m"]), 1.0e-9)
        finger_backlash = motor_backlash / gear * spool_radius / moment_arm
        output_mm = finger_backlash * finger_length * 1000.0
        return [
            f"Estimated motor-side deadband: {motor_backlash:.4f} rad",
            f"Estimated fingertip equivalent deadband: {output_mm:.3f} mm",
        ]

    def _crossing_time(self, values: List[Any], threshold: float) -> Optional[float]:
        for index in range(1, len(values)):
            t0, y0 = values[index - 1]
            t1, y1 = values[index]
            if (y0 <= threshold <= y1) or (y1 <= threshold <= y0):
                if abs(y1 - y0) < 1.0e-12:
                    return t1
                alpha = (threshold - y0) / (y1 - y0)
                return t0 + alpha * (t1 - t0)
        return None

    def _publish_float(self, publisher: Any, value: float) -> None:
        msg = Float64()
        msg.data = value
        publisher.publish(msg)

    def _publish_test_command(self, payload: Any) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self.test_pub.publish(msg)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = TestExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
