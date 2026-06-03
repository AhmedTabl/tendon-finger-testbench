"""ROS 2 plant simulation node for the tendon finger testbench."""

from __future__ import annotations

import json
import math
import os
from typing import Any, Dict

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from tendon_finger_testbench.defaults import PLANT_DEFAULTS
from tendon_finger_testbench.model import TendonFingerPlant
from tendon_finger_testbench.ros_helpers import (
    declare_parameters,
    parameters_to_dict,
    read_parameters,
)


class MuJoCoBridge:
    """Tiny MuJoCo integration used for model loading and optional mirroring.

    The Python plant is authoritative for this first pass because it exposes
    tendon slack, backlash, and sensor effects directly. If MuJoCo is installed,
    the XML model is loaded and the simulated finger state is mirrored into the
    MuJoCo data object. Enabling ``mujoco.launch_viewer`` opens the passive
    viewer when supported by the installed MuJoCo package.
    """

    def __init__(self, node: Node, xml_path: str, enabled: bool, launch_viewer: bool) -> None:
        self.node = node
        self.enabled = False
        self.model = None
        self.data = None
        self.mujoco = None
        self.viewer = None

        if not enabled:
            node.get_logger().info("MuJoCo bridge disabled by parameter.")
            return
        if not xml_path or not os.path.exists(xml_path):
            node.get_logger().warn(f"MuJoCo XML not found: {xml_path!r}; using Python plant only.")
            return

        try:
            import mujoco  # type: ignore

            self.mujoco = mujoco
            self.model = mujoco.MjModel.from_xml_path(xml_path)
            self.data = mujoco.MjData(self.model)
            self.enabled = True
            node.get_logger().info(f"Loaded MuJoCo model: {xml_path}")

            if launch_viewer:
                try:
                    import mujoco.viewer  # type: ignore

                    self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                    node.get_logger().info("MuJoCo passive viewer launched.")
                except Exception as exc:  # pragma: no cover - viewer is environment-specific.
                    node.get_logger().warn(f"MuJoCo viewer unavailable: {exc}")
        except Exception as exc:
            node.get_logger().warn(f"MuJoCo import/load failed: {exc}; using Python plant only.")

    def sync(self, sample: Dict[str, float]) -> None:
        if not self.enabled or self.model is None or self.data is None or self.mujoco is None:
            return
        if self.model.nq > 0:
            self.data.qpos[0] = sample["finger_position"]
        if self.model.nv > 0:
            self.data.qvel[0] = sample["finger_velocity"]
        if self.model.nu > 0:
            self.data.ctrl[0] = sample["tendon_torque_finger"]
        self.mujoco.mj_forward(self.model, self.data)
        if self.viewer is not None:
            try:
                self.viewer.sync()
            except Exception as exc:  # pragma: no cover - viewer is environment-specific.
                self.node.get_logger().warn(f"MuJoCo viewer sync failed: {exc}")
                self.viewer = None


class PlantSimNode(Node):
    def __init__(self) -> None:
        super().__init__("plant_sim_node")
        declare_parameters(self, PLANT_DEFAULTS)
        params = read_parameters(self, PLANT_DEFAULTS)

        xml_path = str(params.get("mujoco.xml_path", ""))
        if not xml_path:
            xml_path = self._default_mujoco_xml_path()
            params["mujoco.xml_path"] = xml_path

        self.plant = TendonFingerPlant(params)
        self.commanded_torque = 0.0
        self.last_sample: Dict[str, float] = {}
        self.sequence = 0

        self.mujoco = MuJoCoBridge(
            self,
            xml_path,
            bool(params.get("mujoco.enable", True)),
            bool(params.get("mujoco.launch_viewer", False)),
        )

        self.true_pub = self.create_publisher(JointState, "/finger/true_state", 10)
        self.sensor_pub = self.create_publisher(JointState, "/finger/sensor_state", 10)
        self.motor_encoder_pub = self.create_publisher(Float64, "/finger/motor_encoder", 10)
        self.output_sensor_pub = self.create_publisher(Float64, "/finger/output_sensor", 10)
        self.safety_pub = self.create_publisher(DiagnosticArray, "/finger/safety_status", 10)

        self.create_subscription(Float64, "/finger/command_torque", self._torque_callback, 10)
        self.create_subscription(String, "/finger/test_command", self._test_command_callback, 10)

        self.add_on_set_parameters_callback(self._on_parameters)

        self.update_rate_hz = max(float(params["update_rate_hz"]), 1.0)
        self.dt = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(self.dt, self._timer_callback)
        self.get_logger().info(
            f"Plant simulation running at {self.update_rate_hz:.1f} Hz; "
            f"MuJoCo {'enabled' if self.mujoco.enabled else 'not active'}."
        )

    def _default_mujoco_xml_path(self) -> str:
        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory("tendon_finger_testbench")
            return os.path.join(share, "mujoco", "tendon_finger.xml")
        except Exception:
            here = os.path.dirname(os.path.abspath(__file__))
            return os.path.abspath(os.path.join(here, "..", "mujoco", "tendon_finger.xml"))

    def _on_parameters(self, parameters: Any) -> SetParametersResult:
        updates = parameters_to_dict(parameters)
        self.plant.update_params(updates)
        if "load.external_torque" in updates:
            self.plant.set_external_load(float(updates["load.external_torque"]))
        if "update_rate_hz" in updates:
            self.get_logger().warn("update_rate_hz changed; restart node to rebuild the timer.")
        return SetParametersResult(successful=True)

    def _torque_callback(self, msg: Float64) -> None:
        self.commanded_torque = msg.data

    def _test_command_callback(self, msg: String) -> None:
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignoring malformed test command: {msg.data}")
            return

        if command.get("reset"):
            self.plant.reset()
            self.get_logger().info("Plant reset from /finger/test_command.")
        if "external_load" in command:
            self.plant.set_external_load(float(command["external_load"]))
            self.get_logger().info(
                f"External load set to {self.plant.state.external_load:.4f} N m."
            )

    def _timer_callback(self) -> None:
        sample = self.plant.step(self.commanded_torque, self.dt)
        self.last_sample = sample
        self.sequence += 1
        self.mujoco.sync(sample)

        stamp = self.get_clock().now().to_msg()
        self.true_pub.publish(self._make_true_state(stamp, sample))
        self.sensor_pub.publish(self._make_sensor_state(stamp, sample))

        motor_msg = Float64()
        motor_msg.data = sample["measured_motor_position"]
        self.motor_encoder_pub.publish(motor_msg)

        output_msg = Float64()
        output_msg.data = sample["measured_output_position"]
        self.output_sensor_pub.publish(output_msg)

        self.safety_pub.publish(self._make_safety_status(stamp, sample))

    def _make_true_state(self, stamp: Any, sample: Dict[str, float]) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [
            "motor_shaft",
            "finger_joint",
            "tendon_drive",
            "spool_transmission",
        ]
        msg.position = [
            sample["motor_position"],
            sample["finger_position"],
            sample["tendon_drive_angle"],
            sample["transmitted_spool_angle"],
        ]
        msg.velocity = [
            sample["motor_velocity"],
            sample["finger_velocity"],
            0.0,
            0.0,
        ]
        msg.effort = [
            sample["applied_torque"],
            sample["tendon_torque_finger"],
            sample["cable_tension"],
            sample["tendon_torque_motor"],
        ]
        return msg

    def _make_sensor_state(self, stamp: Any, sample: Dict[str, float]) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [
            "motor_encoder",
            "output_sensor",
            "cable_tension",
            "temperature",
            "external_load",
        ]
        msg.position = [
            sample["measured_motor_position"],
            sample["measured_output_position"],
            sample["cable_tension"],
            sample["temperature"],
            sample["external_load"],
        ]
        msg.velocity = [
            sample["measured_motor_velocity"],
            sample["measured_output_velocity"],
            0.0,
            0.0,
            0.0,
        ]
        msg.effort = [
            sample["applied_torque"],
            sample["current"],
            sample["cable_tension"],
            sample["temperature"],
            sample["external_load"],
        ]
        return msg

    def _make_safety_status(self, stamp: Any, sample: Dict[str, float]) -> DiagnosticArray:
        array = DiagnosticArray()
        array.header.stamp = stamp

        status = DiagnosticStatus()
        status.name = "tendon_finger_testbench/safety"
        status.hardware_id = "simulated_tendon_finger"
        events = str(sample.get("events", ""))
        if not math.isfinite(sample.get("safe", 0.0)) or sample.get("safe", 0.0) < 0.5:
            status.level = DiagnosticStatus.WARN
            status.message = events or "safety warning"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "OK"

        status.values = [
            KeyValue(key="events", value=events),
            KeyValue(key="temperature_c", value=f"{sample['temperature']:.3f}"),
            KeyValue(key="current_a", value=f"{sample['current']:.3f}"),
            KeyValue(key="applied_torque_nm", value=f"{sample['applied_torque']:.4f}"),
            KeyValue(key="external_load_nm", value=f"{sample['external_load']:.4f}"),
        ]
        array.status.append(status)
        return array


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = PlantSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

