"""ROS 2 MuJoCo plant for the tendon-driven finger testbench."""

from __future__ import annotations

import json
import math
import os
import random
from typing import Any, Dict, Mapping, Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from tendon_finger_testbench.defaults import PLANT_DEFAULTS
from tendon_finger_testbench.model import (
    FixedStepDelay,
    TendonFingerPlant,
    clamp,
    quantize_angle,
)
from tendon_finger_testbench.ros_helpers import (
    declare_parameters,
    parameters_to_dict,
    read_parameters,
)


class MuJoCoTendonPlant:
    """MuJoCo-authoritative motor, gearbox, tendon, and finger mechanism."""

    def __init__(
        self,
        node: Node,
        xml_path: str,
        params: Mapping[str, Any],
        launch_viewer: bool,
    ) -> None:
        import mujoco  # type: ignore

        self.node = node
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.params: Dict[str, Any] = dict(params)
        self.viewer = None
        self.command_delay = FixedStepDelay()
        self.sensor_delay = FixedStepDelay()
        self._rng = random.Random(int(self._p("sensors.random_seed", 7)))
        self.applied_torque = 0.0
        self.temperature = self._p("thermal.ambient_temperature_c", 25.0)
        self.external_load = self._p("load.external_torque", 0.0)

        self.motor_joint = self._id(mujoco.mjtObj.mjOBJ_JOINT, "motor_shaft_joint")
        self.spool_joint = self._id(mujoco.mjtObj.mjOBJ_JOINT, "spool_joint")
        self.finger_joint = self._id(mujoco.mjtObj.mjOBJ_JOINT, "finger_joint")
        self.drive_tendon = self._id(mujoco.mjtObj.mjOBJ_TENDON, "drive_tendon")
        self.cable_path = self._id(mujoco.mjtObj.mjOBJ_TENDON, "cable_path")
        self.motor_actuator = self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, "motor_torque")
        self.gearbox_equality = self._id(
            mujoco.mjtObj.mjOBJ_EQUALITY, "gearbox_constraint"
        )
        self.guide_equality = self._id(
            mujoco.mjtObj.mjOBJ_EQUALITY, "guide_pulley_constraint"
        )
        self.finger_body = self._id(mujoco.mjtObj.mjOBJ_BODY, "finger_link")

        self.motor_qpos = int(self.model.jnt_qposadr[self.motor_joint])
        self.spool_qpos = int(self.model.jnt_qposadr[self.spool_joint])
        self.finger_qpos = int(self.model.jnt_qposadr[self.finger_joint])
        self.motor_dof = int(self.model.jnt_dofadr[self.motor_joint])
        self.spool_dof = int(self.model.jnt_dofadr[self.spool_joint])
        self.finger_dof = int(self.model.jnt_dofadr[self.finger_joint])

        self._base_finger_mass = float(self.model.body_mass[self.finger_body])
        self._base_finger_inertia = self.model.body_inertia[self.finger_body].copy()
        self._geom_ids = {
            name: self._id(mujoco.mjtObj.mjOBJ_GEOM, name)
            for name in (
                "spool_hub",
                "spool_left_flange",
                "spool_right_flange",
                "spool_cable_layer",
                "finger_core",
                "finger_shell",
                "fingertip_pad",
            )
        }
        self._site_ids = {
            name: self._id(mujoco.mjtObj.mjOBJ_SITE, name)
            for name in ("spool_exit", "finger_tendon_anchor", "fingertip")
        }

        self.update_params(params)
        self.reset()
        node.get_logger().info(f"Loaded MuJoCo mechanism: {xml_path}")

        if launch_viewer:
            try:
                import mujoco.viewer  # type: ignore

                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                node.get_logger().info("MuJoCo passive viewer launched.")
            except Exception as exc:  # pragma: no cover - display-specific.
                node.get_logger().warn(f"MuJoCo viewer unavailable: {exc}")

    def _id(self, object_type: Any, name: str) -> int:
        object_id = self.mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise RuntimeError(f"MuJoCo model is missing required object {name!r}")
        return int(object_id)

    def _p(self, name: str, default: float) -> float:
        value = self.params.get(name, default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _bool(self, name: str, default: bool) -> bool:
        value = self.params.get(name, default)
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _effective_slack(self, moment_arm: float) -> float:
        cable_slack = max(self._p("tendon.slack_m", 0.001), 0.0)
        finger_length = max(self._p("finger.length_m", 0.09), 1.0e-5)
        output_deadband = max(
            self._p("backlash.output_deadband_mm", 0.0), 0.0
        ) * 0.001
        return cable_slack + output_deadband * moment_arm / finger_length

    def _low_speed_friction(
        self,
        velocity: float,
        coulomb_friction: float,
        static_friction: float,
    ) -> float:
        stiction_velocity = max(
            self._p("friction.stiction_velocity_rad_s", 0.025), 1.0e-5
        )
        static_delta = max(static_friction - coulomb_friction, 0.0)
        stribeck = static_delta * math.exp(
            -(abs(velocity) / stiction_velocity) ** 2
        )
        return stribeck * math.tanh(velocity / (0.2 * stiction_velocity))

    def update_params(self, params: Mapping[str, Any]) -> None:
        self.params.update(dict(params))
        self._rng.seed(int(self._p("sensors.random_seed", 7)))

        gear_ratio = max(self._p("transmission.gear_ratio", 5.0), 1.0e-6)
        spool_radius = max(self._p("transmission.spool_radius_m", 0.012), 1.0e-5)
        moment_arm = max(
            self._p("transmission.tendon_moment_arm_m", 0.018), 1.0e-5
        )
        effective_slack = self._effective_slack(moment_arm)

        self.model.eq_data[self.gearbox_equality, 1] = 1.0 / gear_ratio
        self.model.eq_data[self.guide_equality, 1] = -spool_radius / 0.009
        backlash = max(self._p("backlash.motor_deadband_rad", 0.0), 0.0)
        self.model.eq_solimp[self.gearbox_equality] = (
            0.05,
            0.9999,
            max(backlash / gear_ratio, 1.0e-5),
            0.5,
            2.0,
        )

        tendon_adr = int(self.model.tendon_adr[self.drive_tendon])
        self.model.wrap_prm[tendon_adr] = -spool_radius
        self.model.wrap_prm[tendon_adr + 1] = moment_arm
        self.model.tendon_lengthspring[self.drive_tendon, 0] = -effective_slack
        self.model.tendon_lengthspring[self.drive_tendon, 1] = 10.0
        self.model.tendon_stiffness[self.drive_tendon] = max(
            self._p("tendon.stiffness_n_per_m", 850.0), 0.0
        )
        self.model.tendon_damping[self.drive_tendon] = max(
            self._p("tendon.damping_n_s_per_m", 1.5), 0.0
        )

        self.model.dof_armature[self.motor_dof] = max(
            self._p("motor.inertia_kg_m2", 0.00012), 0.0
        )
        self.model.dof_damping[self.motor_dof] = max(
            self._p("motor.viscous_damping_nm_s_per_rad", 0.0002), 0.0
        )
        self.model.dof_frictionloss[self.motor_dof] = max(
            self._p("motor.coulomb_friction_nm", 0.0003), 0.0
        )
        self.model.jnt_stiffness[self.motor_joint] = max(
            self._p("motor.stiffness_nm_per_rad", 0.0), 0.0
        )
        self.model.dof_armature[self.spool_dof] = max(
            self._p("transmission.spool_inertia_kg_m2", 0.00035), 0.0
        )
        self.model.dof_damping[self.spool_dof] = max(
            self._p(
                "transmission.spool_viscous_damping_nm_s_per_rad", 0.0002
            ),
            0.0,
        )
        self.model.dof_frictionloss[self.spool_dof] = max(
            self._p("transmission.spool_coulomb_friction_nm", 0.0002), 0.0
        )

        self.model.dof_armature[self.finger_dof] = max(
            self._p("finger.inertia_kg_m2", 0.00018), 0.0
        )
        self.model.dof_damping[self.finger_dof] = max(
            self._p("finger.viscous_damping_nm_s_per_rad", 0.0015), 0.0
        )
        self.model.dof_frictionloss[self.finger_dof] = max(
            self._p("finger.coulomb_friction_nm", 0.0008), 0.0
        )
        self.model.jnt_stiffness[self.finger_joint] = max(
            self._p("finger.joint_stiffness_nm_per_rad", 0.02), 0.0
        )

        finger_mass = max(self._p("finger.mass_kg", 0.055), 1.0e-6)
        mass_scale = finger_mass / max(self._base_finger_mass, 1.0e-9)
        self.model.body_mass[self.finger_body] = finger_mass
        self.model.body_inertia[self.finger_body] = (
            self._base_finger_inertia * mass_scale
        )
        self.model.body_ipos[self.finger_body, 2] = -clamp(
            self._p("finger.com_length_m", 0.045),
            0.0,
            max(self._p("finger.length_m", 0.09), 1.0e-4),
        )

        self.model.opt.gravity[:] = (
            (0.0, 0.0, -9.80665)
            if self._bool("finger.enable_gravity", True)
            else (0.0, 0.0, 0.0)
        )
        self.model.jnt_range[self.finger_joint] = (
            self._p("safety.min_position_rad", -0.05),
            self._p("safety.max_position_rad", 1.35),
        )
        self.model.jnt_range[self.motor_joint, 0] = self._p(
            "safety.min_motor_position_rad", 0.0
        )

        kt = max(self._p("motor.torque_constant_nm_per_a", 0.11), 1.0e-9)
        torque_limit = min(
            abs(self._p("safety.max_torque_nm", 0.35)),
            kt * abs(self._p("safety.max_current_a", 2.5)),
        )
        minimum_torque = clamp(
            self._p("safety.min_torque_nm", 0.0), -torque_limit, torque_limit
        )
        self.model.actuator_ctrlrange[self.motor_actuator] = (
            minimum_torque,
            torque_limit,
        )

        self._update_visual_dimensions(spool_radius, moment_arm)
        self.mujoco.mj_forward(self.model, self.data)

    def _update_visual_dimensions(self, spool_radius: float, moment_arm: float) -> None:
        hub = self._geom_ids["spool_hub"]
        self.model.geom_size[hub, 0] = spool_radius
        cable_layer = self._geom_ids["spool_cable_layer"]
        self.model.geom_size[cable_layer, 0] = spool_radius + 0.0008
        for name in ("spool_left_flange", "spool_right_flange"):
            self.model.geom_size[self._geom_ids[name], 0] = spool_radius + 0.006
        self.model.site_pos[self._site_ids["spool_exit"], 2] = spool_radius + 0.0006

        anchor = self._site_ids["finger_tendon_anchor"]
        self.model.site_pos[anchor, 0] = -moment_arm

        length = max(self._p("finger.length_m", 0.09), 0.025)
        core = self._geom_ids["finger_core"]
        self.model.geom_pos[core, 2] = -0.5 * length
        self.model.geom_size[core, 1] = 0.5 * length
        shell = self._geom_ids["finger_shell"]
        self.model.geom_pos[shell, 2] = -0.5 * length
        self.model.geom_size[shell, 2] = max(0.5 * length - 0.002, 0.005)
        tip_start = max(length - 0.009, 0.016)
        tip_end = length + 0.004
        tip_geom = self._geom_ids["fingertip_pad"]
        self.model.geom_pos[tip_geom, 2] = -0.5 * (tip_start + tip_end)
        self.model.geom_size[tip_geom, 1] = 0.5 * (tip_end - tip_start)
        self.model.site_pos[self._site_ids["fingertip"], 2] = -tip_end

    def reset(self) -> None:
        self.mujoco.mj_resetData(self.model, self.data)
        self.command_delay.reset()
        self.sensor_delay.reset()
        self.applied_torque = 0.0
        self.temperature = self._p("thermal.ambient_temperature_c", 25.0)
        self.external_load = self._p("load.external_torque", 0.0)
        self.mujoco.mj_forward(self.model, self.data)

    def set_external_load(self, torque_nm: float) -> None:
        self.external_load = float(torque_nm)

    def step(self, commanded_motor_torque: float, dt: float) -> Dict[str, float]:
        dt = clamp(dt, 1.0e-5, 0.01)
        events = []
        if not math.isfinite(commanded_motor_torque):
            commanded_motor_torque = 0.0
            events.append("invalid torque command replaced with zero")

        delayed_command = self.command_delay.push(
            commanded_motor_torque,
            self._p("delay.command_delay_s", 0.0),
            dt,
        )
        kt = max(self._p("motor.torque_constant_nm_per_a", 0.11), 1.0e-9)
        torque_limit = min(
            abs(self._p("safety.max_torque_nm", 0.35)),
            kt * abs(self._p("safety.max_current_a", 2.5)),
        )
        minimum_torque = clamp(
            self._p("safety.min_torque_nm", 0.0), -torque_limit, torque_limit
        )
        if self.temperature >= self._p("safety.max_temperature_c", 85.0):
            delayed_command = 0.0
            events.append("temperature limit exceeded; torque disabled")

        saturated_command = clamp(delayed_command, minimum_torque, torque_limit)
        if saturated_command != delayed_command:
            events.append("torque/current command saturated")

        lag_tau = max(self._p("actuator.torque_lag_time_constant_s", 0.008), 0.0)
        alpha = 1.0 if lag_tau <= dt else clamp(dt / lag_tau, 0.0, 1.0)
        self.applied_torque += alpha * (saturated_command - self.applied_torque)

        self.model.opt.timestep = dt
        self.data.ctrl[self.motor_actuator] = self.applied_torque
        self.data.qfrc_applied[:] = 0.0
        motor_friction = self._low_speed_friction(
            float(self.data.qvel[self.motor_dof]),
            self._p("motor.coulomb_friction_nm", 0.0003),
            self._p("motor.static_friction_nm", 0.0005),
        )
        finger_friction = self._low_speed_friction(
            float(self.data.qvel[self.finger_dof]),
            self._p("finger.coulomb_friction_nm", 0.0008),
            self._p("finger.static_friction_nm", 0.0012),
        )
        self.data.qfrc_applied[self.motor_dof] = -motor_friction
        self.data.qfrc_applied[self.finger_dof] = (
            -self.external_load - finger_friction
        )
        self.mujoco.mj_step(self.model, self.data)

        max_velocity = abs(self._p("safety.max_velocity_rad_s", 18.0))
        for dof in (self.motor_dof, self.spool_dof, self.finger_dof):
            if abs(self.data.qvel[dof]) > max_velocity:
                self.data.qvel[dof] = clamp(
                    self.data.qvel[dof], -max_velocity, max_velocity
                )
                events.append("joint velocity clamped")

        current = self.applied_torque / kt
        ambient = self._p("thermal.ambient_temperature_c", 25.0)
        heating = self._p("thermal.heating_coeff_c_per_a2_s", 0.09) * current**2
        cooling = self._p("thermal.cooling_coeff_per_s", 0.025) * (
            self.temperature - ambient
        )
        self.temperature += (heating - cooling) * dt

        sample = self._sample_state(commanded_motor_torque, delayed_command, current)
        if not all(
            math.isfinite(sample[key])
            for key in (
                "motor_position",
                "motor_velocity",
                "finger_position",
                "finger_velocity",
                "cable_tension",
                "temperature",
            )
        ):
            events.append("invalid MuJoCo state detected; plant reset")
            self.reset()
            sample = self._sample_state(0.0, 0.0, 0.0)

        sample["safe"] = 1.0 if not events else 0.0
        sample["events"] = "; ".join(dict.fromkeys(events))
        self._sync_viewer()
        return sample

    def _sample_state(
        self,
        commanded_torque: float,
        delayed_command: float,
        current: float,
    ) -> Dict[str, float]:
        motor_position = float(self.data.qpos[self.motor_qpos])
        motor_velocity = float(self.data.qvel[self.motor_dof])
        spool_position = float(self.data.qpos[self.spool_qpos])
        spool_velocity = float(self.data.qvel[self.spool_dof])
        finger_position = float(self.data.qpos[self.finger_qpos])
        finger_velocity = float(self.data.qvel[self.finger_dof])
        tendon_length = float(self.data.ten_length[self.drive_tendon])
        tendon_velocity = float(self.data.ten_velocity[self.drive_tendon])

        moment_arm = max(
            self._p("transmission.tendon_moment_arm_m", 0.018), 1.0e-9
        )
        effective_slack = self._effective_slack(moment_arm)
        tendon_extension = max(0.0, -effective_slack - tendon_length)
        cable_tension = max(
            0.0,
            self._p("tendon.stiffness_n_per_m", 850.0) * tendon_extension
            - self._p("tendon.damping_n_s_per_m", 1.5) * tendon_velocity,
        )
        spool_radius = max(self._p("transmission.spool_radius_m", 0.012), 1.0e-9)

        raw_sensor = {
            "measured_motor_position": quantize_angle(
                motor_position, self._p("sensors.encoder_counts_per_rev", 4096.0)
            )
            + self._rng.gauss(
                0.0, max(self._p("sensors.encoder_noise_std_rad", 0.0), 0.0)
            ),
            "measured_motor_velocity": motor_velocity
            + self._rng.gauss(
                0.0,
                max(
                    self._p("sensors.encoder_velocity_noise_std_rad_s", 0.0), 0.0
                ),
            ),
        }
        if self._bool("sensors.output_sensor_enabled", True):
            raw_sensor["measured_output_position"] = finger_position + self._rng.gauss(
                0.0,
                max(self._p("sensors.output_sensor_noise_std_rad", 0.0), 0.0),
            )
            raw_sensor["measured_output_velocity"] = finger_velocity + self._rng.gauss(
                0.0,
                max(
                    self._p(
                        "sensors.output_sensor_velocity_noise_std_rad_s", 0.0
                    ),
                    0.0,
                ),
            )
        else:
            raw_sensor["measured_output_position"] = float("nan")
            raw_sensor["measured_output_velocity"] = float("nan")
        sensor = self.sensor_delay.push(
            raw_sensor,
            self._p("delay.measurement_delay_s", 0.0),
            max(float(self.model.opt.timestep), 1.0e-9),
        )

        gravity_torque = 0.0
        if self._bool("finger.enable_gravity", True):
            gravity_torque = (
                self._p("finger.mass_kg", 0.055)
                * 9.80665
                * self._p("finger.com_length_m", 0.045)
                * math.sin(finger_position)
            )

        return {
            "motor_position": motor_position,
            "motor_velocity": motor_velocity,
            "finger_position": finger_position,
            "finger_velocity": finger_velocity,
            "transmitted_spool_angle": spool_position,
            "spool_velocity": spool_velocity,
            "tendon_drive_angle": spool_position
            * spool_radius
            / moment_arm,
            "tendon_length": tendon_length,
            "tendon_velocity": tendon_velocity,
            "visual_cable_length": float(self.data.ten_length[self.cable_path]),
            "cable_tension": cable_tension,
            "tendon_torque_motor": cable_tension * spool_radius,
            "tendon_torque_finger": cable_tension * moment_arm,
            "motor_friction_torque": (
                self._p("motor.viscous_damping_nm_s_per_rad", 0.0002)
                * motor_velocity
                + self._p("motor.coulomb_friction_nm", 0.0003)
                * math.tanh(motor_velocity / 1.0e-4)
                + self._low_speed_friction(
                    motor_velocity,
                    self._p("motor.coulomb_friction_nm", 0.0003),
                    self._p("motor.static_friction_nm", 0.0005),
                )
            ),
            "finger_friction_torque": (
                self._p("finger.viscous_damping_nm_s_per_rad", 0.0015)
                * finger_velocity
                + self._p("finger.coulomb_friction_nm", 0.0008)
                * math.tanh(finger_velocity / 1.0e-4)
                + self._low_speed_friction(
                    finger_velocity,
                    self._p("finger.coulomb_friction_nm", 0.0008),
                    self._p("finger.static_friction_nm", 0.0012),
                )
            ),
            "gravity_torque": gravity_torque,
            "commanded_torque": commanded_torque,
            "delayed_commanded_torque": delayed_command,
            "applied_torque": self.applied_torque,
            "current": current,
            "external_load": self.external_load,
            "temperature": self.temperature,
            "measured_motor_position": sensor["measured_motor_position"],
            "measured_motor_velocity": sensor["measured_motor_velocity"],
            "measured_output_position": sensor["measured_output_position"],
            "measured_output_velocity": sensor["measured_output_velocity"],
        }

    def _sync_viewer(self) -> None:
        if self.viewer is None:
            return
        try:
            if self.viewer.is_running():
                self.viewer.sync()
            else:
                self.viewer = None
        except Exception as exc:  # pragma: no cover - display-specific.
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

        self.using_mujoco = bool(params.get("mujoco.enable", True))
        if self.using_mujoco:
            try:
                self.plant: Any = MuJoCoTendonPlant(
                    self,
                    xml_path,
                    params,
                    bool(params.get("mujoco.launch_viewer", True)),
                )
            except Exception as exc:
                raise RuntimeError(f"MuJoCo plant initialization failed: {exc}") from exc
        else:
            self.get_logger().warn(
                "MuJoCo disabled; falling back to the legacy Python plant."
            )
            self.plant = TendonFingerPlant(params)

        self.commanded_torque = 0.0
        self.last_sample: Dict[str, float] = {}
        self.true_pub = self.create_publisher(JointState, "/finger/true_state", 10)
        self.sensor_pub = self.create_publisher(JointState, "/finger/sensor_state", 10)
        self.motor_encoder_pub = self.create_publisher(
            Float64, "/finger/motor_encoder", 10
        )
        self.output_sensor_pub = self.create_publisher(
            Float64, "/finger/output_sensor", 10
        )
        self.safety_pub = self.create_publisher(
            DiagnosticArray, "/finger/safety_status", 10
        )

        self.create_subscription(
            Float64, "/finger/command_torque", self._torque_callback, 10
        )
        self.create_subscription(
            String, "/finger/test_command", self._test_command_callback, 10
        )
        self.add_on_set_parameters_callback(self._on_parameters)

        self.update_rate_hz = max(float(params["update_rate_hz"]), 1.0)
        self.dt = 1.0 / self.update_rate_hz
        self.last_update_time = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self._timer_callback)
        self.get_logger().info(
            f"Plant simulation running at {self.update_rate_hz:.1f} Hz; "
            f"{'MuJoCo authoritative' if self.using_mujoco else 'legacy Python plant'}."
        )

    def _default_mujoco_xml_path(self) -> str:
        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory("tendon_finger_testbench")
            return os.path.join(share, "mujoco", "tendon_finger.xml")
        except Exception:
            here = os.path.dirname(os.path.abspath(__file__))
            return os.path.abspath(
                os.path.join(here, "..", "mujoco", "tendon_finger.xml")
            )

    def _on_parameters(self, parameters: Any) -> SetParametersResult:
        updates = parameters_to_dict(parameters)
        try:
            self.plant.update_params(updates)
            if "load.external_torque" in updates:
                self.plant.set_external_load(float(updates["load.external_torque"]))
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        if "update_rate_hz" in updates:
            self.get_logger().warn(
                "update_rate_hz changed; restart node to rebuild the timer."
            )
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
            self.commanded_torque = 0.0
            self.last_update_time = self.get_clock().now()
            self.get_logger().info("Plant reset from /finger/test_command.")
        if "external_load" in command:
            self.plant.set_external_load(float(command["external_load"]))
            load_value = (
                self.plant.external_load
                if hasattr(self.plant, "external_load")
                else self.plant.state.external_load
            )
            self.get_logger().info(
                f"External load set to {load_value:.4f} N m."
            )

    def _timer_callback(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.last_update_time).nanoseconds * 1.0e-9
        self.last_update_time = now
        sample = self.plant.step(
            self.commanded_torque,
            clamp(elapsed, 0.25 * self.dt, 4.0 * self.dt),
        )
        self.last_sample = sample

        stamp = now.to_msg()
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
            "spool_output",
            "finger_joint",
            "drive_tendon",
            "cable_path",
        ]
        msg.position = [
            sample["motor_position"],
            sample["transmitted_spool_angle"],
            sample["finger_position"],
            sample.get("tendon_length", 0.0),
            sample.get("visual_cable_length", 0.0),
        ]
        msg.velocity = [
            sample["motor_velocity"],
            sample.get("spool_velocity", 0.0),
            sample["finger_velocity"],
            sample.get("tendon_velocity", 0.0),
            0.0,
        ]
        msg.effort = [
            sample["applied_torque"],
            sample["tendon_torque_motor"],
            sample["tendon_torque_finger"],
            sample["cable_tension"],
            sample["cable_tension"],
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

    def _make_safety_status(
        self, stamp: Any, sample: Dict[str, float]
    ) -> DiagnosticArray:
        array = DiagnosticArray()
        array.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = "tendon_finger_testbench/safety"
        status.hardware_id = "mujoco_tendon_finger"
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
            KeyValue(
                key="applied_torque_nm", value=f"{sample['applied_torque']:.4f}"
            ),
            KeyValue(
                key="cable_tension_n", value=f"{sample['cable_tension']:.3f}"
            ),
            KeyValue(
                key="external_load_nm", value=f"{sample['external_load']:.4f}"
            ),
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
