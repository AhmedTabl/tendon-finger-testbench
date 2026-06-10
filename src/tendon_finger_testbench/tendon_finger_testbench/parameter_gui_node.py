"""Tkinter control panel for live testbench tuning and test execution."""

from __future__ import annotations

from dataclasses import dataclass
import json
import queue
import threading
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from tendon_finger_testbench.defaults import (
    CONTROLLER_DEFAULTS,
    PLANT_DEFAULTS,
    TEST_DEFAULTS,
)
from tendon_finger_testbench.ros_helpers import joint_value


try:
    import tkinter as tk
    from tkinter import ttk
except Exception:  # pragma: no cover - exercised only on systems without Tk.
    tk = None
    ttk = None


PLANT_NODE = "/plant_sim_node"
CONTROLLER_NODE = "/controller_node"
TEST_NODE = "/test_executor_node"

TEST_MODES = ("step", "backlash", "friction", "sine", "chirp", "load", "thermal")


@dataclass(frozen=True)
class ParamSpec:
    name: str
    label: str
    default: Any
    choices: Optional[Sequence[str]] = None


PARAMETER_LABELS: Dict[str, str] = {
    "motor.inertia_kg_m2": "Inertia [kg m^2]",
    "motor.viscous_damping_nm_s_per_rad": "Viscous damping [N m s/rad]",
    "motor.coulomb_friction_nm": "Coulomb friction [N m]",
    "motor.static_friction_nm": "Static friction [N m]",
    "motor.stiffness_nm_per_rad": "Stiffness [N m/rad]",
    "motor.torque_constant_nm_per_a": "Torque constant [N m/A]",
    "actuator.torque_lag_time_constant_s": "Torque lag time constant [s]",
    "thermal.ambient_temperature_c": "Ambient temperature [deg C]",
    "thermal.heating_coeff_c_per_a2_s": "Heating coefficient [deg C/(A^2 s)]",
    "thermal.cooling_coeff_per_s": "Cooling coefficient [1/s]",
    "safety.max_current_a": "Maximum current [A]",
    "safety.min_torque_nm": "Minimum torque [N m]",
    "safety.max_torque_nm": "Maximum torque [N m]",
    "safety.max_velocity_rad_s": "Maximum velocity [rad/s]",
    "safety.min_motor_position_rad": "Minimum motor position [rad]",
    "safety.min_position_rad": "Minimum finger position [rad]",
    "safety.max_position_rad": "Maximum finger position [rad]",
    "safety.max_temperature_c": "Maximum temperature [deg C]",
    "finger.inertia_kg_m2": "Inertia [kg m^2]",
    "finger.length_m": "Length [m]",
    "finger.mass_kg": "Mass [kg]",
    "finger.com_length_m": "Center-of-mass length [m]",
    "finger.viscous_damping_nm_s_per_rad": "Viscous damping [N m s/rad]",
    "finger.coulomb_friction_nm": "Coulomb friction [N m]",
    "finger.static_friction_nm": "Static friction [N m]",
    "finger.joint_stiffness_nm_per_rad": "Joint stiffness [N m/rad]",
    "finger.enable_gravity": "Gravity enabled [on/off]",
    "friction.stiction_velocity_rad_s": "Stiction velocity [rad/s]",
    "transmission.gear_ratio": "Gear ratio [-]",
    "transmission.spool_radius_m": "Spool radius [m]",
    "transmission.spool_inertia_kg_m2": "Spool inertia [kg m^2]",
    "transmission.spool_viscous_damping_nm_s_per_rad": "Spool viscous damping [N m s/rad]",
    "transmission.spool_coulomb_friction_nm": "Spool Coulomb friction [N m]",
    "transmission.tendon_moment_arm_m": "Tendon moment arm [m]",
    "tendon.stiffness_n_per_m": "Stiffness [N/m]",
    "tendon.damping_n_s_per_m": "Damping [N s/m]",
    "tendon.slack_m": "Slack [m]",
    "backlash.motor_deadband_rad": "Motor deadband [rad]",
    "backlash.output_deadband_mm": "Output deadband [mm]",
    "load.external_torque": "External load [N m]",
    "sensors.encoder_counts_per_rev": "Encoder resolution [counts/rev]",
    "sensors.encoder_noise_std_rad": "Encoder position noise [rad]",
    "sensors.encoder_velocity_noise_std_rad_s": "Encoder velocity noise [rad/s]",
    "sensors.output_sensor_enabled": "Output sensor enabled [on/off]",
    "sensors.output_sensor_noise_std_rad": "Output position noise [rad]",
    "sensors.output_sensor_velocity_noise_std_rad_s": "Output velocity noise [rad/s]",
    "sensors.random_seed": "Random seed [-]",
    "delay.command_delay_s": "Command delay [s]",
    "delay.measurement_delay_s": "Measurement delay [s]",
    "feedback_source": "Feedback source [selection]",
    "target_position_rad": "Target position [rad]",
    "target_velocity_rad_s": "Target velocity [rad/s]",
    "position.kp": "Proportional gain [1/s]",
    "position.ki": "Integral gain [1/s^2]",
    "position.kd": "Derivative gain [-]",
    "position.integral_limit": "Integral limit [rad s]",
    "position.velocity_limit_rad_s": "Velocity limit [rad/s]",
    "velocity.kp": "Proportional gain [N m s/rad]",
    "velocity.ki": "Integral gain [N m/rad]",
    "velocity.kd": "Derivative gain [N m s^2/rad]",
    "velocity.integral_limit": "Integral limit [rad]",
    "velocity.minimum_torque_nm": "Minimum torque [N m]",
    "velocity.torque_limit_nm": "Torque limit [N m]",
    "feedforward.velocity_gain_nm_per_rad_s": "Velocity gain [N m s/rad]",
    "feedforward.acceleration_gain_nm_per_rad_s2": "Acceleration gain [N m s^2/rad]",
    "feedforward.gravity_compensation": "Gravity compensation [on/off]",
    "feedforward.joint_stiffness_compensation": "Joint stiffness compensation [on/off]",
    "feedforward.friction_compensation": "Friction compensation [on/off]",
    "model.finger_mass_kg": "Finger mass [kg]",
    "model.finger_com_length_m": "Center-of-mass length [m]",
    "model.finger_coulomb_friction_nm": "Coulomb friction [N m]",
    "model.finger_joint_stiffness_nm_per_rad": "Joint stiffness [N m/rad]",
    "duration_s": "Duration [s]",
    "start_delay_s": "Start delay [s]",
    "publish_rate_hz": "Publish rate [Hz]",
    "step.target_rad": "Target [rad]",
    "step.hold_s": "Hold time [s]",
    "backlash.amplitude_rad": "Amplitude [rad]",
    "backlash.cycles": "Cycles [-]",
    "backlash.period_s": "Period [s]",
    "friction.amplitude_rad": "Amplitude [rad]",
    "friction.period_s": "Period [s]",
    "sine.amplitude_rad": "Amplitude [rad]",
    "sine.bias_rad": "Bias [rad]",
    "sine.frequency_hz": "Frequency [Hz]",
    "chirp.amplitude_rad": "Amplitude [rad]",
    "chirp.bias_rad": "Bias [rad]",
    "chirp.start_frequency_hz": "Start frequency [Hz]",
    "chirp.end_frequency_hz": "End frequency [Hz]",
    "load.target_rad": "Target [rad]",
    "load.external_torque_nm": "External load [N m]",
    "thermal.amplitude_rad": "Amplitude [rad]",
    "thermal.bias_rad": "Bias [rad]",
    "thermal.frequency_hz": "Frequency [Hz]",
}


def parameter_label(name: str) -> str:
    return PARAMETER_LABELS.get(name, f"{name.rsplit('.', 1)[-1].replace('_', ' ').title()} [-]")


def specs(
    defaults: Dict[str, Any],
    names: Iterable[str],
    choices: Optional[Dict[str, Sequence[str]]] = None,
) -> List[ParamSpec]:
    choices = choices or {}
    return [ParamSpec(name, parameter_label(name), defaults[name], choices.get(name)) for name in names]


PARAMETER_TABS: List[Tuple[str, List[Tuple[str, str, List[ParamSpec]]]]] = [
    (
        "Motor",
        [
            (
                "Motor And Actuator",
                PLANT_NODE,
                specs(
                    PLANT_DEFAULTS,
                    (
                        "motor.inertia_kg_m2",
                        "motor.viscous_damping_nm_s_per_rad",
                        "motor.coulomb_friction_nm",
                        "motor.static_friction_nm",
                        "motor.stiffness_nm_per_rad",
                        "motor.torque_constant_nm_per_a",
                        "actuator.torque_lag_time_constant_s",
                    ),
                ),
            ),
            (
                "Thermal And Limits",
                PLANT_NODE,
                specs(
                    PLANT_DEFAULTS,
                    (
                        "thermal.ambient_temperature_c",
                        "thermal.heating_coeff_c_per_a2_s",
                        "thermal.cooling_coeff_per_s",
                        "safety.max_current_a",
                        "safety.min_torque_nm",
                        "safety.max_torque_nm",
                        "safety.max_velocity_rad_s",
                        "safety.min_motor_position_rad",
                        "safety.min_position_rad",
                        "safety.max_position_rad",
                        "safety.max_temperature_c",
                    ),
                ),
            ),
        ],
    ),
    (
        "Plant",
        [
            (
                "Finger",
                PLANT_NODE,
                specs(
                    PLANT_DEFAULTS,
                    (
                        "finger.inertia_kg_m2",
                        "finger.length_m",
                        "finger.mass_kg",
                        "finger.com_length_m",
                        "finger.viscous_damping_nm_s_per_rad",
                        "finger.coulomb_friction_nm",
                        "finger.static_friction_nm",
                        "finger.joint_stiffness_nm_per_rad",
                        "finger.enable_gravity",
                        "friction.stiction_velocity_rad_s",
                    ),
                ),
            ),
            (
                "Transmission Tendon Backlash",
                PLANT_NODE,
                specs(
                    PLANT_DEFAULTS,
                    (
                        "transmission.gear_ratio",
                        "transmission.spool_radius_m",
                        "transmission.spool_inertia_kg_m2",
                        "transmission.spool_viscous_damping_nm_s_per_rad",
                        "transmission.spool_coulomb_friction_nm",
                        "transmission.tendon_moment_arm_m",
                        "tendon.stiffness_n_per_m",
                        "tendon.damping_n_s_per_m",
                        "tendon.slack_m",
                        "backlash.motor_deadband_rad",
                        "backlash.output_deadband_mm",
                        "load.external_torque",
                    ),
                ),
            ),
        ],
    ),
    (
        "Sensors",
        [
            (
                "Sensors And Delay",
                PLANT_NODE,
                specs(
                    PLANT_DEFAULTS,
                    (
                        "sensors.encoder_counts_per_rev",
                        "sensors.encoder_noise_std_rad",
                        "sensors.encoder_velocity_noise_std_rad_s",
                        "sensors.output_sensor_enabled",
                        "sensors.output_sensor_noise_std_rad",
                        "sensors.output_sensor_velocity_noise_std_rad_s",
                        "sensors.random_seed",
                        "delay.command_delay_s",
                        "delay.measurement_delay_s",
                    ),
                ),
            ),
        ],
    ),
    (
        "Controller",
        [
            (
                "Targets And Feedback",
                CONTROLLER_NODE,
                specs(
                    CONTROLLER_DEFAULTS,
                    (
                        "feedback_source",
                        "target_position_rad",
                        "target_velocity_rad_s",
                    ),
                    {"feedback_source": ("output_sensor", "motor_encoder")},
                ),
            ),
            (
                "Position Loop",
                CONTROLLER_NODE,
                specs(
                    CONTROLLER_DEFAULTS,
                    (
                        "position.kp",
                        "position.ki",
                        "position.kd",
                        "position.integral_limit",
                        "position.velocity_limit_rad_s",
                    ),
                ),
            ),
            (
                "Velocity Loop",
                CONTROLLER_NODE,
                specs(
                    CONTROLLER_DEFAULTS,
                    (
                        "velocity.kp",
                        "velocity.ki",
                        "velocity.kd",
                        "velocity.integral_limit",
                        "velocity.minimum_torque_nm",
                        "velocity.torque_limit_nm",
                    ),
                ),
            ),
            (
                "Feedforward Model",
                CONTROLLER_NODE,
                specs(
                    CONTROLLER_DEFAULTS,
                    (
                        "feedforward.velocity_gain_nm_per_rad_s",
                        "feedforward.acceleration_gain_nm_per_rad_s2",
                        "feedforward.gravity_compensation",
                        "feedforward.joint_stiffness_compensation",
                        "feedforward.friction_compensation",
                        "model.finger_mass_kg",
                        "model.finger_com_length_m",
                        "model.finger_coulomb_friction_nm",
                        "model.finger_joint_stiffness_nm_per_rad",
                        "transmission.gear_ratio",
                        "transmission.spool_radius_m",
                        "transmission.tendon_moment_arm_m",
                        "motor.torque_constant_nm_per_a",
                        "safety.max_current_a",
                        "safety.max_torque_nm",
                    ),
                ),
            ),
        ],
    ),
]


TEST_COMMON_SPECS = specs(
    TEST_DEFAULTS,
    (
        "duration_s",
        "start_delay_s",
        "publish_rate_hz",
    ),
)

TEST_PARAMETER_SPECS: Dict[str, List[ParamSpec]] = {
    "step": specs(TEST_DEFAULTS, ("step.target_rad", "step.hold_s")),
    "backlash": specs(
        TEST_DEFAULTS,
        ("backlash.amplitude_rad", "backlash.cycles", "backlash.period_s"),
    ),
    "friction": specs(TEST_DEFAULTS, ("friction.amplitude_rad", "friction.period_s")),
    "sine": specs(
        TEST_DEFAULTS,
        ("sine.amplitude_rad", "sine.bias_rad", "sine.frequency_hz"),
    ),
    "chirp": specs(
        TEST_DEFAULTS,
        (
            "chirp.amplitude_rad",
            "chirp.bias_rad",
            "chirp.start_frequency_hz",
            "chirp.end_frequency_hz",
        ),
    ),
    "load": specs(TEST_DEFAULTS, ("load.target_rad", "load.external_torque_nm")),
    "thermal": specs(
        TEST_DEFAULTS,
        ("thermal.amplitude_rad", "thermal.bias_rad", "thermal.frequency_hz"),
    ),
}


class ParameterGuiRosNode(Node):
    """ROS side of the control panel.

    Tk callbacks enqueue work into this node; the ROS timer processes the queue
    from the executor thread and sends results back to Tk through status_queue.
    """

    def __init__(self) -> None:
        super().__init__("parameter_gui_node")
        self.action_queue: "queue.Queue[Tuple[Any, ...]]" = queue.Queue()
        self.status_queue: "queue.Queue[Tuple[str, Any]]" = queue.Queue()
        self._parameter_clients: Dict[Tuple[str, str], Any] = {}

        self.position_pub = self.create_publisher(Float64, "/finger/command_position", 10)
        self.velocity_pub = self.create_publisher(Float64, "/finger/command_velocity", 10)
        self.torque_pub = self.create_publisher(Float64, "/finger/command_torque", 10)
        self.test_pub = self.create_publisher(String, "/finger/test_command", 10)

        self.create_subscription(DiagnosticArray, "/finger/safety_status", self._safety_status, 10)
        self.create_subscription(JointState, "/finger/sensor_state", self._sensor_state, 10)
        self.create_subscription(JointState, "/finger/controller_state", self._controller_state, 10)

        self.create_timer(0.05, self._process_actions)

    def _process_actions(self) -> None:
        for _ in range(20):
            try:
                action = self.action_queue.get_nowait()
            except queue.Empty:
                return

            kind = action[0]
            if kind == "set_parameters":
                _, target, updates = action
                self._request_set_parameters(target, updates)
            elif kind == "get_parameters":
                _, target, names, request_id = action
                self._request_get_parameters(target, names, request_id)
            elif kind == "set_params_then_start":
                _, target, updates, mode = action
                self._set_params_then_start(target, updates, mode)
            elif kind == "start_test":
                _, mode = action
                self._publish_test_command({"start_test": mode})
                self._post_status(f"Started {mode} test.")
            elif kind == "stop_test":
                self._publish_test_command({"stop_test": True})
                self._post_status("Requested test stop.")
            elif kind == "reset":
                self._publish_test_command({"reset": True})
                self._post_status("Requested plant reset.")
            elif kind == "command":
                _, command_type, value = action
                self._publish_command(command_type, value)

    def _client(self, target: str, service: str) -> Any:
        key = (target, service)
        if key in self._parameter_clients:
            return self._parameter_clients[key]
        srv_type = SetParameters if service == "set" else GetParameters
        client = self.create_client(srv_type, f"{target}/{service}_parameters")
        self._parameter_clients[key] = client
        return client

    def _request_set_parameters(self, target: str, updates: Dict[str, Any]) -> None:
        client = self._client(target, "set")
        if not client.service_is_ready():
            self._post_status(f"{target} parameter service is not ready.")
            return

        request = SetParameters.Request()
        request.parameters = [
            Parameter(name, value=value).to_parameter_msg() for name, value in updates.items()
        ]
        future = client.call_async(request)
        future.add_done_callback(lambda fut, node=target: self._set_done(fut, node))

    def _set_params_then_start(self, target: str, updates: Dict[str, Any], mode: str) -> None:
        client = self._client(target, "set")
        if not client.service_is_ready():
            self._post_status(f"{target} parameter service is not ready.")
            return

        request = SetParameters.Request()
        request.parameters = [
            Parameter(name, value=value).to_parameter_msg() for name, value in updates.items()
        ]
        future = client.call_async(request)
        future.add_done_callback(
            lambda fut, node=target, selected=mode: self._set_done_then_start(fut, node, selected)
        )

    def _request_get_parameters(self, target: str, names: Sequence[str], request_id: str) -> None:
        client = self._client(target, "get")
        if not client.service_is_ready():
            self._post_status(f"{target} parameter service is not ready.")
            return

        request = GetParameters.Request()
        request.names = list(names)
        future = client.call_async(request)
        future.add_done_callback(
            lambda fut, node=target, names=list(names), rid=request_id: self._get_done(
                fut, node, names, rid
            )
        )

    def _set_done(self, future: Any, target: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._post_status(f"Failed to update {target}: {exc}")
            return
        failures = [result.reason for result in response.results if not result.successful]
        if failures:
            self._post_status(f"{target} rejected update: {'; '.join(failures)}")
        else:
            self._post_status(f"Updated {target} parameters.")

    def _set_done_then_start(self, future: Any, target: str, mode: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._post_status(f"Failed to update {target}: {exc}")
            return
        failures = [result.reason for result in response.results if not result.successful]
        if failures:
            self._post_status(f"{target} rejected update: {'; '.join(failures)}")
            return
        self._publish_test_command({"start_test": mode})
        self._post_status(f"Applied {mode} parameters and started test.")

    def _get_done(
        self,
        future: Any,
        target: str,
        names: Sequence[str],
        request_id: str,
    ) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._post_status(f"Failed to read {target}: {exc}")
            return
        values = {
            name: parameter_value_to_python(value)
            for name, value in zip(names, response.values)
        }
        self.status_queue.put(("parameters", (request_id, values)))

    def _publish_command(self, command_type: str, value: float) -> None:
        msg = Float64()
        msg.data = value
        if command_type == "position":
            self.position_pub.publish(msg)
            self._post_status(f"Published position command {value:.4g} rad.")
        elif command_type == "velocity":
            self.velocity_pub.publish(msg)
            self._post_status(f"Published velocity command {value:.4g} rad/s.")
        elif command_type == "torque":
            self.torque_pub.publish(msg)
            self._post_status(f"Published direct torque command {value:.4g} N m.")

    def _publish_test_command(self, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self.test_pub.publish(msg)

    def _safety_status(self, msg: DiagnosticArray) -> None:
        if msg.status:
            self.status_queue.put(("safety", msg.status[0].message))

    def _sensor_state(self, msg: JointState) -> None:
        self.status_queue.put(
            (
                "sensor",
                {
                    "motor": joint_value(msg, "motor_encoder", "position", 0.0),
                    "output": joint_value(msg, "output_sensor", "position", 0.0),
                    "tension": joint_value(msg, "cable_tension", "position", 0.0),
                    "temperature": joint_value(msg, "temperature", "position", 0.0),
                    "load": joint_value(msg, "external_load", "position", 0.0),
                },
            )
        )

    def _controller_state(self, msg: JointState) -> None:
        self.status_queue.put(
            (
                "controller",
                {
                    "target": joint_value(msg, "target_position", "position", 0.0),
                    "error": joint_value(msg, "position_error", "position", 0.0),
                    "torque": joint_value(msg, "torque_command", "position", 0.0),
                    "current": joint_value(msg, "current_command", "position", 0.0),
                },
            )
        )

    def _post_status(self, text: str) -> None:
        self.status_queue.put(("status", text))


class ScrollableFrame(ttk.Frame):  # type: ignore[misc]
    def __init__(self, parent: Any) -> None:
        super().__init__(parent)
        self.canvas = tk.Canvas(self, borderwidth=0, highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.body = ttk.Frame(self.canvas)
        self.window_id = self.canvas.create_window((0, 0), window=self.body, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.scrollbar.grid(row=0, column=1, sticky="ns")
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)

        self.body.bind("<Configure>", self._on_body_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

    def _on_body_configure(self, _event: Any) -> None:
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event: Any) -> None:
        self.canvas.itemconfigure(self.window_id, width=event.width)


class ParameterSection:
    def __init__(
        self,
        parent: Any,
        title: str,
        target: str,
        param_specs: Sequence[ParamSpec],
        app: "ControlPanelApp",
    ) -> None:
        self.title = title
        self.target = target
        self.specs = list(param_specs)
        self.app = app
        self.variables: Dict[str, Any] = {}

        self.frame = ttk.LabelFrame(parent, text=title, padding=10)
        self.frame.columnconfigure(1, weight=1)

        for row, spec in enumerate(self.specs):
            ttk.Label(self.frame, text=spec.label).grid(row=row, column=0, sticky="w", pady=2)
            self.variables[spec.name] = self._make_variable(spec)
            widget = self._make_widget(spec)
            widget.grid(row=row, column=1, sticky="ew", padx=(8, 0), pady=2)

        buttons = ttk.Frame(self.frame)
        buttons.grid(row=len(self.specs), column=0, columnspan=2, sticky="e", pady=(8, 0))
        ttk.Button(buttons, text="Refresh", command=self.refresh).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(buttons, text="Apply", command=self.apply).grid(row=0, column=1)

    def _make_variable(self, spec: ParamSpec) -> Any:
        if isinstance(spec.default, bool):
            return tk.BooleanVar(value=bool(spec.default))
        return tk.StringVar(value=str(spec.default))

    def _make_widget(self, spec: ParamSpec) -> Any:
        variable = self.variables[spec.name]
        if isinstance(spec.default, bool):
            return ttk.Checkbutton(self.frame, variable=variable)
        if spec.choices:
            return ttk.Combobox(
                self.frame,
                textvariable=variable,
                values=list(spec.choices),
                state="readonly",
            )
        return ttk.Entry(self.frame, textvariable=variable)

    def grid(self, row: int) -> None:
        self.frame.grid(row=row, column=0, sticky="ew", padx=10, pady=8)

    def apply(self) -> None:
        updates = self.collect_updates()
        if updates is None:
            return
        self.app.node.action_queue.put(("set_parameters", self.target, updates))

    def refresh(self) -> None:
        request_id = self.app.register_request(self)
        names = [spec.name for spec in self.specs]
        self.app.node.action_queue.put(("get_parameters", self.target, names, request_id))

    def collect_updates(self) -> Optional[Dict[str, Any]]:
        updates: Dict[str, Any] = {}
        for spec in self.specs:
            try:
                updates[spec.name] = self._value_for(spec)
            except ValueError as exc:
                self.app.set_status(f"{self.title}: {exc}")
                return None
        return updates

    def _value_for(self, spec: ParamSpec) -> Any:
        variable = self.variables[spec.name]
        if isinstance(spec.default, bool):
            return bool(variable.get())
        text = str(variable.get()).strip()
        if isinstance(spec.default, int) and not isinstance(spec.default, bool):
            return int(text)
        if isinstance(spec.default, float):
            return float(text)
        return text

    def update_values(self, values: Dict[str, Any]) -> None:
        for spec in self.specs:
            if spec.name not in values:
                continue
            value = values[spec.name]
            variable = self.variables[spec.name]
            if isinstance(spec.default, bool):
                variable.set(bool(value))
            else:
                variable.set(str(value))


class ControlPanelApp:
    def __init__(self, root: Any, node: ParameterGuiRosNode) -> None:
        self.root = root
        self.node = node
        self.pending_requests: Dict[str, ParameterSection] = {}
        self.request_counter = 0
        self.sections: List[ParameterSection] = []
        self.test_common_section: Optional[ParameterSection] = None
        self.test_mode_sections: Dict[str, ParameterSection] = {}

        root.title("Tendon Finger Testbench Controls")
        root.minsize(860, 640)

        self.status_var = tk.StringVar(value="GUI ready.")
        self.safety_var = tk.StringVar(value="Safety: waiting for plant...")
        self.live_var = tk.StringVar(value="Live state: waiting for topics...")
        self.controller_var = tk.StringVar(value="Controller: waiting for topics...")
        self.mode_var = tk.StringVar(value=TEST_MODES[0])
        self.position_command = tk.StringVar(value="0.0")
        self.velocity_command = tk.StringVar(value="0.0")
        self.torque_command = tk.StringVar(value="0.0")

        self._build()
        self.root.after(500, self.refresh_all)
        self.root.after(100, self._poll_status)

    def _build(self) -> None:
        notebook = ttk.Notebook(self.root)
        notebook.grid(row=0, column=0, sticky="nsew")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        self._build_tests_tab(notebook)
        for tab_name, section_defs in PARAMETER_TABS:
            scroll = ScrollableFrame(notebook)
            scroll.body.columnconfigure(0, weight=1)
            notebook.add(scroll, text=tab_name)
            for row, (title, target, param_specs) in enumerate(section_defs):
                section = ParameterSection(scroll.body, title, target, param_specs, self)
                section.grid(row)
                self.sections.append(section)

        footer = ttk.Frame(self.root, padding=(8, 4))
        footer.grid(row=1, column=0, sticky="ew")
        footer.columnconfigure(0, weight=1)
        ttk.Label(footer, textvariable=self.status_var).grid(row=0, column=0, sticky="w")
        ttk.Button(footer, text="Refresh All", command=self.refresh_all).grid(row=0, column=1)

    def _build_tests_tab(self, notebook: Any) -> None:
        scroll = ScrollableFrame(notebook)
        scroll.body.columnconfigure(0, weight=1)
        notebook.add(scroll, text="Tests")

        controls = ttk.LabelFrame(scroll.body, text="Test Control", padding=10)
        controls.grid(row=0, column=0, sticky="ew", padx=10, pady=8)
        controls.columnconfigure(1, weight=1)
        ttk.Label(controls, text="Selected test").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            controls,
            textvariable=self.mode_var,
            values=list(TEST_MODES),
            state="readonly",
        ).grid(row=0, column=1, sticky="ew", padx=(8, 0))
        ttk.Button(controls, text="Start", command=self.start_selected_test).grid(
            row=0, column=2, padx=(8, 0)
        )
        ttk.Button(controls, text="Stop", command=self.stop_test).grid(row=0, column=3, padx=(8, 0))
        ttk.Button(controls, text="Reset System", command=self.reset_plant).grid(
            row=0, column=4, padx=(8, 0)
        )
        ttk.Label(controls, textvariable=self.safety_var).grid(
            row=1, column=0, columnspan=5, sticky="w", pady=(8, 0)
        )
        ttk.Label(controls, textvariable=self.live_var).grid(
            row=2, column=0, columnspan=5, sticky="w", pady=(4, 0)
        )
        ttk.Label(controls, textvariable=self.controller_var).grid(
            row=3, column=0, columnspan=5, sticky="w", pady=(4, 0)
        )

        manual = ttk.LabelFrame(scroll.body, text="Manual Commands", padding=10)
        manual.grid(row=1, column=0, sticky="ew", padx=10, pady=8)
        for column in range(3):
            manual.columnconfigure(column * 2 + 1, weight=1)
        self._command_row(manual, 0, "Position [rad]", self.position_command, "position")
        self._command_row(manual, 1, "Velocity [rad/s]", self.velocity_command, "velocity")
        self._command_row(manual, 2, "Direct torque [N m]", self.torque_command, "torque")

        common = ParameterSection(scroll.body, "Common Test Parameters", TEST_NODE, TEST_COMMON_SPECS, self)
        common.grid(2)
        self.sections.append(common)
        self.test_common_section = common

        test_notebook = ttk.Notebook(scroll.body)
        test_notebook.grid(row=3, column=0, sticky="nsew", padx=10, pady=8)
        for mode in TEST_MODES:
            tab = ttk.Frame(test_notebook, padding=8)
            tab.columnconfigure(0, weight=1)
            test_notebook.add(tab, text=mode.title())
            section = ParameterSection(
                tab,
                f"{mode.title()} Parameters",
                TEST_NODE,
                TEST_PARAMETER_SPECS[mode],
                self,
            )
            section.grid(0)
            self.sections.append(section)
            self.test_mode_sections[mode] = section

    def _command_row(
        self,
        parent: Any,
        row: int,
        label: str,
        variable: Any,
        command_type: str,
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        ttk.Entry(parent, textvariable=variable).grid(row=row, column=1, sticky="ew", padx=(8, 8))
        ttk.Button(
            parent,
            text="Publish",
            command=lambda kind=command_type, var=variable: self.publish_command(kind, var),
        ).grid(row=row, column=2, sticky="e", pady=2)

    def start_selected_test(self) -> None:
        mode = self.mode_var.get()
        updates: Dict[str, Any] = {"mode": mode}
        for section in (self.test_common_section, self.test_mode_sections.get(mode)):
            if section is None:
                continue
            section_updates = section.collect_updates()
            if section_updates is None:
                return
            updates.update(section_updates)
        self.node.action_queue.put(("set_params_then_start", TEST_NODE, updates, mode))

    def stop_test(self) -> None:
        self.node.action_queue.put(("stop_test",))

    def reset_plant(self) -> None:
        self.node.action_queue.put(("reset",))

    def publish_command(self, command_type: str, variable: Any) -> None:
        try:
            value = float(variable.get())
        except ValueError:
            self.set_status(f"Invalid {command_type} command value.")
            return
        self.node.action_queue.put(("command", command_type, value))

    def register_request(self, section: ParameterSection) -> str:
        self.request_counter += 1
        request_id = str(self.request_counter)
        self.pending_requests[request_id] = section
        return request_id

    def refresh_all(self) -> None:
        for section in self.sections:
            section.refresh()

    def set_status(self, text: str) -> None:
        self.status_var.set(text)

    def _poll_status(self) -> None:
        try:
            while True:
                kind, payload = self.node.status_queue.get_nowait()
                if kind == "status":
                    self.status_var.set(str(payload))
                elif kind == "parameters":
                    request_id, values = payload
                    section = self.pending_requests.pop(request_id, None)
                    if section is not None:
                        section.update_values(values)
                elif kind == "safety":
                    self.safety_var.set(f"Safety: {payload}")
                elif kind == "sensor":
                    self._update_live_sensor(payload)
                elif kind == "controller":
                    self._update_live_controller(payload)
        except queue.Empty:
            pass
        self.root.after(100, self._poll_status)

    def _update_live_sensor(self, values: Dict[str, float]) -> None:
        self.live_var.set(
            "Live state: "
            f"output {values['output']:.3f} rad, "
            f"motor {values['motor']:.3f} rad, "
            f"tension {values['tension']:.2f} N, "
            f"temp {values['temperature']:.1f} C, "
            f"load {values['load']:.3f} N m"
        )

    def _update_live_controller(self, values: Dict[str, float]) -> None:
        self.controller_var.set(
            "Controller: "
            f"target {values['target']:.3f} rad, "
            f"error {values['error']:.3f} rad, "
            f"torque {values['torque']:.3f} N m, "
            f"current {values['current']:.3f} A"
        )


def _spin(node: ParameterGuiRosNode) -> None:
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = ParameterGuiRosNode()

    if tk is None or ttk is None:
        node.get_logger().error("Tkinter is not available; parameter GUI cannot start.")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        node.get_logger().error(f"Tkinter display unavailable; parameter GUI not started: {exc}")
        node.destroy_node()
        rclpy.shutdown()
        return

    spin_thread = threading.Thread(target=_spin, args=(node,), daemon=True)
    spin_thread.start()

    def on_close() -> None:
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    ControlPanelApp(root, node)

    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
