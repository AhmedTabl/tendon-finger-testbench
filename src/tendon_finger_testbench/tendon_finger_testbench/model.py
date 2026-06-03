"""Shared dynamics, filtering, and controller utilities.

The first-pass plant intentionally keeps the tendon transmission in Python so
it is easy to inspect and extend. MuJoCo can be used by the ROS node for scene
loading/visualization, while this module remains dependency-light and unit
testable without ROS 2 or MuJoCo installed.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import random
from typing import Any, Dict, Iterable, Mapping, MutableMapping, Optional, Tuple


EPS = 1.0e-9


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def finite_or(value: float, fallback: float = 0.0) -> float:
    return value if math.isfinite(value) else fallback


def smooth_sign(value: float, width: float) -> float:
    """A continuous sign approximation used for Coulomb friction."""

    width = max(width, EPS)
    return math.tanh(value / width)


def backlash_filter(command: float, transmitted: float, deadband: float) -> float:
    """Lost-motion/backlash filter.

    ``transmitted`` only moves when ``command`` gets more than half the
    configured deadband away from it. This produces a simple, directional
    deadband suitable for reversal tests.
    """

    deadband = max(0.0, deadband)
    if deadband <= EPS:
        return command

    half_gap = 0.5 * deadband
    error = command - transmitted
    if error > half_gap:
        return command - half_gap
    if error < -half_gap:
        return command + half_gap
    return transmitted


def quantize_angle(angle: float, counts_per_rev: float) -> float:
    if counts_per_rev <= 0:
        return angle
    step = 2.0 * math.pi / counts_per_rev
    return round(angle / step) * step


class FixedStepDelay:
    """Small delay line for fixed-rate simulation loops."""

    def __init__(self) -> None:
        self._samples: deque[Any] = deque()

    def reset(self) -> None:
        self._samples.clear()

    def push(self, value: Any, delay_s: float, dt: float) -> Any:
        steps = max(0, int(round(max(0.0, delay_s) / max(dt, EPS))))
        self._samples.append(value)
        while len(self._samples) > steps + 1:
            self._samples.popleft()
        if steps == 0:
            return value
        return self._samples[0]


@dataclass
class PIDState:
    integral: float = 0.0
    previous_error: Optional[float] = None


class PID:
    """PID/PI helper with clamped integrator and output saturation."""

    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        integral_limit: float = 0.0,
        output_limit: float = 0.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.output_limit = abs(output_limit)
        self.state = PIDState()

    def configure(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float,
        output_limit: float,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.output_limit = abs(output_limit)

    def reset(self) -> None:
        self.state = PIDState()

    def update(self, error: float, dt: float, derivative: Optional[float] = None) -> float:
        dt = max(dt, EPS)

        if derivative is None:
            if self.state.previous_error is None:
                d_error = 0.0
            else:
                d_error = (error - self.state.previous_error) / dt
        else:
            d_error = derivative

        self.state.integral += error * dt
        if self.integral_limit > 0.0:
            self.state.integral = clamp(
                self.state.integral, -self.integral_limit, self.integral_limit
            )

        raw_output = self.kp * error + self.ki * self.state.integral + self.kd * d_error
        output = raw_output
        if self.output_limit > 0.0:
            output = clamp(raw_output, -self.output_limit, self.output_limit)

            # Back-calculate the clamped integrator so it cannot keep winding up
            # beyond the saturated command.
            if self.ki > EPS and output != raw_output:
                proportional_derivative = self.kp * error + self.kd * d_error
                self.state.integral = (output - proportional_derivative) / self.ki
                if self.integral_limit > 0.0:
                    self.state.integral = clamp(
                        self.state.integral,
                        -self.integral_limit,
                        self.integral_limit,
                    )

        self.state.previous_error = error
        return output


@dataclass
class PlantState:
    motor_position: float = 0.0
    motor_velocity: float = 0.0
    finger_position: float = 0.0
    finger_velocity: float = 0.0
    transmitted_spool_angle: float = 0.0
    tendon_drive_angle: float = 0.0
    cable_tension: float = 0.0
    applied_torque: float = 0.0
    current: float = 0.0
    temperature: float = 25.0
    external_load: float = 0.0


class TendonFingerPlant:
    """Physically grounded first-pass motor/spool/tendon/finger model."""

    def __init__(self, params: Optional[Mapping[str, Any]] = None) -> None:
        self.params: Dict[str, Any] = {}
        self.state = PlantState()
        self.command_delay = FixedStepDelay()
        self.sensor_delay = FixedStepDelay()
        self._previous_stretch = 0.0
        self._rng = random.Random(7)
        self.update_params(params or {})
        self.reset()

    def update_params(self, params: Mapping[str, Any]) -> None:
        self.params.update(dict(params))
        seed = int(self._p("sensors.random_seed", 7))
        self._rng.seed(seed)

    def reset(self) -> None:
        ambient = self._p("thermal.ambient_temperature_c", 25.0)
        self.state = PlantState(temperature=ambient, external_load=self._p("load.external_torque", 0.0))
        self.command_delay.reset()
        self.sensor_delay.reset()
        self._previous_stretch = 0.0

    def set_external_load(self, torque_nm: float) -> None:
        self.state.external_load = finite_or(torque_nm)

    def _p(self, name: str, default: float) -> float:
        value = self.params.get(name, default)
        if isinstance(value, bool):
            return float(value)
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def _bool(self, name: str, default: bool) -> bool:
        value = self.params.get(name, default)
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _friction_accel(
        self,
        net_torque_without_friction: float,
        velocity: float,
        inertia: float,
        coulomb: float,
        static: float,
        stiction_velocity: float,
    ) -> Tuple[float, float]:
        inertia = max(inertia, EPS)
        coulomb = max(0.0, coulomb)
        static = max(coulomb, static)
        stiction_velocity = max(stiction_velocity, EPS)

        if abs(velocity) < stiction_velocity and abs(net_torque_without_friction) <= static:
            return 0.0, net_torque_without_friction

        friction_direction = velocity
        if abs(friction_direction) < stiction_velocity:
            friction_direction = net_torque_without_friction
        friction = coulomb * smooth_sign(friction_direction, stiction_velocity)
        return (net_torque_without_friction - friction) / inertia, friction

    def step(self, commanded_motor_torque: float, dt: float) -> Dict[str, float]:
        dt = clamp(dt, 1.0e-5, 0.05)
        s = self.state
        events = []

        if not math.isfinite(commanded_motor_torque):
            commanded_motor_torque = 0.0
            events.append("invalid torque command replaced with zero")

        delayed_command = self.command_delay.push(
            commanded_motor_torque, self._p("delay.command_delay_s", 0.0), dt
        )

        kt = max(self._p("motor.torque_constant_nm_per_a", 0.11), EPS)
        current_limit = max(0.0, self._p("safety.max_current_a", 2.5))
        torque_limit = min(
            max(0.0, self._p("safety.max_torque_nm", 0.35)),
            kt * current_limit,
        )

        if s.temperature >= self._p("safety.max_temperature_c", 85.0):
            delayed_command = 0.0
            events.append("temperature limit exceeded; torque disabled")

        saturated_command = clamp(delayed_command, -torque_limit, torque_limit)
        if saturated_command != delayed_command:
            events.append("torque/current command saturated")

        lag_tau = max(0.0, self._p("actuator.torque_lag_time_constant_s", 0.015))
        if lag_tau > dt:
            alpha = clamp(dt / lag_tau, 0.0, 1.0)
            s.applied_torque += alpha * (saturated_command - s.applied_torque)
        else:
            s.applied_torque = saturated_command

        s.current = s.applied_torque / kt

        gear_ratio = max(self._p("transmission.gear_ratio", 1.0), EPS)
        spool_radius = max(self._p("transmission.spool_radius_m", 0.012), EPS)
        tendon_moment_arm = max(self._p("transmission.tendon_moment_arm_m", 0.018), EPS)
        finger_length = max(self._p("finger.length_m", 0.09), EPS)

        spool_angle = s.motor_position / gear_ratio
        motor_deadband = self._p("backlash.motor_deadband_rad", 0.0)
        s.transmitted_spool_angle = backlash_filter(
            spool_angle, s.transmitted_spool_angle, motor_deadband
        )

        ideal_tendon_drive_angle = (s.transmitted_spool_angle * spool_radius) / tendon_moment_arm
        output_deadband_angle = (
            self._p("backlash.output_deadband_mm", 0.0) * 0.001 / finger_length
        )
        s.tendon_drive_angle = backlash_filter(
            ideal_tendon_drive_angle, s.tendon_drive_angle, output_deadband_angle
        )

        tendon_displacement = s.tendon_drive_angle * tendon_moment_arm
        finger_tendon_displacement = s.finger_position * tendon_moment_arm
        slack = max(0.0, self._p("tendon.slack_m", 0.001))
        stretch = tendon_displacement - finger_tendon_displacement - slack
        stretch_rate = (stretch - self._previous_stretch) / dt
        self._previous_stretch = stretch

        if stretch > 0.0:
            tension = (
                self._p("tendon.stiffness_n_per_m", 850.0) * stretch
                + self._p("tendon.damping_n_s_per_m", 1.5) * stretch_rate
            )
            s.cable_tension = max(0.0, tension)
        else:
            s.cable_tension = 0.0

        tendon_torque_motor = s.cable_tension * spool_radius / gear_ratio
        tendon_torque_finger = s.cable_tension * tendon_moment_arm

        motor_net = (
            s.applied_torque
            - tendon_torque_motor
            - self._p("motor.viscous_damping_nm_s_per_rad", 0.0025) * s.motor_velocity
            - self._p("motor.stiffness_nm_per_rad", 0.0) * s.motor_position
        )
        motor_accel, motor_friction = self._friction_accel(
            motor_net,
            s.motor_velocity,
            self._p("motor.inertia_kg_m2", 0.0018),
            self._p("motor.coulomb_friction_nm", 0.006),
            self._p("motor.static_friction_nm", 0.011),
            self._p("friction.stiction_velocity_rad_s", 0.025),
        )

        gravity_torque = 0.0
        if self._bool("finger.enable_gravity", True):
            gravity_torque = (
                self._p("finger.mass_kg", 0.055)
                * 9.80665
                * self._p("finger.com_length_m", 0.045)
                * math.sin(s.finger_position)
            )

        finger_net = (
            tendon_torque_finger
            - self._p("finger.viscous_damping_nm_s_per_rad", 0.008) * s.finger_velocity
            - self._p("finger.joint_stiffness_nm_per_rad", 0.015) * s.finger_position
            - gravity_torque
            - s.external_load
        )
        finger_accel, finger_friction = self._friction_accel(
            finger_net,
            s.finger_velocity,
            self._p("finger.inertia_kg_m2", 0.0022),
            self._p("finger.coulomb_friction_nm", 0.012),
            self._p("finger.static_friction_nm", 0.018),
            self._p("friction.stiction_velocity_rad_s", 0.025),
        )

        s.motor_velocity += motor_accel * dt
        s.motor_position += s.motor_velocity * dt
        s.finger_velocity += finger_accel * dt
        s.finger_position += s.finger_velocity * dt

        max_velocity = self._p("safety.max_velocity_rad_s", 18.0)
        if abs(s.motor_velocity) > max_velocity:
            s.motor_velocity = clamp(s.motor_velocity, -max_velocity, max_velocity)
            events.append("motor velocity clamped")
        if abs(s.finger_velocity) > max_velocity:
            s.finger_velocity = clamp(s.finger_velocity, -max_velocity, max_velocity)
            events.append("finger velocity clamped")

        lower = self._p("safety.min_position_rad", -0.05)
        upper = self._p("safety.max_position_rad", 1.35)
        if s.finger_position < lower:
            s.finger_position = lower
            s.finger_velocity = max(0.0, s.finger_velocity)
            events.append("lower joint limit reached")
        if s.finger_position > upper:
            s.finger_position = upper
            s.finger_velocity = min(0.0, s.finger_velocity)
            events.append("upper joint limit reached")

        ambient = self._p("thermal.ambient_temperature_c", 25.0)
        heating = self._p("thermal.heating_coeff_c_per_a2_s", 0.09) * (s.current * s.current)
        cooling = self._p("thermal.cooling_coeff_per_s", 0.025) * (s.temperature - ambient)
        s.temperature += (heating - cooling) * dt
        if s.temperature >= self._p("safety.max_temperature_c", 85.0):
            events.append("temperature limit exceeded")

        if not all(
            math.isfinite(value)
            for value in (
                s.motor_position,
                s.motor_velocity,
                s.finger_position,
                s.finger_velocity,
                s.cable_tension,
                s.temperature,
            )
        ):
            events.append("invalid state detected; plant reset")
            self.reset()

        raw_sensor = self._make_sensor_sample()
        delayed_sensor = self.sensor_delay.push(
            raw_sensor, self._p("delay.measurement_delay_s", 0.0), dt
        )

        safe = not events
        return {
            "safe": 1.0 if safe else 0.0,
            "events": "; ".join(dict.fromkeys(events)),
            "motor_position": s.motor_position,
            "motor_velocity": s.motor_velocity,
            "finger_position": s.finger_position,
            "finger_velocity": s.finger_velocity,
            "transmitted_spool_angle": s.transmitted_spool_angle,
            "tendon_drive_angle": s.tendon_drive_angle,
            "cable_tension": s.cable_tension,
            "tendon_torque_motor": tendon_torque_motor,
            "tendon_torque_finger": tendon_torque_finger,
            "motor_friction_torque": motor_friction,
            "finger_friction_torque": finger_friction,
            "gravity_torque": gravity_torque,
            "commanded_torque": commanded_motor_torque,
            "delayed_commanded_torque": delayed_command,
            "applied_torque": s.applied_torque,
            "current": s.current,
            "external_load": s.external_load,
            "temperature": s.temperature,
            "measured_motor_position": delayed_sensor["measured_motor_position"],
            "measured_motor_velocity": delayed_sensor["measured_motor_velocity"],
            "measured_output_position": delayed_sensor["measured_output_position"],
            "measured_output_velocity": delayed_sensor["measured_output_velocity"],
        }

    def _make_sensor_sample(self) -> Dict[str, float]:
        s = self.state
        encoder_counts = self._p("sensors.encoder_counts_per_rev", 4096.0)
        motor_position = quantize_angle(s.motor_position, encoder_counts)
        motor_position += self._rng.gauss(0.0, max(0.0, self._p("sensors.encoder_noise_std_rad", 0.0)))

        motor_velocity = s.motor_velocity + self._rng.gauss(
            0.0, max(0.0, self._p("sensors.encoder_velocity_noise_std_rad_s", 0.0))
        )

        if self._bool("sensors.output_sensor_enabled", True):
            output_position = s.finger_position + self._rng.gauss(
                0.0, max(0.0, self._p("sensors.output_sensor_noise_std_rad", 0.0))
            )
            output_velocity = s.finger_velocity + self._rng.gauss(
                0.0, max(0.0, self._p("sensors.output_sensor_velocity_noise_std_rad_s", 0.0))
            )
        else:
            output_position = float("nan")
            output_velocity = float("nan")

        return {
            "measured_motor_position": motor_position,
            "measured_motor_velocity": motor_velocity,
            "measured_output_position": output_position,
            "measured_output_velocity": output_velocity,
        }


def flatten_ros_parameters(prefix: str, value: Any) -> Dict[str, Any]:
    """Convert nested YAML dictionaries into ROS-style dotted parameters."""

    if not isinstance(value, Mapping):
        return {prefix: value}
    flattened: Dict[str, Any] = {}
    for key, child in value.items():
        child_prefix = f"{prefix}.{key}" if prefix else str(key)
        flattened.update(flatten_ros_parameters(child_prefix, child))
    return flattened


def merge_parameter_dicts(*dicts: Mapping[str, Any]) -> Dict[str, Any]:
    merged: Dict[str, Any] = {}
    for mapping in dicts:
        for key, value in mapping.items():
            if isinstance(value, Mapping):
                merged.update(flatten_ros_parameters(str(key), value))
            else:
                merged[str(key)] = value
    return merged

