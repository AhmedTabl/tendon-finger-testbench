# tendon_finger_testbench

Simulation-first ROS 2 Humble mini-project for actuator characterization and
control of a simplified tendon-driven robotic finger.

The project is meant for practicing motor bring-up habits before touching real
hardware: cascaded control, parameter identification, backlash and friction
tests, stiffness/slack experiments, safety limits, logging, and offline analysis.

## System Overview

```text
command_position / command_velocity
              |
              v
    +-------------------+       command_torque        +----------------------+
    | cascaded control  | --------------------------> | MuJoCo plant         |
    | pos -> vel -> tau |                             | shaft/gearbox/spool  |
    +-------------------+                             | tendon/finger        |
              ^                                       +----------+-----------+
              |                                                  |
              | sensor_state                                     | true_state
              +--------------------------------------------------+

The MuJoCo model is both the visual scene and authoritative mechanical plant.
Python supplies ROS communication, sensor effects, delay, thermal state, safety,
and a compatibility fallback when MuJoCo is disabled.
```

## Model

The plant uses the standard single-axis form:

```text
J*q_ddot + b*q_dot + tau_c*sgn(q_dot) + k*q
    = tau_motor_effective - tau_load
```

The model is split into motor, spool, tendon, and finger elements:

- Motor axis: motor inertia, damping, friction, optional stiffness,
  torque/current saturation, actuator lag, and a shaft encoder.
- Transmission: an editable MuJoCo gearbox constraint maps motor angle to spool
  angle through `gear_ratio`. Spool inertia, damping, friction, and radius are
  editable independently.
- Tendon: spool displacement is `spool_radius * motor_angle / gear_ratio`.
  Finger tendon displacement is `tendon_moment_arm * finger_angle`.
- Cable tension is generated only after slack is taken up:

```text
stretch = tendon_drive_displacement - finger_tendon_displacement - slack
tension = max(0, tendon_stiffness*stretch + tendon_damping*stretch_rate)
```

- Finger axis: link inertia, damping, Coulomb/static friction, joint stiffness,
  gravity, external load torque, and hard stops.

Motor deadband changes the gearbox constraint compliance. Output/fingertip
deadband is converted into additional cable take-up distance with:

```text
extra_slack = output_deadband_mm * 0.001
    * tendon_moment_arm / finger_length
```

Sensors include motor encoder quantization, Gaussian encoder noise, optional
output sensor noise, and command/measurement delay.

## Workspace Layout

```text
tendon_finger_testbench_ws/
  src/tendon_finger_testbench/
    config/default_params.yaml
    launch/sim.launch.py
    mujoco/tendon_finger.xml
    tendon_finger_testbench/
      plant_sim_node.py
      controller_node.py
      command_cli_node.py
      test_executor_node.py
      data_logger_node.py
      analysis/
```

## Setup

Install ROS 2 Humble on Ubuntu 22.04, then from the workspace root:

```bash
source /opt/ros/humble/setup.bash
pip install -r src/tendon_finger_testbench/requirements.txt
colcon build --symlink-install
source install/setup.bash
```

`mujoco` is required for the detailed mechanical plant and viewer. When it is
disabled, the original Python model remains available as a compatibility
fallback. Without matplotlib, CSV analysis still works except plotting.

## Run The Simulation

Launch plant, controller, logger, MuJoCo viewer, Python control GUI, and an idle
test executor:

```bash
ros2 launch tendon_finger_testbench sim.launch.py
```

Launch headless:

```bash
ros2 launch tendon_finger_testbench sim.launch.py mujoco_viewer:=false use_gui:=false
```

Launch with the terminal CLI:

```bash
ros2 launch tendon_finger_testbench sim.launch.py use_cli:=true
```

Disable the Python control GUI:

```bash
ros2 launch tendon_finger_testbench sim.launch.py use_gui:=false
```

Send commands from another terminal:

```bash
ros2 topic pub --once /finger/command_position std_msgs/msg/Float64 "{data: 0.7}"
ros2 topic pub --once /finger/command_velocity std_msgs/msg/Float64 "{data: 0.0}"
```

The CLI accepts commands such as:

```text
pos 0.8
posdeg 45
motor 1.2
tip 25
vel 0.0
load 0.05
reset
```

Direct torque commands publish to `/finger/command_torque`. If the controller is
running, it also publishes this topic, so use direct torque mode with care.

## Python Control GUI

`parameter_gui_node` launches by default. It provides test selection, start/stop
controls, plant reset, per-test parameter editors, shared motor/actuator
parameters, plant/sensor/safety tuning, controller gains, and manual command
publishing.

## Topics

- `/finger/command_position` (`std_msgs/Float64`)
- `/finger/command_velocity` (`std_msgs/Float64`)
- `/finger/command_torque` (`std_msgs/Float64`)
- `/finger/controller_state` (`sensor_msgs/JointState`)
- `/finger/true_state` (`sensor_msgs/JointState`)
- `/finger/sensor_state` (`sensor_msgs/JointState`)
- `/finger/motor_encoder` (`std_msgs/Float64`)
- `/finger/output_sensor` (`std_msgs/Float64`)
- `/finger/test_command` (`std_msgs/String`, JSON payload)
- `/finger/safety_status` (`diagnostic_msgs/DiagnosticArray`)

## Automated Tests

Run the step-response test:

```bash
ros2 launch tendon_finger_testbench sim.launch.py \
  auto_start_test:=true test_mode:=step
```

Other modes:

```bash
test_mode:=backlash
test_mode:=friction
test_mode:=sine
test_mode:=chirp
test_mode:=load
test_mode:=thermal
```

You can also start tests through `/finger/test_command`:

```bash
ros2 topic pub --once /finger/test_command std_msgs/msg/String \
  "{data: '{\"start_test\": \"backlash\"}'}"
```

Implemented test profiles:

- Step response: target angle, rise time, overshoot, settling time,
  steady-state error, peak torque/current.
- Backlash/reversal: repeated positive/negative reversals and simple deadband
  estimate.
- Friction sweep: slow sinusoidal sweep for Coulomb/viscous fitting.
- Sine/chirp: sinusoidal and chirped trajectory tracking.
- Load/disturbance: applies external load torque via plant test command.
- Thermal/duty-cycle: repeated open/close motion with current-squared heating.

## Logging

`data_logger_node` writes timestamped CSV files under `data/` with:

```text
time, command_position, command_velocity, command_torque_or_current,
true_motor_position, true_motor_velocity, true_spool_position,
true_spool_velocity, measured_motor_encoder_position, true_finger_position,
true_finger_velocity, measured_output_position, tendon_length, tendon_velocity,
visual_cable_length, cable_tension, controller_error_position,
controller_error_velocity, torque_command, current_command,
applied_torque_after_saturation, external_load, temperature, safety_status
```

## Offline Analysis

After sourcing the workspace:

```bash
ros2 run tendon_finger_testbench plot_test data/testbench_*.csv
ros2 run tendon_finger_testbench step_response_metrics data/testbench_*.csv
ros2 run tendon_finger_testbench backlash_analysis data/testbench_*.csv
ros2 run tendon_finger_testbench friction_fit data/testbench_*.csv
ros2 run tendon_finger_testbench generate_summary_report data/testbench_*.csv
```

For shell globs, use the concrete CSV file path if your shell expands to more
than one log.

## Safety Layer

The plant clamps or reports:

- max current/torque
- max velocity
- joint hard stops
- temperature limit
- invalid commands or NaN state
- exploding/invalid simulation state

Safety state is published on `/finger/safety_status`.

## Parameter Tuning

Most important parameters live in `config/default_params.yaml`.

Good live-tuning candidates:

- shared motor and actuator parameters in the GUI Motor tab
- `plant_sim_node.transmission.gear_ratio`
- `plant_sim_node.transmission.spool_*`
- `controller_node.position.*`
- `controller_node.velocity.*`
- `plant_sim_node.tendon.stiffness_n_per_m`
- `plant_sim_node.tendon.damping_n_s_per_m`
- `plant_sim_node.backlash.*`
- `plant_sim_node.sensors.*noise*`
- `plant_sim_node.delay.*`
- `plant_sim_node.load.external_torque`
- `plant_sim_node.safety.max_current_a`

Restart recommended:

- update rates
- MuJoCo XML path/viewer settings
- model topology changes

Example:

```bash
ros2 param set /controller_node position.kp 12.0
ros2 param set /plant_sim_node backlash.motor_deadband_rad 0.04
ros2 param set /plant_sim_node tendon.stiffness_n_per_m 1200.0
```

## Known Limitations

- The generic 42 mm-class motor and fixture are custom primitive geometry, not
  exact CAD for a specific commercial part.
- The compliant tendon and visual pulley route are not a finite-element cable
  or strand/contact solver.
- Static friction uses a low-speed Stribeck approximation with generic,
  unmeasured defaults.
- No custom ROS messages are used, so `JointState` fields carry several scalar
  diagnostics by name.
- Direct torque mode and closed-loop controller mode share `/finger/command_torque`;
  avoid running both command sources during open-loop tests.
- Friction and backlash estimates are simple heuristics meant for practice and
  comparison, not final system identification.

## Future Improvements

- Add a `ros2_control` hardware interface and swap the plant for real motor
  driver/encoder IO.
- Add custom messages once the state schema stabilizes.
- Add calibration routines for encoder zeroing, tendon pretension, and joint
  limits.
- Add multi-link fingers and coupled tendon routing.
- Add richer frequency response identification for chirp/sine tests.

## Model References

No external meshes are copied into this package. The custom editable MJCF was
informed by the [MuJoCo modeling guide](https://mujoco.readthedocs.io/en/stable/modeling.html),
[XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html),
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie),
[Yale OpenHand hardware](https://github.com/grablab/openhand-hardware), and
[Tactile SoftHand-A CAD](https://github.com/HaoranLi-Data/Tactile_SoftHand_A).
