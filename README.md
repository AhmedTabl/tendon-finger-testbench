# tendon_finger_testbench_ws

ROS 2 Humble workspace for `tendon_finger_testbench`, a simulation-first
tendon-driven robotic finger actuator characterization project.

Start here:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch tendon_finger_testbench sim.launch.py
```

See [src/tendon_finger_testbench/README.md](src/tendon_finger_testbench/README.md)
for the full project guide.

