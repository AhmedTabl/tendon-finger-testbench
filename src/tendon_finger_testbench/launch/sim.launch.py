"""Launch the simulated tendon finger plant, controller, logger, and tools."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("tendon_finger_testbench")
    default_params = os.path.join(package_share, "config", "default_params.yaml")

    params_file = LaunchConfiguration("params_file")
    use_cli = LaunchConfiguration("use_cli")
    use_gui = LaunchConfiguration("use_gui")
    use_logger = LaunchConfiguration("use_logger")
    use_test_executor = LaunchConfiguration("use_test_executor")
    test_mode = LaunchConfiguration("test_mode")
    auto_start_test = LaunchConfiguration("auto_start_test")
    mujoco_viewer = LaunchConfiguration("mujoco_viewer")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to YAML parameter file.",
            ),
            DeclareLaunchArgument(
                "use_cli",
                default_value="false",
                description="Launch the terminal command CLI.",
            ),
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Launch the Python parameter/test control GUI.",
            ),
            DeclareLaunchArgument(
                "use_logger",
                default_value="true",
                description="Launch the CSV data logger.",
            ),
            DeclareLaunchArgument(
                "use_test_executor",
                default_value="true",
                description="Launch automated test executor.",
            ),
            DeclareLaunchArgument(
                "test_mode",
                default_value="step",
                description="Test mode: step, backlash, friction, sine, chirp, load, thermal.",
            ),
            DeclareLaunchArgument(
                "auto_start_test",
                default_value="false",
                description="Start the selected test as soon as the test executor launches.",
            ),
            DeclareLaunchArgument(
                "mujoco_viewer",
                default_value="true",
                description="Open MuJoCo passive viewer if mujoco is installed.",
            ),
            Node(
                package="tendon_finger_testbench",
                executable="plant_sim_node",
                name="plant_sim_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "mujoco.launch_viewer": ParameterValue(mujoco_viewer, value_type=bool),
                    },
                ],
            ),
            Node(
                package="tendon_finger_testbench",
                executable="controller_node",
                name="controller_node",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="tendon_finger_testbench",
                executable="data_logger_node",
                name="data_logger_node",
                output="screen",
                condition=IfCondition(use_logger),
                parameters=[params_file],
            ),
            Node(
                package="tendon_finger_testbench",
                executable="test_executor_node",
                name="test_executor_node",
                output="screen",
                condition=IfCondition(use_test_executor),
                parameters=[
                    params_file,
                    {
                        "mode": test_mode,
                        "auto_start": ParameterValue(auto_start_test, value_type=bool),
                    },
                ],
            ),
            Node(
                package="tendon_finger_testbench",
                executable="command_cli_node",
                name="command_cli_node",
                output="screen",
                condition=IfCondition(use_cli),
                parameters=[params_file],
            ),
            Node(
                package="tendon_finger_testbench",
                executable="parameter_gui_node",
                name="parameter_gui_node",
                output="screen",
                condition=IfCondition(use_gui),
            ),
        ]
    )
