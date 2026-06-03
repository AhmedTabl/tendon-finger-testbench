from glob import glob
import os

from setuptools import find_packages, setup


package_name = "tendon_finger_testbench"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.py"))),
        (os.path.join("share", package_name, "mujoco"), glob(os.path.join("mujoco", "*.xml"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="Simulation-first ROS 2 testbench for a tendon-driven robotic finger actuator.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plant_sim_node = tendon_finger_testbench.plant_sim_node:main",
            "controller_node = tendon_finger_testbench.controller_node:main",
            "command_cli_node = tendon_finger_testbench.command_cli_node:main",
            "test_executor_node = tendon_finger_testbench.test_executor_node:main",
            "data_logger_node = tendon_finger_testbench.data_logger_node:main",
            "plot_test = tendon_finger_testbench.analysis.plot_test:main",
            "step_response_metrics = tendon_finger_testbench.analysis.step_response_metrics:main",
            "backlash_analysis = tendon_finger_testbench.analysis.backlash_analysis:main",
            "friction_fit = tendon_finger_testbench.analysis.friction_fit:main",
            "generate_summary_report = tendon_finger_testbench.analysis.generate_summary_report:main",
        ],
    },
)

