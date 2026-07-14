import os
from glob import glob
from setuptools import find_packages, setup

package_name = "climbingrobot_optimization"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # install launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="ALPINE port maintainer",
    maintainer_email="dev@example.com",
    description=(
        "Python (+ optional C++) port of the MATLAB MPC controller and jump "
        "optimal control code from the ALPINE climbing_robots2 repository."
    ),
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ros2 run climbingrobot_optimization jump_optimizer_node
            "jump_optimizer_node = "
            "climbingrobot_optimization.ros2_nodes.jump_optimizer_node:main",
            # ros2 run climbingrobot_optimization mpc_controller_node
            "mpc_controller_node = "
            "climbingrobot_optimization.ros2_nodes.mpc_controller_node:main",
        ],
    },
)
