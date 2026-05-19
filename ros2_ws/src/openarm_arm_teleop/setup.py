from setuptools import find_packages, setup

package_name = "openarm_arm_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/openarm_arm_teleop.launch.py",
                "launch/openarm_ee_teleop_sim.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/openarm_arm_teleop.yaml",
                "config/openarm_ee_teleop.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="F710 teleop package for OpenArm bimanual control.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "arm_teleop_node = openarm_arm_teleop.arm_teleop_node:main",
            "ee_teleop_node = openarm_arm_teleop.ee_teleop_node:main",
        ],
    },
)
