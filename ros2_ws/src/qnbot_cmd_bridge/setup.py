from setuptools import find_packages, setup

package_name = "qnbot_cmd_bridge"

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
                "launch/exo_cmd_bridge.launch.py",
                "launch/aimotor_trigger_move.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/exo_cmd_bridge.yaml",
                "config/aimotor_trigger_move.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="将 qnbot_teleoperator 的 /exo 数据桥接到 /svtrobot_cmd、/lift_control_cmd（与 f710_teleop 接口一致）",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "exo_cmd_bridge_node = qnbot_cmd_bridge.exo_cmd_bridge_node:main",
            "aimotor_trigger_move_node = qnbot_cmd_bridge.aimotor_trigger_move_node:main",
        ],
    },
)
