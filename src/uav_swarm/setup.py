from glob import glob
from setuptools import setup

package_name = "uav_swarm"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lyj",
    maintainer_email="lyj@example.com",
    description="ROS2 multi-UAV swarm simulation, formation control, evaluation, and visualization.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "swarm_sim_node = uav_swarm.swarm_sim_node:main",
            "swarm_controller_node = uav_swarm.swarm_controller_node:main",
            "swarm_comm_node = uav_swarm.swarm_comm_node:main",
            "swarm_dynamic_obstacle_node = uav_swarm.swarm_dynamic_obstacle_node:main",
            "swarm_task_manager_node = uav_swarm.swarm_task_manager_node:main",
            "swarm_agent_bidder_node = uav_swarm.swarm_agent_bidder_node:main",
            "swarm_keyboard_command_node = uav_swarm.swarm_keyboard_command_node:main",
            "swarm_eval_node = uav_swarm.swarm_eval_node:main",
            "swarm_viz_node = uav_swarm.swarm_viz_node:main",
            "swarm_compare_metrics = uav_swarm.swarm_compare_metrics:main",
            "swarm_marl_rollout_node = uav_swarm.swarm_marl_rollout_node:main",
            "swarm_marl_compare = uav_swarm.swarm_marl_compare:main",
        ],
    },
)
