from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sim = Node(
        package='uav_gnc',
        executable='simulation_node',   # CMake에서 만든 실행파일 이름
        name='sim_node',
        output='screen'
        # parameters=[ ... ]  # yaml은 나중에
    )

    guidance = Node(
        package='uav_gnc',
        executable='guidance_node',
        name='guidance_node',
        output='screen'
    )

    nav = Node(
        package='uav_gnc',
        executable='navigation_node',
        name='navigation_node',
        output='screen'
    )

    control = Node(
        package='uav_gnc',
        executable='control_node',
        name='control_node',
        output='screen'
    )

    return LaunchDescription([
        sim,
        guidance,
        nav,
        control
    ])