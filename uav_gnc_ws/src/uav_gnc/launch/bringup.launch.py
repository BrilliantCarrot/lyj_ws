from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('uav_gnc')
    simulation_yaml = os.path.join(pkg, 'config', 'simulation.yaml')
    guidance_yaml = os.path.join(pkg, 'config', 'guidance.yaml')
    control_yaml = os.path.join(pkg, 'config', 'control.yaml')
    navigation_yaml = os.path.join(pkg, 'config', 'navigation.yaml')

    simulation = Node(
        package='uav_gnc',
        executable='simulation_node',   # CMake에서 만든 실행파일 이름
        name='simulation_node',
        output='screen',
        parameters=[simulation_yaml]
    )
    guidance = Node(
        package='uav_gnc',
        executable='guidance_node',
        name='guidance_node',
        output='screen',
        parameters=[guidance_yaml]
    )
    navigation = Node(
        package='uav_gnc',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[navigation_yaml]
    )
    control = Node(
        package='uav_gnc',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[control_yaml]
    )
    return LaunchDescription([
        simulation,
        guidance,
        navigation,
        control,
    ])