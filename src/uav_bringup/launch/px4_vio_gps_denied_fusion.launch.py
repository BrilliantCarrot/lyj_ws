from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    publish_vio_velocity = LaunchConfiguration("publish_vio_velocity")
    vio_px4_rate_hz = LaunchConfiguration("vio_px4_rate_hz")
    vio_position_variance = LaunchConfiguration("vio_position_variance")
    vio_velocity_variance = LaunchConfiguration("vio_velocity_variance")
    run_velocity_check = LaunchConfiguration("run_velocity_check")
    vio_align_use_fixed = LaunchConfiguration("vio_align_use_fixed")
    vio_align_yaw_deg = LaunchConfiguration("vio_align_yaw_deg")
    vio_align_offset_x = LaunchConfiguration("vio_align_offset_x")
    vio_align_offset_y = LaunchConfiguration("vio_align_offset_y")
    vio_align_offset_z = LaunchConfiguration("vio_align_offset_z")

    px4_vio_shadow_launch = os.path.join(
        get_package_share_directory("uav_bringup"),
        "launch",
        "px4_vio_shadow.launch.py",
    )

    vio_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(px4_vio_shadow_launch),
        launch_arguments={
            "start_openvins": "true",
            "align_vio_odom": "true",
            "publish_to_px4_ekf": "true",
            "vio_px4_input_topic": "/vio_aligned/odom",
            "publish_vio_velocity": publish_vio_velocity,
            "vio_px4_rate_hz": vio_px4_rate_hz,
            "vio_position_variance": vio_position_variance,
            "vio_velocity_variance": vio_velocity_variance,
            "run_velocity_check": run_velocity_check,
            "vio_align_use_fixed": vio_align_use_fixed,
            "vio_align_yaw_deg": vio_align_yaw_deg,
            "vio_align_offset_x": vio_align_offset_x,
            "vio_align_offset_y": vio_align_offset_y,
            "vio_align_offset_z": vio_align_offset_z,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_vio_velocity",
            default_value="false",
            description="GPS-denied first-pass default keeps EV velocity disabled. Enable only after velocity validation.",
        ),
        DeclareLaunchArgument(
            "vio_px4_rate_hz",
            default_value="50.0",
            description="PX4 external vision input rate limit.",
        ),
        DeclareLaunchArgument(
            "vio_position_variance",
            default_value="0.09",
            description="Position variance sent to PX4 EKF2.",
        ),
        DeclareLaunchArgument(
            "vio_velocity_variance",
            default_value="0.16",
            description="Velocity variance sent to PX4 EKF2 when velocity publishing is enabled.",
        ),
        DeclareLaunchArgument(
            "run_velocity_check",
            default_value="true",
            description="Run VIO-vs-PX4 velocity comparison while testing GPS-denied fusion.",
        ),
        DeclareLaunchArgument(
            "vio_align_use_fixed",
            default_value="false",
            description="Use fixed VIO yaw/offset alignment instead of live /nav/odom calibration.",
        ),
        DeclareLaunchArgument(
            "vio_align_yaw_deg",
            default_value="0.0",
            description="Fixed yaw alignment in degrees.",
        ),
        DeclareLaunchArgument(
            "vio_align_offset_x",
            default_value="0.0",
            description="Fixed x offset.",
        ),
        DeclareLaunchArgument(
            "vio_align_offset_y",
            default_value="0.0",
            description="Fixed y offset.",
        ),
        DeclareLaunchArgument(
            "vio_align_offset_z",
            default_value="0.0",
            description="Fixed z offset.",
        ),
        vio_pipeline,
    ])
