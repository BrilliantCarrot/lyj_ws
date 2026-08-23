from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    start_openvins = LaunchConfiguration("start_openvins")
    publish_to_px4_ekf = LaunchConfiguration("publish_to_px4_ekf")
    publish_vio_velocity = LaunchConfiguration("publish_vio_velocity")
    vio_px4_rate_hz = LaunchConfiguration("vio_px4_rate_hz")
    vio_position_variance = LaunchConfiguration("vio_position_variance")
    vio_velocity_variance = LaunchConfiguration("vio_velocity_variance")
    align_vio_odom = LaunchConfiguration("align_vio_odom")
    run_velocity_check = LaunchConfiguration("run_velocity_check")
    vio_align_use_fixed = LaunchConfiguration("vio_align_use_fixed")
    vio_align_yaw_deg = LaunchConfiguration("vio_align_yaw_deg")
    vio_align_offset_x = LaunchConfiguration("vio_align_offset_x")
    vio_align_offset_y = LaunchConfiguration("vio_align_offset_y")
    vio_align_offset_z = LaunchConfiguration("vio_align_offset_z")
    vio_px4_input_topic = LaunchConfiguration("vio_px4_input_topic")
    openvins_config = LaunchConfiguration("openvins_config")
    openvins_package = LaunchConfiguration("openvins_package")
    openvins_executable = LaunchConfiguration("openvins_executable")

    bridge_config = os.path.join(
        get_package_share_directory("uav_bringup"),
        "config",
        "px4_vio_shadow_bridge.yaml",
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="px4_vio_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )

    imu_adapter = Node(
        package="uav_perception",
        executable="imu_lio_adapter_node",
        name="imu_vio_adapter_node",
        output="screen",
        parameters=[{
            "input_topic": "/gazebo/imu_raw",
            "output_topic": "/vio/imu",
            "output_frame_id": "imu",
            "preserve_input_frame": False,
            "accel_noise_std": 0.03,
            "gyro_noise_std": 0.002,
            "orientation_covariance": 0.01,
            "enforce_monotonic_stamps": True,
            "monotonic_stamp_step_ns": 1000000,
        }],
    )

    openvins = Node(
        package=openvins_package,
        executable=openvins_executable,
        name="openvins_msckf",
        output="screen",
        condition=IfCondition(start_openvins),
        parameters=[{
            "verbosity": "INFO",
            "use_stereo": True,
            "max_cameras": 2,
            "save_total_state": False,
            "config_path": openvins_config,
        }],
        remappings=[
            ("imu0", "/vio/imu"),
            ("cam0/image_raw", "/stereo/left/image_raw"),
            ("cam1/image_raw", "/stereo/right/image_raw"),
            ("/imu0", "/vio/imu"),
            ("/cam0/image_raw", "/stereo/left/image_raw"),
            ("/cam1/image_raw", "/stereo/right/image_raw"),
            ("odomimu", "/vio/odom"),
            ("pathimu", "/vio/path"),
            ("poseimu", "/vio/pose"),
            ("ov_msckf/odomimu", "/vio/odom"),
            ("ov_msckf/pathimu", "/vio/path"),
            ("/ov_msckf/odomimu", "/vio/odom"),
            ("/ov_msckf/pathimu", "/vio/path"),
        ],
    )

    vio_to_px4_ev = Node(
        package="uav_px4_bridge",
        executable="lio_to_px4_visual_odometry",
        name="vio_to_px4_visual_odometry",
        output="screen",
        condition=IfCondition(publish_to_px4_ekf),
        parameters=[{
            "input_topic": vio_px4_input_topic,
            "output_topic": "/fmu/in/vehicle_visual_odometry",
            "publish_orientation": False,
            "publish_velocity": publish_vio_velocity,
            "publish_rate_hz": vio_px4_rate_hz,
            "position_variance": vio_position_variance,
            "velocity_variance": vio_velocity_variance,
            "quality": 90,
        }],
    )

    vio_odom_aligner = Node(
        package="uav_bringup",
        executable="vio_odom_aligner_node",
        name="vio_odom_aligner_node",
        output="screen",
        condition=IfCondition(align_vio_odom),
        parameters=[{
            "reference_topic": "/nav/odom",
            "vio_topic": "/vio/odom",
            "output_topic": "/vio_aligned/odom",
            "output_frame_id": "world",
            "output_child_frame_id": "imu",
            "calibration_samples": 200,
            "min_motion_m": 0.25,
            "align_z_offset": True,
            "rotate_orientation_yaw": True,
            "use_fixed_alignment": vio_align_use_fixed,
            "fixed_yaw_deg": vio_align_yaw_deg,
            "fixed_offset_x": vio_align_offset_x,
            "fixed_offset_y": vio_align_offset_y,
            "fixed_offset_z": vio_align_offset_z,
        }],
    )

    vio_velocity_error = Node(
        package="uav_bringup",
        executable="vio_velocity_error_node",
        name="vio_velocity_error_node",
        output="screen",
        condition=IfCondition(run_velocity_check),
        parameters=[{
            "reference_topic": "/nav/odom",
            "vio_topic": "/vio_aligned/odom",
            "max_pair_age_s": 0.05,
            "min_samples": 20,
        }],
    )

    base_to_stereo_left_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_stereo_left_tf",
        arguments=["0.14", "0.06", "0.08", "0.0", "0.0", "0.0", "base_link", "stereo_left"],
    )

    base_to_stereo_right_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_stereo_right_tf",
        arguments=["0.14", "-0.06", "0.08", "0.0", "0.0", "0.0", "base_link", "stereo_right"],
    )

    base_to_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_vio_imu_tf",
        arguments=["0.0", "0.0", "0.02", "0.0", "0.0", "0.0", "base_link", "imu"],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_openvins",
            default_value="false",
            description="Start OpenVINS after sourcing its install/setup.bash. Default false keeps this launch as a sensor shadow pipeline.",
        ),
        DeclareLaunchArgument(
            "publish_to_px4_ekf",
            default_value="false",
            description="Publish /vio/odom to PX4 EKF2 as /fmu/in/vehicle_visual_odometry.",
        ),
        DeclareLaunchArgument(
            "publish_vio_velocity",
            default_value="false",
            description="Include VIO velocity in the PX4 external vision message.",
        ),
        DeclareLaunchArgument(
            "vio_px4_rate_hz",
            default_value="50.0",
            description="Rate limit for PX4 external vision input. Set <=0 to publish every VIO odom sample.",
        ),
        DeclareLaunchArgument(
            "vio_position_variance",
            default_value="0.09",
            description="Position variance sent in PX4 vehicle_visual_odometry.",
        ),
        DeclareLaunchArgument(
            "vio_velocity_variance",
            default_value="0.16",
            description="Velocity variance sent in PX4 vehicle_visual_odometry when velocity publishing is enabled.",
        ),
        DeclareLaunchArgument(
            "vio_px4_input_topic",
            default_value="/vio/odom",
            description="Odometry topic converted to PX4 external vision. Use /vio_aligned/odom after frame alignment validation.",
        ),
        DeclareLaunchArgument(
            "align_vio_odom",
            default_value="false",
            description="Publish /vio_aligned/odom by yaw/axis-aligning OpenVINS odom to /nav/odom for shadow validation.",
        ),
        DeclareLaunchArgument(
            "run_velocity_check",
            default_value="false",
            description="Compare /vio_aligned/odom velocity against /nav/odom for pre-fusion validation.",
        ),
        DeclareLaunchArgument(
            "vio_align_use_fixed",
            default_value="false",
            description="Use fixed VIO yaw/offset alignment instead of calibrating from /nav/odom.",
        ),
        DeclareLaunchArgument(
            "vio_align_yaw_deg",
            default_value="0.0",
            description="Fixed yaw alignment in degrees for /vio_aligned/odom.",
        ),
        DeclareLaunchArgument(
            "vio_align_offset_x",
            default_value="0.0",
            description="Fixed x offset for /vio_aligned/odom.",
        ),
        DeclareLaunchArgument(
            "vio_align_offset_y",
            default_value="0.0",
            description="Fixed y offset for /vio_aligned/odom.",
        ),
        DeclareLaunchArgument(
            "vio_align_offset_z",
            default_value="0.0",
            description="Fixed z offset for /vio_aligned/odom.",
        ),
        DeclareLaunchArgument(
            "openvins_package",
            default_value="ov_msckf",
            description="OpenVINS ROS2 package name.",
        ),
        DeclareLaunchArgument(
            "openvins_executable",
            default_value="run_subscribe_msckf",
            description="OpenVINS executable name.",
        ),
        DeclareLaunchArgument(
            "openvins_config",
            default_value=os.path.join(
                get_package_share_directory("uav_bringup"),
                "config",
                "openvins_uav_gnc",
                "estimator_config.yaml",
            ),
            description="OpenVINS estimator config path. Placeholder until OpenVINS is installed/configured.",
        ),
        gz_bridge,
        imu_adapter,
        openvins,
        vio_to_px4_ev,
        vio_odom_aligner,
        vio_velocity_error,
        base_to_stereo_left_tf,
        base_to_stereo_right_tf,
        base_to_imu_tf,
    ])
