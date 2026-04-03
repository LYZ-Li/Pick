"""Phase B: Hand-eye calibration.

Launches Phase A hardware plus easy_handeye2 for calibration.
Requires a ChArUco or ArUco board visible to the camera.

Calibration type is selected automatically based on camera_mount:
  camera_mount:=fixed  → eye_to_hand calibration
  camera_mount:=wrist  → eye_in_hand calibration

After calibration is complete, update the camera extrinsics in the
workcell description or replace the static TF publisher with the
calibrated result.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    ur_type = LaunchConfiguration('ur_type')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    camera_mount = LaunchConfiguration('camera_mount')

    # Derive calibration type from camera_mount
    calibration_type = PythonExpression([
        "'eye_in_hand' if '", camera_mount, "' == 'wrist' else 'eye_to_hand'"
    ])

    # Phase A hardware
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur5e_pick_place_bringup'),
                'launch', 'phase_a_hardware.launch.py',
            ])
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'ur_type': ur_type,
            'use_fake_hardware': use_fake_hardware,
            'camera_mount': camera_mount,
        }.items(),
    )

    # easy_handeye2 calibration
    handeye_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('easy_handeye2'),
                'launch', 'calibrate.launch.py',
            ])
        ),
        launch_arguments={
            'calibration_type': calibration_type,
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'tool0',
            'tracking_base_frame': 'camera_link',
            'tracking_marker_frame': 'marker_frame',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.251'),
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('camera_mount', default_value='fixed',
                              description='Camera mounting: "fixed" (eye-to-hand) or "wrist" (eye-in-hand)'),
        hardware_launch,
        handeye_launch,
    ])
