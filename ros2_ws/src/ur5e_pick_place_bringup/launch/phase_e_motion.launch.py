"""Phase E: Motion execution verification.

Launches Phase A hardware plus the MTC pick-place server.
Use this to validate that a hardcoded grasp pose can be executed
cleanly (approach, close, lift, place, retreat) before connecting
live GPD output.

To test, call the service manually:
  ros2 service call /execute_pick_place ur5e_pick_place_interfaces/srv/PickPlace \\
    "{grasp_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.4, y: 0.0, z: 0.1}, orientation: {w: 0.0, x: 0.0, y: 1.0, z: 0.0}}}, place_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.55, y: -0.3, z: 0.2}, orientation: {w: 0.0, x: 0.0, y: 1.0, z: 0.0}}}}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    ur_type = LaunchConfiguration('ur_type')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    camera_mount = LaunchConfiguration('camera_mount')

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

    mtc_server = Node(
        package='ur5e_pick_place_mtc',
        executable='pick_place_server',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.251'),
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('camera_mount', default_value='fixed',
                              description='Camera mounting: "fixed" (eye-to-hand) or "wrist" (eye-in-hand)'),
        hardware_launch,
        mtc_server,
    ])
