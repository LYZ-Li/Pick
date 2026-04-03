"""Phase D: Grasp detection verification.

Launches Phase C (hardware + perception) plus GPD grasp detection.
Use this to validate that GPD generates physically plausible grasps
for the Robotiq gripper. Visualize grasp candidates in RViz.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    ur_type = LaunchConfiguration('ur_type')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    camera_mount = LaunchConfiguration('camera_mount')

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur5e_pick_place_bringup'),
                'launch', 'phase_c_perception.launch.py',
            ])
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'ur_type': ur_type,
            'use_fake_hardware': use_fake_hardware,
            'camera_mount': camera_mount,
        }.items(),
    )

    gpd_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gpd_ros2'),
                'launch', 'grasp_detection_server.launch.py',
            ])
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.251'),
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('camera_mount', default_value='fixed',
                              description='Camera mounting: "fixed" (eye-to-hand) or "wrist" (eye-in-hand)'),
        perception_launch,
        gpd_server,
    ])
