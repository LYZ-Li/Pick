"""Phase C: Perception verification.

Launches Phase A hardware plus the tabletop perception node.
Use this to validate that the segmented cloud excludes the table
and contains only the target objects. Tune workspace bounds and
RANSAC thresholds in workspace.yaml.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory('ur5e_pick_place_bringup')
    workspace_yaml = os.path.join(bringup_share, 'config', 'workspace.yaml')

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

    perception_node = Node(
        package='ur5e_tabletop_perception',
        executable='tabletop_perception_node',
        parameters=[{
            'input_cloud_topic': '/camera/depth/color/points',
            'output_cloud_topic': '/tabletop/segmented_cloud',
            'target_frame': 'base',
            'workspace_yaml': workspace_yaml,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.251'),
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('camera_mount', default_value='fixed',
                              description='Camera mounting: "fixed" (eye-to-hand) or "wrist" (eye-in-hand)'),
        hardware_launch,
        perception_node,
    ])
