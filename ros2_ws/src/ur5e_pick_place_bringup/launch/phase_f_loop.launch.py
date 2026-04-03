"""Phase F: Full closed-loop pick-place forever loop.

Launches all components: hardware, perception, GPD, MTC, and supervisor.
Only use this after all previous phases have been independently validated.
The supervisor will continuously cycle: observe -> detect -> pick -> place.

The supervisor pauses after 5 consecutive failures. Restart the supervisor
node or reset failure_count to resume.
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

    # Phase D includes hardware + perception + GPD
    grasp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur5e_pick_place_bringup'),
                'launch', 'phase_d_grasp.launch.py',
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

    supervisor = Node(
        package='ur5e_tabletop_perception',
        executable='supervisor_node',
        parameters=[{
            'workspace_yaml': workspace_yaml,
            'segmented_cloud_topic': '/tabletop/segmented_cloud',
            'gpd_service': '/detect_grasps',
            'mtc_execute_service': '/execute_pick_place',
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.251'),
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('camera_mount', default_value='fixed',
                              description='Camera mounting: "fixed" (eye-to-hand) or "wrist" (eye-in-hand)'),
        grasp_launch,
        mtc_server,
        supervisor,
    ])
