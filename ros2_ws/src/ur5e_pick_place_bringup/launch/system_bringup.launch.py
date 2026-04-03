"""Full system bringup — launches all phases together.

For incremental validation, prefer the individual phase launch files:
  phase_a_hardware.launch.py
  phase_b_calibration.launch.py
  phase_c_perception.launch.py
  phase_d_grasp.launch.py
  phase_e_motion.launch.py
  phase_f_loop.launch.py

Camera mount options:
  camera_mount:=fixed  (default) — eye-to-hand, static TF from base_link to camera
  camera_mount:=wrist            — eye-in-hand, camera TF from URDF on tool0
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import LaunchConfigurationEquals
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
    launch_rviz = LaunchConfiguration('launch_rviz')
    camera_mount = LaunchConfiguration('camera_mount')

    # ---- UR Robot Driver ----
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch', 'ur_control.launch.py',
            ])
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'launch_rviz': 'false',
            'use_fake_hardware': use_fake_hardware,
        }.items(),
    )

    # ---- MoveIt Move Group ----
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'launch', 'ur_moveit.launch.py',
            ])
        ),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': launch_rviz,
            'use_fake_hardware': use_fake_hardware,
        }.items(),
    )

    # ---- RealSense Camera ----
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch', 'rs_launch.py',
            ])
        ),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30',
            'initial_reset': 'true',
        }.items(),
    )

    # ---- Hand-Eye Static TF (only for fixed camera mount) ----
    handeye_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.420', '--y', '-0.015', '--z', '0.865',
            '--qx', '0.7071', '--qy', '0.7071', '--qz', '0.0', '--qw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
        ],
        condition=LaunchConfigurationEquals('camera_mount', 'fixed'),
    )

    # ---- Tabletop Perception ----
    perception_node = Node(
        package='ur5e_tabletop_perception',
        executable='tabletop_perception_node',
        parameters=[{
            'input_cloud_topic': '/camera/depth/color/points',
            'output_cloud_topic': '/tabletop/segmented_cloud',
            'target_frame': 'base_link',
            'workspace_yaml': workspace_yaml,
        }],
        output='screen',
    )

    # ---- GPD Grasp Detection ----
    gpd_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gpd_ros2'),
                'launch', 'grasp_detection_server.launch.py',
            ])
        ),
    )

    # ---- MTC Pick-Place Server ----
    mtc_server = Node(
        package='ur5e_pick_place_mtc',
        executable='pick_place_server',
        output='screen',
    )

    # ---- Supervisor ----
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
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('camera_mount', default_value='fixed',
                              description='Camera mounting: "fixed" (eye-to-hand) or "wrist" (eye-in-hand)'),
        ur_driver_launch,
        moveit_launch,
        realsense_launch,
        handeye_tf,
        perception_node,
        gpd_server,
        mtc_server,
        supervisor,
    ])
