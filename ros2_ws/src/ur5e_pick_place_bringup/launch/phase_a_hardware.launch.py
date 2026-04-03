"""Phase A: Hardware bringup only.

Launches UR driver, MoveIt, RealSense, the Robotiq gripper controller,
and the camera TF.
Use this to validate that all hardware interfaces are stable before
moving to calibration or perception.

Camera mount options:
  camera_mount:=fixed  (default) — eye-to-hand, static TF from base_link to camera
  camera_mount:=wrist            — eye-in-hand, camera TF from URDF on tool0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    ur_type = LaunchConfiguration('ur_type')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    camera_mount = LaunchConfiguration('camera_mount')
    launch_gripper = LaunchConfiguration('launch_gripper')
    gripper_port = LaunchConfiguration('gripper_port')
    robotiq_controllers_config = PathJoinSubstitution([
        FindPackageShare('ur5e_pick_place_bringup'),
        'config', 'robotiq_gripper_controllers.yaml',
    ])

    # UR Robot Driver
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

    # MoveIt
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'launch', 'ur_moveit.launch.py',
            ])
        ),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': 'true',
            'use_fake_hardware': use_fake_hardware,
        }.items(),
    )

    robotiq_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('ur5e_pick_place_bringup'),
            'urdf', 'robotiq_gripper_control.urdf.xacro',
        ]),
        ' use_fake_hardware:=', use_fake_hardware,
        ' gripper_port:=', gripper_port,
    ])

    robotiq_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='robotiq_controller_manager',
        parameters=[
            {'robot_description': robotiq_description},
            robotiq_controllers_config,
        ],
        output='screen',
        condition=IfCondition(launch_gripper),
    )

    robotiq_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/robotiq_controller_manager',
            '-p', robotiq_controllers_config,
        ],
        output='screen',
        condition=IfCondition(launch_gripper),
    )

    robotiq_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_position_controller',
            '-c', '/robotiq_controller_manager',
            '-p', robotiq_controllers_config,
        ],
        output='screen',
        condition=IfCondition(launch_gripper),
    )

    # RealSense Camera
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

    # Static hand-eye TF — only needed for fixed (eye-to-hand) camera mount.
    # For wrist mount the camera_link TF comes from the URDF via robot_state_publisher.
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

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.251'),
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument(
            'launch_gripper',
            default_value='true',
            description='Launch the Robotiq gripper ros2_control path.',
        ),
        DeclareLaunchArgument(
            'gripper_port',
            default_value='/dev/robotiq',
            description='Serial device exposed to the Robotiq driver.',
        ),
        DeclareLaunchArgument('camera_mount', default_value='fixed',
                              description='Camera mounting: "fixed" (eye-to-hand) or "wrist" (eye-in-hand)'),
        ur_driver_launch,
        moveit_launch,
        robotiq_controller_manager,
        robotiq_joint_state_broadcaster_spawner,
        robotiq_gripper_controller_spawner,
        realsense_launch,
        handeye_tf,
    ])
