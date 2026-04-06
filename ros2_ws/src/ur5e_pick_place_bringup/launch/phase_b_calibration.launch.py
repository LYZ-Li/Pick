"""Phase B: Hand-eye calibration.

Launches Phase A hardware, an ArUco detector chain, and easy_handeye2.
The default supported path is a fixed camera (eye-on-base) calibration
using an ArUco board attached to the robot end effector.

After calibration is complete, update the camera extrinsics in the
workcell description or replace the static TF publisher with the
calibrated result.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    bringup_share = get_package_share_directory('ur5e_pick_place_bringup')
    calibration_config = os.path.join(
        bringup_share, 'config', 'phase_b_calibration.yaml'
    )
    if not os.path.exists(calibration_config):
        calibration_config = os.path.join(
            '/root/ros2_ws',
            'src',
            'ur5e_pick_place_bringup',
            'config',
            'phase_b_calibration.yaml',
        )

    with open(calibration_config, 'r', encoding='utf-8') as file_handle:
        config = yaml.safe_load(file_handle)

    phase_b_defaults = config['phase_b']
    aruco_defaults = config['aruco_board']

    robot_ip = LaunchConfiguration('robot_ip')
    ur_type = LaunchConfiguration('ur_type')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    camera_mount = LaunchConfiguration('camera_mount')
    launch_gripper = LaunchConfiguration('launch_gripper')
    gripper_port = LaunchConfiguration('gripper_port')
    calibration_name = LaunchConfiguration('name')
    image_topic = LaunchConfiguration('detector_image_topic')
    camera_info_topic = LaunchConfiguration('detector_camera_info_topic')
    detector_camera_frame = LaunchConfiguration('detector_camera_frame')
    detector_reference_frame = LaunchConfiguration('detector_reference_frame')
    marker_frame = LaunchConfiguration('marker_frame')
    markers_topic = LaunchConfiguration('markers_topic')
    tracked_marker_id = LaunchConfiguration('tracked_marker_id')

    calibration_type = PythonExpression([
        "'eye_in_hand' if '", camera_mount, "' == 'wrist' else 'eye_on_base'"
    ])

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
            'calibration_mode': 'true',
            'launch_gripper': launch_gripper,
            'gripper_port': gripper_port,
        }.items(),
    )

    aruco_tracker = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='aruco_tracker',
        parameters=[{
            'marker_size': aruco_defaults['marker_size_m'],
            'camera_frame': detector_camera_frame,
            'reference_frame': detector_reference_frame,
            'image_is_rectified': True,
            'use_camera_info': True,
        }],
        remappings=[
            ('/image', image_topic),
            ('/camera_info', camera_info_topic),
        ],
        output='screen',
    )

    marker_tf_bridge = Node(
        package='ur5e_pick_place_bringup',
        executable='aruco_marker_tf_bridge',
        parameters=[{
            'markers_topic': markers_topic,
            'tracked_marker_id': tracked_marker_id,
            'output_frame': marker_frame,
        }],
        output='screen',
    )

    handeye_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('easy_handeye2'),
                'launch', 'calibrate.launch.py',
            ])
        ),
        launch_arguments={
            'name': calibration_name,
            'calibration_type': calibration_type,
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'tool0',
            'tracking_base_frame': detector_reference_frame,
            'tracking_marker_frame': marker_frame,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.251'),
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument(
            'launch_gripper',
            default_value='true',
            description='Launch the Robotiq gripper ros2_control path from Phase A.',
        ),
        DeclareLaunchArgument(
            'gripper_port',
            default_value='/dev/robotiq',
            description='Serial device exposed to the Robotiq driver.',
        ),
        DeclareLaunchArgument(
            'camera_mount',
            default_value='fixed',
            description='Camera mounting: "fixed" (supported default) or "wrist".',
        ),
        DeclareLaunchArgument(
            'name',
            default_value=phase_b_defaults['calibration_name'],
            description='easy_handeye2 calibration session name.',
        ),
        DeclareLaunchArgument(
            'detector_image_topic',
            default_value=phase_b_defaults['image_topic'],
            description='Detector image topic.',
        ),
        DeclareLaunchArgument(
            'detector_camera_info_topic',
            default_value=phase_b_defaults['camera_info_topic'],
            description='Detector camera info topic.',
        ),
        DeclareLaunchArgument(
            'detector_camera_frame',
            default_value=phase_b_defaults['detector_camera_frame'],
            description='Camera optical frame used by the detector.',
        ),
        DeclareLaunchArgument(
            'detector_reference_frame',
            default_value=phase_b_defaults['detector_reference_frame'],
            description='Reference frame used for marker detections and calibration.',
        ),
        DeclareLaunchArgument(
            'marker_frame',
            default_value=phase_b_defaults['marker_frame'],
            description='TF frame published for the selected board marker.',
        ),
        DeclareLaunchArgument(
            'markers_topic',
            default_value=phase_b_defaults['markers_topic'],
            description='Aruco marker array topic published by aruco_ros.',
        ),
        DeclareLaunchArgument(
            'tracked_marker_id',
            default_value=str(aruco_defaults['tracked_marker_id']),
            description='Marker id selected from the default ArUco board.',
        ),
        DeclareLaunchArgument(
            'board_config_file',
            default_value=calibration_config,
            description='Board configuration file used by the calibration tools.',
        ),
        hardware_launch,
        aruco_tracker,
        marker_tf_bridge,
        handeye_launch,
    ])
