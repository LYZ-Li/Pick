import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('ur5e_pick_place_bringup')

    workspace_yaml = os.path.join(bringup_share, 'config', 'workspace.yaml')
    hardcoded_pick_place_yaml = os.path.join(
        get_package_share_directory('ur5e_cell_moveit_config'),
        'config',
        'hardcoded_pick_place.yaml'
    )

    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_realsense = LaunchConfiguration('use_realsense')
    camera_name = LaunchConfiguration('camera_name')
    input_cloud_topic = LaunchConfiguration('input_cloud_topic')
    use_gpd = LaunchConfiguration('use_gpd')
    use_supervisor = LaunchConfiguration('use_supervisor')

    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur5e_cell_moveit_config'),
                'launch',
                'start_robot.launch.py'
            )
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'gripper_com_port': '/dev/robotiq',
            'camera_mount_mode': 'fixed',
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur5e_cell_moveit_config'),
                'launch',
                'moveit.launch.py'
            )
        ),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': launch_rviz,
        }.items(),
    )

    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur5e_cell_moveit_config'),
                'launch',
                'gripper.launch.py'
            )
        ),
        launch_arguments={
            'gripper_com_port': '/dev/robotiq',
            'launch_rviz': 'false',
        }.items(),
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        condition=IfCondition(use_realsense),
        launch_arguments={
            'camera_name': camera_name,
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30',
        }.items(),
    )

    perception_node = Node(
        package='ur5e_tabletop_perception',
        executable='tabletop_perception_node',
        parameters=[{
            'input_cloud_topic': input_cloud_topic,
            'output_cloud_topic': '/tabletop/segmented_cloud',
            'target_frame': 'base',
            'workspace_yaml': workspace_yaml,
        }],
        output='screen',
    )

    gpd_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gpd_ros2'),
                'launch',
                'grasp_detection_server.launch.py'
            )
        ),
        condition=IfCondition(use_gpd),
    )

    mtc_server = Node(
        package='ur5e_pick_place_mtc',
        executable='pick_place_server',
        parameters=[hardcoded_pick_place_yaml],
        output='screen',
    )

    gripper_command_server = Node(
        package='ur5e_gripper_control',
        executable='gripper_command_server',
        output='screen',
    )

    supervisor = Node(
        package='ur5e_tabletop_perception',
        executable='supervisor_node',
        condition=IfCondition(use_supervisor),
        parameters=[{
            'workspace_yaml': workspace_yaml,
            'segmented_cloud_topic': '/tabletop/segmented_cloud',
            'gpd_service': '/detect_grasps',
            'mtc_execute_service': '/execute_pick_place',
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.251'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('use_realsense', default_value='true'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument(
            'input_cloud_topic',
            default_value='/camera/depth/color/points',
        ),
        DeclareLaunchArgument('use_gpd', default_value='false'),
        DeclareLaunchArgument('use_supervisor', default_value='false'),
        ur_driver_launch,
        moveit_launch,
        gripper_launch,
        realsense_launch,
        perception_node,
        gpd_server,
        mtc_server,
        gripper_command_server,
        supervisor,
    ])
