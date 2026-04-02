from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_share = get_package_share_directory('ur5e_pick_place_bringup')
    perception_share = get_package_share_directory('ur5e_tabletop_perception')

    workspace_yaml = os.path.join(bringup_share, 'config', 'workspace.yaml')
    rs_yaml = os.path.join(bringup_share, 'config', 'realsense.yaml')
    handeye_yaml = os.path.join(bringup_share, 'config', 'handeye_static.yaml')

    # Replace these with your actual UR driver and MoveIt launch files/packages
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/path/to/ur_robot_driver/launch/ur_control.launch.py'
        ),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': '192.168.0.10',
            'launch_rviz': 'false',
        }.items()
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/path/to/your_ur_moveit_config/launch/move_group.launch.py'
        )
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30',
        }.items()
    )

    handeye_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.420', '-0.015', '0.865',
            '0.7071', '0.7071', '0.0', '0.0',
            'base', 'camera_link'
        ]
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
        output='screen'
    )

    gpd_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gpd_ros2'),
                'launch',
                'grasp_detection_server.launch.py'
            )
        )
    )

    mtc_server = Node(
        package='ur5e_pick_place_mtc',
        executable='pick_place_server',
        output='screen'
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
        output='screen'
    )

    return LaunchDescription([
        ur_driver_launch,
        moveit_launch,
        realsense_launch,
        handeye_tf,
        perception_node,
        gpd_server,
        mtc_server,
        supervisor,
    ])