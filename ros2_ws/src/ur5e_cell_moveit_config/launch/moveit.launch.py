import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ur_moveit_config'),
                    'launch',
                    'ur_moveit.launch.py',
                )
            ),
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
                'launch_rviz': LaunchConfiguration('launch_rviz'),
            }.items(),
        ),
    ])
