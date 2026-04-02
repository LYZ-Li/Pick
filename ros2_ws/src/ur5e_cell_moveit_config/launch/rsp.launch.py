import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('ur5e_cell_description')
    model_path = os.path.join(description_share, 'urdf', 'ur5e_cell.urdf.xacro')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            model_path,
            ' ',
            'ur_type:=', LaunchConfiguration('ur_type'),
            ' ',
            'use_fake_hardware:=', LaunchConfiguration('use_fake_hardware'),
            ' ',
            'gripper_com_port:=', LaunchConfiguration('gripper_com_port'),
            ' ',
            'camera_mount_mode:=', LaunchConfiguration('camera_mount_mode'),
            ' ',
            'kinematics_params:=', LaunchConfiguration('kinematics_parameters_file'),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('gripper_com_port', default_value='/dev/robotiq'),
        DeclareLaunchArgument('camera_mount_mode', default_value='fixed'),
        DeclareLaunchArgument(
            'kinematics_parameters_file',
            default_value=os.path.join(
                get_package_share_directory('ur_description'),
                'config',
                'ur5e',
                'default_kinematics.yaml',
            ),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_description}],
        ),
    ])
