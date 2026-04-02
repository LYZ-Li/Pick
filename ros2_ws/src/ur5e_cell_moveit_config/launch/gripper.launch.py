import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('robotiq_description')
    model_path = os.path.join(
        description_share,
        'urdf',
        'robotiq_2f_85_gripper.urdf.xacro',
    )
    update_rate_yaml = os.path.join(
        description_share,
        'config',
        'robotiq_update_rate.yaml',
    )
    controllers_yaml = os.path.join(
        description_share,
        'config',
        'robotiq_controllers.yaml',
    )

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            model_path,
            ' ',
            'use_fake_hardware:=false',
            ' ',
            'com_port:=', LaunchConfiguration('gripper_com_port'),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('gripper_com_port', default_value='/dev/robotiq'),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace='gripper',
            parameters=[
                {'robot_description': robot_description},
                update_rate_yaml,
                controllers_yaml,
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager',
                '/gripper/controller_manager',
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'robotiq_gripper_controller',
                '--controller-manager',
                '/gripper/controller_manager',
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'robotiq_activation_controller',
                '--controller-manager',
                '/gripper/controller_manager',
            ],
            output='screen',
        ),
    ])
