"""Launch file to visualize the workcell URDF in RViz."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('ur5e_workcell_description'),
            'urdf', 'ur5e_workcell.urdf.xacro',
        ]),
        ' ur_type:=', ur_type,
        ' use_fake_hardware:=true',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('ur5e_workcell_description'),
            'config', 'view_workcell.rviz',
        ])],
    )

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
