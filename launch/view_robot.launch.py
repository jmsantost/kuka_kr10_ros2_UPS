"""Visualiza el KR10 R1100-2 en RViz con sliders que arrancan en home."""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
                PathJoinSubstitution([FindPackageShare('kuka_agilus_support'),
                                      'urdf', 'kr10_r1100_2.urdf.xacro']),
                ' mode:=mock',
            ]),
            value_type=str,
        )
    }
    home_yaml = PathJoinSubstitution([FindPackageShare('kuka_bridge'), 'config', 'home.yaml'])
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('kuka_resources'), 'config', 'view_6_axis_urdf.rviz'])

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='both', parameters=[robot_description]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='joint_state_publisher', output='log', parameters=[home_yaml]),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', rviz_config], parameters=[robot_description]),
    ])
