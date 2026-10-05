"""kuka_bridge + robot_state_publisher + RViz (posicion real del robot).

    ros2 launch kuka_bridge bridge.launch.py                  # solo lectura
    ros2 launch kuka_bridge bridge.launch.py allow_motion:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
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
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('kuka_resources'), 'config', 'view_6_axis_urdf.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='172.31.1.147'),
        DeclareLaunchArgument('robot_port', default_value='7000'),
        DeclareLaunchArgument('allow_motion', default_value='false'),
        DeclareLaunchArgument('max_step_deg', default_value='10.0'),
        DeclareLaunchArgument('allowed_modes', default_value='T1',
                              description="Modos en los que se permite mover, p. ej. 'T1,AUT'"),
        DeclareLaunchArgument('protocol', default_value='single',
                              description='single (ros_server.src) o stream (ros_stream.src)'),
        DeclareLaunchArgument('max_total_deg', default_value='0.0',
                              description='Movimiento total maximo por eje (0 = sin limite)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='kuka_bridge', executable='kuka_bridge', output='screen',
             parameters=[{
                 'robot_ip': LaunchConfiguration('robot_ip'),
                 'robot_port': ParameterValue(LaunchConfiguration('robot_port'),
                                              value_type=int),
                 'protocol': LaunchConfiguration('protocol'),
                 'allowed_modes': LaunchConfiguration('allowed_modes'),
                 'allow_motion': ParameterValue(LaunchConfiguration('allow_motion'),
                                                value_type=bool),
                 'max_step_deg': ParameterValue(LaunchConfiguration('max_step_deg'),
                                                value_type=float),
                 'max_total_deg': ParameterValue(LaunchConfiguration('max_total_deg'),
                                                 value_type=float),
             }]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='both', parameters=[robot_description]),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', rviz_config], parameters=[robot_description],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
