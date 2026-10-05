"""MoveIt 2 + kuka_bridge para el KR10 R1100-2.

La velocidad real la fija ros_server.src (VEL_PTP), no el escalado de MoveIt.

    ros2 launch kuka_bridge moveit.launch.py                     # planificar, sin mover
    ros2 launch kuka_bridge moveit.launch.py allow_motion:=true  # Execute mueve el robot
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

MODEL = 'kr10_r1100_2'


def generate_launch_description():
    support = get_package_share_directory('kuka_agilus_support')
    moveit_pkg = get_package_share_directory('kuka_kr_moveit_config')
    bridge_pkg = get_package_share_directory('kuka_bridge')

    moveit_config = (
        MoveItConfigsBuilder('kuka_kr', package_name='kuka_kr_moveit_config')
        .robot_description(file_path=os.path.join(support, 'urdf', f'{MODEL}.urdf.xacro'),
                           mappings={'mode': 'mock'})
        .robot_description_semantic(file_path=os.path.join(moveit_pkg, 'urdf', f'{MODEL}.srdf'))
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path=os.path.join(support, 'config', f'{MODEL}_joint_limits.yaml'))
        .trajectory_execution(file_path=os.path.join(bridge_pkg, 'config',
                                                     'moveit_controllers.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    # planning_6_axis.rviz de kuka_resources sin el panel RvizVisualToolsGui
    rviz_config = os.path.join(bridge_pkg, 'config', 'moveit.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='172.31.1.147'),
        DeclareLaunchArgument('allow_motion', default_value='false'),
        DeclareLaunchArgument('robot_port', default_value='7000'),
        DeclareLaunchArgument('max_step_deg', default_value='10.0'),
        DeclareLaunchArgument('allowed_modes', default_value='T1',
                              description="Modos en los que se permite mover, p. ej. 'T1,AUT'"),
        DeclareLaunchArgument('protocol', default_value='single',
                              description='single (ros_server.src) o stream (ros_stream.src)'),
        DeclareLaunchArgument('max_total_deg', default_value='0.0',
                              description='Movimiento total maximo por eje (0 = sin limite)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('scene', default_value='true',
                              description='Cargar config/scene.yaml en MoveIt'),
        Node(package='kuka_bridge', executable='kuka_bridge', output='screen',
             parameters=[{
                 'robot_ip': LaunchConfiguration('robot_ip'),
                 'protocol': LaunchConfiguration('protocol'),
                 'allowed_modes': LaunchConfiguration('allowed_modes'),
                 'robot_port': ParameterValue(LaunchConfiguration('robot_port'),
                                              value_type=int),
                 'allow_motion': ParameterValue(LaunchConfiguration('allow_motion'),
                                                value_type=bool),
                 'max_step_deg': ParameterValue(LaunchConfiguration('max_step_deg'),
                                                value_type=float),
                 'max_total_deg': ParameterValue(LaunchConfiguration('max_total_deg'),
                                                 value_type=float),
             }]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='both', parameters=[moveit_config.robot_description]),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[moveit_config.to_dict()]),
        Node(package='kuka_bridge', executable='scene_publisher', output='screen',
             parameters=[os.path.join(bridge_pkg, 'config', 'scene.yaml')],
             condition=IfCondition(LaunchConfiguration('scene'))),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', rviz_config],
             # El panel MotionPlanning copia los parametros de move_group y falla
             # si aqui ya hay alguno numerico con el mismo nombre: solo el solver.
             parameters=[{'robot_description_kinematics': {'manipulator': {
                 'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin'}}}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
