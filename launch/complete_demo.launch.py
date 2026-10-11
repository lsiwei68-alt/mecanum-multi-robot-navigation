import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_multi_nav')
    dual_nav = os.path.join(package_share, 'launch', 'dual_robot_nav.launch.py')
    rviz_config = os.path.join(package_share, 'rviz', 'multi_robot.rviz')

    headless = LaunchConfiguration('headless')
    native_lidar = LaunchConfiguration('native_lidar')
    software_rendering = LaunchConfiguration('software_rendering')
    use_rviz = LaunchConfiguration('rviz')
    run_demo = LaunchConfiguration('run_demo')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    spacing = LaunchConfiguration('spacing')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('native_lidar', default_value='false'),
        DeclareLaunchArgument('software_rendering', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('run_demo', default_value='true'),
        DeclareLaunchArgument('goal_x', default_value='3.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('spacing', default_value='1.2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dual_nav),
            launch_arguments={
                'headless': headless,
                'native_lidar': native_lidar,
                'software_rendering': software_rendering,
            }.items(),
        ),
        TimerAction(period=10.0, actions=[
            Node(
                package='rviz2', executable='rviz2', name='multi_robot_rviz',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}], output='screen',
                condition=IfCondition(use_rviz),
            ),
        ]),
        TimerAction(period=13.0, actions=[
            Node(
                package='mecanum_multi_nav', executable='formation_demo',
                name='formation_demo_launcher', output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'goal_x': ParameterValue(goal_x, value_type=float),
                    'goal_y': ParameterValue(goal_y, value_type=float),
                    'goal_yaw': ParameterValue(goal_yaw, value_type=float),
                    'spacing': ParameterValue(spacing, value_type=float),
                }],
                condition=IfCondition(run_demo),
            ),
        ]),
    ])
