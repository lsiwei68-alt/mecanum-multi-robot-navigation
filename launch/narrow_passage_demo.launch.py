import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_multi_nav')
    dual_nav = os.path.join(package_share, 'launch', 'dual_robot_nav.launch.py')
    world = os.path.join(package_share, 'worlds', 'narrow_passage.sdf')
    map_file = os.path.join(package_share, 'maps', 'narrow_passage.yaml')
    rviz_config = os.path.join(package_share, 'rviz', 'multi_robot.rviz')

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')
    run_demo = LaunchConfiguration('run_demo')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('run_demo', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dual_nav),
            launch_arguments={
                'headless': headless,
                'native_lidar': 'false',
                'world': world,
                'map': map_file,
                'scenario': 'narrow_passage',
            }.items(),
        ),
        TimerAction(period=10.0, actions=[
            Node(
                package='rviz2', executable='rviz2',
                name='narrow_passage_rviz',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}], output='screen',
                condition=IfCondition(use_rviz),
            ),
        ]),
        TimerAction(period=13.0, actions=[
            Node(
                package='mecanum_multi_nav',
                executable='trajectory_coordinator',
                name='trajectory_coordinator',
                parameters=[{'use_sim_time': True}], output='screen',
                condition=IfCondition(run_demo),
            ),
        ]),
    ])
