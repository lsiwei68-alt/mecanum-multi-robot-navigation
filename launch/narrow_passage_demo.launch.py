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
    world = os.path.join(package_share, 'worlds', 'narrow_passage.sdf')
    map_file = os.path.join(package_share, 'maps', 'narrow_passage.yaml')
    rviz_config = os.path.join(package_share, 'rviz', 'multi_robot.rviz')

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')
    run_demo = LaunchConfiguration('run_demo')
    clearance = LaunchConfiguration('clearance')
    time_margin = LaunchConfiguration('time_margin')
    release_min_separation = LaunchConfiguration('release_min_separation')
    release_confirmation_cycles = LaunchConfiguration(
        'release_confirmation_cycles')
    robot1_priority = LaunchConfiguration('robot1_priority')
    robot2_priority = LaunchConfiguration('robot2_priority')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('run_demo', default_value='true'),
        DeclareLaunchArgument('clearance', default_value='0.80'),
        DeclareLaunchArgument('time_margin', default_value='4.0'),
        DeclareLaunchArgument('release_min_separation', default_value='1.0'),
        DeclareLaunchArgument(
            'release_confirmation_cycles', default_value='5'),
        DeclareLaunchArgument('robot1_priority', default_value='0'),
        DeclareLaunchArgument('robot2_priority', default_value='0'),
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
        # Let localization and both costmaps settle before requesting the
        # demonstration paths.  Interface discovery alone is too early and
        # can make Navfn choose a different route from run to run.
        TimerAction(period=13.0, actions=[
            Node(
                package='mecanum_multi_nav',
                executable='trajectory_coordinator',
                name='trajectory_coordinator',
                parameters=[{
                    'use_sim_time': True,
                    'clearance': ParameterValue(clearance, value_type=float),
                    'time_margin': ParameterValue(
                        time_margin, value_type=float),
                    'release_min_separation': ParameterValue(
                        release_min_separation, value_type=float),
                    'release_confirmation_cycles': ParameterValue(
                        release_confirmation_cycles, value_type=int),
                    'robot1_priority': ParameterValue(
                        robot1_priority, value_type=int),
                    'robot2_priority': ParameterValue(
                        robot2_priority, value_type=int),
                }],
                output='screen',
                condition=IfCondition(run_demo),
            ),
        ]),
    ])
