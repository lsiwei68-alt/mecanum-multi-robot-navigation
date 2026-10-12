import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_multi_nav')
    generic_demo = os.path.join(
        package_share, 'launch', 'narrow_passage_demo.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(generic_demo),
            launch_arguments={
                'world': os.path.join(
                    package_share, 'worlds', 'bottleneck_corridor.sdf'),
                'map': os.path.join(
                    package_share, 'maps', 'bottleneck_corridor.yaml'),
                'scenario': 'bottleneck_corridor',
                'robot1_start_x': '-1.4',
                'robot1_start_y': '-3.8',
                'robot1_start_yaw': '1.5708',
                'robot2_start_x': '1.4',
                'robot2_start_y': '-3.8',
                'robot2_start_yaw': '1.5708',
                'robot1_goal_x': '-0.6',
                'robot1_goal_y': '3.8',
                'robot1_goal_yaw': '1.5708',
                'robot2_goal_x': '0.6',
                'robot2_goal_y': '3.8',
                'robot2_goal_yaw': '1.5708',
            }.items(),
        ),
    ])
