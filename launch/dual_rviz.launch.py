import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_multi_nav')
    config = os.path.join(package_share, 'rviz', 'multi_robot.rviz')
    return LaunchDescription([
        Node(
            package='rviz2', executable='rviz2', name='multi_robot_rviz',
            arguments=['-d', config], parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
