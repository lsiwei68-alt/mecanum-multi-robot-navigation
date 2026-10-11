import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _render_robot(package_share, namespace):
    source = os.path.join(package_share, 'models', 'mecanum_robot.sdf.xacro')
    document = xacro.process_file(source, mappings={'namespace': namespace})
    path = os.path.join(tempfile.gettempdir(), f'mecanum_multi_nav_{namespace}.sdf')
    with open(path, 'w', encoding='utf-8') as output:
        output.write(document.toxml())
    return path


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_multi_nav')
    world = os.path.join(package_share, 'worlds', 'arena.sdf')
    robot_sdf = _render_robot(package_share, 'robot1')
    headless = LaunchConfiguration('headless')

    gazebo_gui = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r',
            '--render-engine', 'ogre',
            '--render-engine-gui', 'ogre',
            '--render-engine-server', 'ogre',
            world,
        ],
        output='screen',
        condition=UnlessCondition(headless),
    )
    gazebo_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', '--render-engine', 'ogre', world],
        output='screen',
        condition=IfCondition(headless),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'arena', '-file', robot_sdf, '-name', 'robot1',
            '-x', '-2.5', '-y', '-2.0', '-z', '0.02', '-Y', '0.0',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/robot1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/robot1/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/robot1/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/robot1/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[('/robot1/tf', '/tf')],
        parameters=[{
            'use_sim_time': True,
            'qos_overrides./robot1/cmd_vel.subscriber.reliability': 'reliable',
        }],
        output='screen',
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0.13', '0', '0', '0',
            'base_link', 'lidar_link',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    synthetic_lidar = Node(
        package='mecanum_multi_nav',
        executable='synthetic_lidar',
        namespace='robot1',
        parameters=[{
            'use_sim_time': True,
            'robot_name': 'robot1',
            'other_robot': '',
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Run only the Gazebo server without the GUI.',
        ),
        gazebo_gui,
        gazebo_server,
        bridge,
        lidar_tf,
        synthetic_lidar,
        TimerAction(period=2.0, actions=[spawn_robot]),
    ])
