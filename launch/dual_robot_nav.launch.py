import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def _render_robot(package_share, namespace, native_lidar=False):
    source = os.path.join(package_share, 'models', 'mecanum_robot.sdf.xacro')
    document = xacro.process_file(source, mappings={
        'namespace': namespace,
        'native_lidar': 'true' if native_lidar else 'false',
    })
    suffix = '_native_lidar' if native_lidar else ''
    path = os.path.join(
        tempfile.gettempdir(), f'mecanum_multi_nav_{namespace}{suffix}.sdf')
    with open(path, 'w', encoding='utf-8') as output:
        output.write(document.toxml())
    return path


def _render_nav2_params(package_share, namespace, native_lidar=False):
    source = os.path.join(package_share, 'config', 'nav2_omni.yaml')
    with open(source, 'r', encoding='utf-8') as params:
        rendered = params.read().replace('__NS__', namespace)
    if native_lidar:
        rendered = rendered.replace('scan_topic: scan_static', 'scan_topic: scan')
    suffix = '_native_lidar' if native_lidar else ''
    path = os.path.join(
        tempfile.gettempdir(), f'nav2_omni_{namespace}{suffix}.yaml')
    with open(path, 'w', encoding='utf-8') as output:
        output.write(rendered)
    return path


def _render_world(package_share, engine):
    source = os.path.join(package_share, 'worlds', 'arena.sdf')
    with open(source, 'r', encoding='utf-8') as world_file:
        rendered = world_file.read().replace(
            '<render_engine>ogre</render_engine>',
            f'<render_engine>{engine}</render_engine>')
    path = os.path.join(tempfile.gettempdir(), f'arena_{engine}.sdf')
    with open(path, 'w', encoding='utf-8') as output:
        output.write(rendered)
    return path


def _robot_bridge(namespace, condition=None):
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{namespace}_bridge',
        arguments=[
            f'/{namespace}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            f'/{namespace}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{namespace}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/{namespace}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            f'/{namespace}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        parameters=[{
            'use_sim_time': True,
            f'qos_overrides./{namespace}/cmd_vel.subscriber.reliability': 'reliable',
        }],
        condition=condition,
        output='screen',
    )


def _static_lidar_tf(namespace):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=namespace,
        name='lidar_static_tf',
        arguments=[
            '0', '0', '0.13', '0', '0', '0',
            f'{namespace}/base_link', f'{namespace}/lidar_link',
        ],
        remappings=[('/tf_static', 'tf_static')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )


def _synthetic_lidar(namespace, other_namespace, native_lidar, scenario):
    return Node(
        package='mecanum_multi_nav',
        executable='synthetic_lidar',
        namespace=namespace,
        name='synthetic_lidar',
        parameters=[{
            'use_sim_time': True,
            'robot_name': namespace,
            'other_robot': other_namespace,
            'scenario': scenario,
        }],
        output='screen',
        condition=UnlessCondition(native_lidar),
    )


def _nav2_bringup(bringup_dir, namespace, map_file, params_file, condition):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'True',
            'slam': 'False',
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': params_file,
            'autostart': 'True',
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items(),
        condition=condition,
    )


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_multi_nav')
    bringup_dir = get_package_share_directory('nav2_bringup')
    world_ogre = _render_world(package_share, 'ogre')
    world_ogre2 = _render_world(package_share, 'ogre2')
    default_world = os.path.join(package_share, 'worlds', 'arena.sdf')
    default_map = os.path.join(package_share, 'maps', 'arena.yaml')
    world_file = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
    scenario = LaunchConfiguration('scenario')
    headless = LaunchConfiguration('headless')
    native_lidar = LaunchConfiguration('native_lidar')
    software_rendering = LaunchConfiguration('software_rendering')

    gui_synthetic = IfCondition(PythonExpression([
        "'", headless, "' == 'false' and '", native_lidar, "' == 'false'",
    ]))
    gui_native_software = IfCondition(PythonExpression([
        "'", headless, "' == 'false' and '", native_lidar,
        "' == 'true' and '", software_rendering, "' == 'true'",
    ]))
    gui_native_hardware = IfCondition(PythonExpression([
        "'", headless, "' == 'false' and '", native_lidar,
        "' == 'true' and '", software_rendering, "' == 'false'",
    ]))
    server_synthetic = IfCondition(PythonExpression([
        "'", headless, "' == 'true' and '", native_lidar, "' == 'false'",
    ]))
    server_native_software = IfCondition(PythonExpression([
        "'", headless, "' == 'true' and '", native_lidar,
        "' == 'true' and '", software_rendering, "' == 'true'",
    ]))
    server_native_hardware = IfCondition(PythonExpression([
        "'", headless, "' == 'true' and '", native_lidar,
        "' == 'true' and '", software_rendering, "' == 'false'",
    ]))

    gazebo_gui = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r',
            '--render-engine', 'ogre',
            '--render-engine-gui', 'ogre',
            '--render-engine-server', 'ogre',
            world_file,
        ], output='screen',
        condition=gui_synthetic,
    )
    gazebo_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', '--render-engine', 'ogre', world_file],
        output='screen',
        condition=server_synthetic,
    )
    gazebo_gui_native_software = ExecuteProcess(
        cmd=['bash', '-lc',
             'export LIBGL_ALWAYS_SOFTWARE=1; '
             'export GALLIUM_DRIVER=llvmpipe; '
             'unset MESA_D3D12_DEFAULT_ADAPTER_NAME; '
             'export QT_QPA_PLATFORM=xcb; '
             f'exec ign gazebo -r {world_ogre2}'],
        output='screen', condition=gui_native_software,
    )
    gazebo_gui_native_hardware = ExecuteProcess(
        cmd=['bash', '-lc',
             'unset LIBGL_ALWAYS_SOFTWARE; '
             'unset GALLIUM_DRIVER; '
             'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA; '
             'export QT_QPA_PLATFORM=xcb; '
             f'exec ign gazebo -r {world_ogre2}'],
        output='screen', condition=gui_native_hardware,
    )
    gazebo_server_native_software = ExecuteProcess(
        cmd=['bash', '-lc',
             'export LIBGL_ALWAYS_SOFTWARE=1; '
             'export GALLIUM_DRIVER=llvmpipe; '
             'unset MESA_D3D12_DEFAULT_ADAPTER_NAME; '
             f'exec ign gazebo -r -s {world_ogre2}'],
        output='screen', condition=server_native_software,
    )
    gazebo_server_native_hardware = ExecuteProcess(
        cmd=['bash', '-lc',
             'unset LIBGL_ALWAYS_SOFTWARE; '
             'unset GALLIUM_DRIVER; '
             'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA; '
             f'exec ign gazebo -r -s {world_ogre2}'],
        output='screen', condition=server_native_hardware,
    )

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    robot1_sdf = _render_robot(package_share, 'robot1')
    robot2_sdf = _render_robot(package_share, 'robot2')
    robot1_native_sdf = _render_robot(package_share, 'robot1', True)
    robot2_native_sdf = _render_robot(package_share, 'robot2', True)
    robot1_params = _render_nav2_params(package_share, 'robot1')
    robot2_params = _render_nav2_params(package_share, 'robot2')
    robot1_native_params = _render_nav2_params(package_share, 'robot1', True)
    robot2_native_params = _render_nav2_params(package_share, 'robot2', True)
    spawn_robot1 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'arena', '-file', robot1_sdf, '-name', 'robot1',
                   '-x', '-3.0', '-y', '-2.0', '-z', '0.02', '-Y', '0.0'],
        output='screen',
        condition=UnlessCondition(native_lidar),
    )
    spawn_robot2 = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'arena', '-file', robot2_sdf, '-name', 'robot2',
                   '-x', '-3.0', '-y', '2.0', '-z', '0.02', '-Y', '0.0'],
        output='screen',
        condition=UnlessCondition(native_lidar),
    )
    spawn_robot1_native = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'arena', '-file', robot1_native_sdf,
                   '-name', 'robot1', '-x', '-3.0', '-y', '-2.0',
                   '-z', '0.02', '-Y', '0.0'],
        output='screen', condition=IfCondition(native_lidar),
    )
    spawn_robot2_native = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'arena', '-file', robot2_native_sdf,
                   '-name', 'robot2', '-x', '-3.0', '-y', '2.0',
                   '-z', '0.02', '-Y', '0.0'],
        output='screen', condition=IfCondition(native_lidar),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run only the Gazebo server without the GUI.',
        ),
        DeclareLaunchArgument(
            'native_lidar', default_value='false',
            description='Use the Gazebo GPU lidar with its supported Ogre2 renderer.',
        ),
        DeclareLaunchArgument(
            'software_rendering', default_value='true',
            description='Use reliable CPU Mesa for Ogre2; false tries WSLg NVIDIA.',
        ),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='SDF world used by the synthetic-lidar simulation.',
        ),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Known occupancy map loaded by both Nav2 stacks.',
        ),
        DeclareLaunchArgument(
            'scenario', default_value='arena',
            description='Geometry set used by the deterministic lidar.',
        ),
        gazebo_gui,
        gazebo_server,
        gazebo_gui_native_software,
        gazebo_gui_native_hardware,
        gazebo_server_native_software,
        gazebo_server_native_hardware,
        clock_bridge,
        _robot_bridge('robot1', UnlessCondition(native_lidar)),
        _robot_bridge('robot2', UnlessCondition(native_lidar)),
        _static_lidar_tf('robot1'),
        _static_lidar_tf('robot2'),
        _synthetic_lidar('robot1', 'robot2', native_lidar, scenario),
        _synthetic_lidar('robot2', 'robot1', native_lidar, scenario),
        Node(
            package='mecanum_multi_nav', executable='tf_aggregator',
            name='tf_aggregator', output='screen',
        ),
        Node(
            package='mecanum_multi_nav', executable='robot_markers',
            name='robot_markers', parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        TimerAction(period=2.0, actions=[
            spawn_robot1, spawn_robot2,
        ]),
        # Let Ogre2 finish world texture initialization before dynamically
        # inserting the first GPU lidar, then serialize the second insertion.
        TimerAction(period=10.0, actions=[
            spawn_robot1_native,
        ]),
        TimerAction(period=18.0, actions=[
            spawn_robot2_native,
        ]),
        # Do not subscribe to a freshly inserted Ogre2 GPU lidar during its
        # first render-target frames; Fortress may call copyTo too early.
        TimerAction(period=22.0, actions=[
            _robot_bridge('robot1', IfCondition(native_lidar)),
            _robot_bridge('robot2', IfCondition(native_lidar)),
        ]),
        TimerAction(
            period=5.0,
            actions=[
                _nav2_bringup(
                    bringup_dir, 'robot1', map_file, robot1_params,
                    UnlessCondition(native_lidar)),
                _nav2_bringup(
                    bringup_dir, 'robot2', map_file, robot2_params,
                    UnlessCondition(native_lidar)),
            ],
        ),
        TimerAction(
            period=25.0,
            actions=[
                _nav2_bringup(
                    bringup_dir, 'robot1', map_file, robot1_native_params,
                    IfCondition(native_lidar)),
                _nav2_bringup(
                    bringup_dir, 'robot2', map_file, robot2_native_params,
                    IfCondition(native_lidar)),
            ],
        ),
    ])
