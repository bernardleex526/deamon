import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('jie_deamon')
    config = LaunchConfiguration('config')
    boolean = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'm20.yaml')),
        DeclareLaunchArgument('cloud_topic', default_value='/LIDAR/POINTS'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('commissioned', default_value='false'),
        DeclareLaunchArgument('enable_web', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('operator_token_file', default_value=os.environ.get('M20_OPERATOR_TOKEN_FILE', '')),
        DeclareLaunchArgument('operator_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('operator_port', default_value='8080'),
        DeclareLaunchArgument('android_port', default_value='8889'),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='m20_cloud_to_scan', output='screen',
             parameters=[config, {'use_sim_time': boolean('use_sim_time')}],
             remappings=[('cloud_in', LaunchConfiguration('cloud_topic')), ('scan', '/m20/scan')]),
        Node(package='jie_deamon', executable='robot_nexus', name='robot_nexus', output='screen',
             parameters=[config, {'enable_web': False,
                                  'operator_control': boolean('enable_web'),
                                  'enable_android': False, 'enable_actions': False,
                                  'web_root': os.path.join(share, 'web'),
                                  'use_sim_time': boolean('use_sim_time')}],
             remappings=[('/scan', '/m20/scan'), ('/cmd_vel', '/m20/cmd_vel_raw'),
                         ('/d1_cmd', '/m20/unsupported_d1_actions')]),
        Node(package='jie_deamon', executable='m20_bridge', name='m20_bridge', output='screen',
             parameters=[config, {'dry_run': boolean('dry_run'),
                                  'commissioned': boolean('commissioned'),
                                  'enable_operator': boolean('enable_web'),
                                  'operator_token_file': LaunchConfiguration('operator_token_file'),
                                  'operator_page': os.path.join(share, 'web', 'm20.html'),
                                  'operator_host': LaunchConfiguration('operator_host'),
                                  'operator_port': ParameterValue(LaunchConfiguration('operator_port'), value_type=int),
                                  'android_port': ParameterValue(LaunchConfiguration('android_port'), value_type=int),
                                  'use_sim_time': boolean('use_sim_time')}]),
    ])
