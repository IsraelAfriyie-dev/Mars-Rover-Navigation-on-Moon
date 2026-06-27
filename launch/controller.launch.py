"""
Controller Launch File for Lunar Rover Navigation.

Launches the local control stack:
- DWA controller
- Pure Pursuit controller
- Velocity command publishing
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    controller_type = LaunchConfiguration('controller_type', default='pure_pursuit')
    cmd_topic = LaunchConfiguration('cmd_topic', default='/cmd_vel')
    odom_topic = LaunchConfiguration('odom_topic', default='/diff_drive_controller/odom')
    path_topic = LaunchConfiguration('path_topic', default='/planned_path')
    
    # Controller parameters
    max_velocity = LaunchConfiguration('max_velocity', default='0.5')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity', default='1.0')
    lookahead_distance = LaunchConfiguration('lookahead_distance', default='0.6')
    desired_velocity = LaunchConfiguration('desired_velocity', default='0.3')
    
    # Set global use_sim_time
    set_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='pure_pursuit',
        choices=['dwa', 'pure_pursuit'],
        description='Type of local controller to use'
    )
    
    declare_cmd_topic = DeclareLaunchArgument(
        'cmd_topic',
        default_value='/cmd_vel',
        description='Output command velocity topic'
    )
    
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/diff_drive_controller/odom',
        description='Odometry input topic'
    )
    
    declare_max_velocity = DeclareLaunchArgument(
        'max_velocity',
        default_value='0.5',
        description='Maximum velocity in m/s'
    )
    
    declare_lookahead = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='0.6',
        description='Lookahead distance for pure pursuit in meters'
    )
    
    # Controller node
    controller_node = Node(
        package='moon_rover_navigation',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'controller_type': controller_type,
            'cmd_topic': cmd_topic,
            'odom_topic': odom_topic,
            'path_topic': path_topic,
            'max_velocity': max_velocity,
            'max_angular_velocity': max_angular_velocity,
            'lookahead_distance': lookahead_distance,
            'desired_velocity': desired_velocity,
        }],
    )
    
    # Localization node
    localization_node = Node(
        package='moon_rover_navigation',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'publish_tf': True,
        }],
    )
    
    return LaunchDescription([
        set_sim_time,
        declare_use_sim_time,
        declare_controller_type,
        declare_cmd_topic,
        declare_odom_topic,
        declare_max_velocity,
        declare_lookahead,
        controller_node,
        localization_node,
    ])