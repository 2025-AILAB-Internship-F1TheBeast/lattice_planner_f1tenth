from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('lattice_planner_pkg')
    
    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'planner_config.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Use simulation mode (ego_racecar/odom) if true, real car mode (/pf/pose/odom) if false'
    )
    
    # Obstacle detection topics
    obstacles_topic_arg = DeclareLaunchArgument(
        'obstacles_topic',
        default_value='/detected_obstacles',
        description='Topic name for obstacle array messages'
    )
    
    updated_map_topic_arg = DeclareLaunchArgument(
        'updated_map_topic', 
        default_value='/updated_map',
        description='Topic name for updated occupancy grid'
    )
    
    markers_topic_arg = DeclareLaunchArgument(
        'markers_topic',
        default_value='/obstacle_markers', 
        description='Topic name for visualization markers'
    )
    
    # Lattice planner node for simulation mode
    lattice_planner_sim_node = Node(
        package='lattice_planner_pkg',
        executable='lattice_planner_node',
        name='lattice_planner',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/odom', 'ego_racecar/odom'),
            ('/scan', '/scan'),
            ('/map', '/updated_map'),
        ],
        condition=IfCondition(LaunchConfiguration('sim_mode'))
    )
    
    # Lattice planner node for real car mode
    lattice_planner_real_node = Node(
        package='lattice_planner_pkg',
        executable='lattice_planner_node',
        name='lattice_planner',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/odom', '/pf/pose/odom'),
            ('/scan', '/scan'),
            ('/map', '/updated_map'),
        ],
        condition=UnlessCondition(LaunchConfiguration('sim_mode'))
    )
    
    # Obstacle subscriber node (runs in both sim and real modes)
    obstacle_subscriber_node = Node(
        package='lattice_planner_pkg',
        executable='obstacle_subscriber_node',
        name='obstacle_subscriber',
        output='screen',
        parameters=[{
            'obstacles_topic': LaunchConfiguration('obstacles_topic'),
            'updated_map_topic': LaunchConfiguration('updated_map_topic'),
            'markers_topic': LaunchConfiguration('markers_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        sim_mode_arg,
        obstacles_topic_arg,
        updated_map_topic_arg, 
        markers_topic_arg,
        lattice_planner_sim_node,
        lattice_planner_real_node,
        obstacle_subscriber_node,
    ])