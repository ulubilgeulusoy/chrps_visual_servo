from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('chrps_visual_servo')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='172.16.0.2',
            description='IP address of the Franka robot controller'
        ),
        
        DeclareLaunchArgument(
            'tag_size',
            default_value='0.05',
            description='Size of the AprilTag in meters'
        ),
        
        DeclareLaunchArgument(
            'desired_factor',
            default_value='5.0',
            description='Distance factor for desired Z position (tag_size * desired_factor)'
        ),
        
        DeclareLaunchArgument(
            'adaptive_gain',
            default_value='true',
            description='Enable adaptive gain control'
        ),
        
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Enable verbose output'
        ),
        
        DeclareLaunchArgument(
            'show_display',
            default_value='false',
            description='Show visual servoing display window'
        ),
        
        Node(
            package='chrps_visual_servo',
            executable='chrps_visual_servo_node',
            name='chrps_visual_servo_node',
            output='screen',
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'tag_size': LaunchConfiguration('tag_size'),
                'desired_factor': LaunchConfiguration('desired_factor'),
                'adaptive_gain': LaunchConfiguration('adaptive_gain'),
                'verbose': LaunchConfiguration('verbose'),
                'show_display': LaunchConfiguration('show_display'),
                'emc_config_path': 'config/eMc.yaml'
            }]
        )
    ])