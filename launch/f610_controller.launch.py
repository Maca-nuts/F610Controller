"""Launch the controller with per-motor gains from a ROS parameter YAML."""
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = get_package_share_path('f610_controller') / 'config' / 'f610_controller.yaml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=str(default_config),
            description='Path to the controller ROS parameter YAML'),
        Node(
            package='f610_controller', executable='controller',
            name='f610_control', output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
