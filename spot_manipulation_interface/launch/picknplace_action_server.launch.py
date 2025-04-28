from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value='/ros2_ws/src/robot.yaml',
            description='Full path to Spot param YAML file'
        ),
        Node(
            package='spot_manipulation_interface',
            executable='spot_grasp_action_server',
            name='spot_grasp_action_server',
            output='screen',
            parameters=[LaunchConfiguration('param_file')]
        )
    ])