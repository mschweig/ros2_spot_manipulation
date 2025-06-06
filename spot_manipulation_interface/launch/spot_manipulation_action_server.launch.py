from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spot_manipulation_interface',
            executable='spot_manipulation_action_server',
            name='spot_manipulation_action_server',
            output='screen',
        )
    ])