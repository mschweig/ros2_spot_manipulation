from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spot_manipulation_interface',
            executable='spot_picknplace_action_server',
            name='spot_picknplace_action_server',
            output='screen',
        )
    ])