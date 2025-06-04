import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare CLI arguments
    size_arg = DeclareLaunchArgument(
        'size',
        default_value='0.22',
        description='Size of the AprilTags (in meters)'
    )
    tag_family_arg = DeclareLaunchArgument(
        'tag_family',
        default_value='tag25h9',
        description='AprilTag family (e.g., tag36h11, tag25h9)'
    )
    backends_arg = DeclareLaunchArgument(
        'backends',
        default_value='PVA',
        description='VPI backends to use (e.g., CUDA, PVA, CPU)'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_rect',
        description='Input image topic to subscribe to'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='Input camera info topic to subscribe to'
    )

    # Substitutions
    size = LaunchConfiguration('size')
    tag_family = LaunchConfiguration('tag_family')
    backends = LaunchConfiguration('backends')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        parameters=[{
            'size': size,
            'max_tags': 64,
            'tile_size': 4,
            'tag_family': tag_family,
            'backends': backends,
        }],
        remappings=[
            ('image', image_topic),
            ('camera_info', camera_info_topic)
        ]
    )

    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[apriltag_node],
        output='screen'
    )

    return launch.LaunchDescription([
        size_arg,
        tag_family_arg,
        backends_arg,
        image_topic_arg,
        camera_info_topic_arg,
        container
    ])
