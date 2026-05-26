from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    use_ros_streamer = LaunchConfiguration("use_ros_streamer")
    image_topic = LaunchConfiguration("image_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("visual_localization"),
                        "config",
                        "visual_localization.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("use_ros_streamer", default_value="false"),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/m100_1/sensors/pylon_camera_node/image_raw",
            ),
            Node(
                package="visual_localization",
                executable="visual_localization_node",
                name="visual_localization",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "use_ros_streamer": use_ros_streamer,
                        "image_topic": image_topic,
                    },
                ],
            ),
        ]
    )
