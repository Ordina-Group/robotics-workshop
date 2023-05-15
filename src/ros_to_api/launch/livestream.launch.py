import json
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter

DEFAULT_CONFIG_PATH = str(pathlib.Path(f"{get_package_share_directory('ros_to_api')}/config/settings.json"))


def generate_launch_description() -> LaunchDescription:
    with open(DEFAULT_CONFIG_PATH, "r") as json_file:
        configuration = json.load(json_file)
    # configuration["message"] = LaunchConfiguration("message")

    # Create node
    publisher_node = Node(
        package="ros_to_api",
        executable="livestream",
        name="ros_to_api",
        parameters=[configuration],
    )

    return LaunchDescription([
        publisher_node,
    ])
