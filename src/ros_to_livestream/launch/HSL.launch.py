import json
import pathlib


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CONFIG_PATH = str(pathlib.Path(f"{get_package_share_directory('ros_to_livestream')}/config/settings.json"))


def generate_launch_description() -> LaunchDescription:
    with open(DEFAULT_CONFIG_PATH, "r") as config_file:
        configuration = json.load(config_file)
        

    # Define launch arguments that can be set from the command line using
    # "<name>:=<value>". If no value is give, the default is used.
    # configuration["message"] = LaunchConfiguration("message", default=configuration["message"])

    # Create nodes
    HSL_node = Node(
        package="ros_to_livestream",
        executable="HSL",
        name="ros_to_livestream",
        parameters=[configuration],
    )

    return LaunchDescription([
        HSL_node,
    ])
