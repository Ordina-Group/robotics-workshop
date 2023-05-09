import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

DEFAULT_CONFIG_PATH = str(pathlib.Path(f"{get_package_share_directory('camera_snap_shot')}/config/camera_snap_shot_settings.json"))


def generate_launch_description() -> LaunchDescription:
    # args that can be set from the command line or a default will be used
    config_file_arg = DeclareLaunchArgument(
        "config_file", default_value=TextSubstitution(text=DEFAULT_CONFIG_PATH)
    )
    # start a turtlesim_node in the turtlesim1 namespace
    camera_snap_shot = Node(
            package='camera_snap_shot',
            executable='take_snap_shot',
            name='sim',
        )

    return LaunchDescription([
        # launch_include,
        camera_snap_shot,
    ])
