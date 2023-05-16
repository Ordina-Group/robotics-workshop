from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    server = LaunchConfiguration("server")
    name = LaunchConfiguration("name")

    # Create node
    publisher_node = Node(
        package="ros2_csi_camera_publish",
        executable="jetson",
        name="csi_camera_publish",
        parameters=[{"server": server},
                    {"name": name}],
    )

    return LaunchDescription([
        publisher_node,
    ])

