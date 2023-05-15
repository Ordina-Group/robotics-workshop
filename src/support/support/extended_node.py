"""Extend the standard ROS2 Node to improve ease-of-use.

The ExtendedNode-class adds a _extract_parameter_value()-method, which can be
used to easily extract the value of a parameter.

"""
from typing import Any

from rclpy.node import Node
from rclpy.parameter import Parameter


class ExtendedNode(Node):
    def __init__(self, package_name: str):
        super().__init__(package_name)

    def _get_parameter_value(self, parameter_name: str) -> Any:
        """Return the value of a parameter.

        Raises a ValueError if a parameter cannot be found.

        """
        if not self.has_parameter(parameter_name):
            self.declare_parameter(parameter_name)

        parameter = self.get_parameter(parameter_name)
        parameter_value = parameter.value

        if parameter.type_ == Parameter.Type.NOT_SET:
            raise ValueError(f"parameter '{parameter_name}' is not set")

        return parameter_value
