import pytest
from rclpy.node import Node

from support.extended_node import ExtendedNode
from support.rclpy_mixin import RclpyMixin


def test_extended_node_extends_rclpy_node():
    assert issubclass(ExtendedNode, Node)


class TestGetParameterValue(RclpyMixin):
    def test_extracts_value_if_parameter_exists(self):
        parameter_name = "name"
        parameter_value = "value"
        node = ExtendedNode("package_name")
        node.declare_parameter(parameter_name, parameter_value)

        actual_value = node._get_parameter_value(parameter_name)

        assert actual_value == parameter_value

    def test_raises_value_error_if_parameter_not_found(self):
        parameter_name = "name"
        node = ExtendedNode("package_name")

        with pytest.raises(ValueError):
            node._get_parameter_value(parameter_name)
