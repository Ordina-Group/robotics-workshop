import rclpy


class RclpyMixin:
    @staticmethod
    def setup_method():
        rclpy.init()

    @staticmethod
    def teardown_method():
        rclpy.shutdown()
