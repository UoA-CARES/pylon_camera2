import os
from timeit import default_timer as timer

import cv2
import rclpy
from rclpy.node import Node

from .camera import Camera


class CameraPublisher(Node):

    def __init__(self):
        super().__init__("camera_publisher")

        self.declare_parameter("camera_names", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("trigger_mode", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("pub_frequency", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("calibration_path", rclpy.Parameter.Type.STRING)
        self.declare_parameter("display", rclpy.Parameter.Type.BOOL)

        camera_names = (
            self.get_parameter("camera_names").get_parameter_value().string_array_value
        )

        trigger_mode = (
            self.get_parameter("trigger_mode").get_parameter_value().integer_value
        )

        pub_frequency = (
            self.get_parameter("pub_frequency").get_parameter_value().integer_value
        )

        calibration_path = (
            self.get_parameter("calibration_path").get_parameter_value().string_value
        )
        calibration_path = os.path.expanduser(calibration_path)

        display = self.get_parameter("display").get_parameter_value().bool_value

        self.get_logger().info(f"Camera names: {camera_names}")
        self.get_logger().info(f"Trigger mode: {trigger_mode}")
        self.get_logger().info(f"Publish frequency: {pub_frequency}")
        self.get_logger().info(f"Calibration path: {calibration_path}")
        self.get_logger().info(f"Display: {display}")

        self.cameras: dict[str:Camera] = {}

        master = True
        for camera_name in camera_names:
            self.get_logger().info(f"Initialising Camera name: {camera_name}")

            self.cameras[camera_name] = Camera(
                camera_name=camera_name,
                node=self,
                trigger_mode=trigger_mode,
                master=master,
                calibration_path=calibration_path,
                display=display,
            )

            master = False

        timer_period = 1.0 / pub_frequency  # hz to seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self) -> None:
        time_stamp = self.get_clock().now().to_msg()

        camera: Camera
        for camera in self.cameras.values():
            camera.trigger()

        # TODO thread this portion?
        for camera in self.cameras.values():
            camera.publish_data(time_stamp)

    def destroy_node(self) -> None:
        for camera in self.cameras.values():
            camera.close()

        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    rclpy.spin(camera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
