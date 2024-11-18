import time

import cv2
import numpy as np
import yaml
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from pypylon import pylon
from rclpy.node import Node, Publisher
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


# TODO shift to cares_lib_ros2
def rectify_image(image: np.ndarray, camera_info: CameraInfo) -> np.ndarray:
    # Get the camera matrix and distortion coefficients
    K = np.array(camera_info.k).reshape((3, 3))
    D = np.array(camera_info.d)

    # Get the rectification matrix
    R = np.array(camera_info.r).reshape((3, 3))
    R = np.eye(3) if np.count_nonzero(R) == 0 else R  # Non-Stereo use Identity

    # Get the new camera matrix
    P = np.array(camera_info.p).reshape((3, 4))
    P = K if np.count_nonzero(P) == 0 else P  # Non-Stereo use K

    # Get the image size
    h, w, _ = image.shape

    # Compute the rectification map
    map_one, map_two = cv2.initUndistortRectifyMap(K, D, R, P, (w, h), cv2.CV_32FC1)

    # Rectify the image
    image_rect = cv2.remap(image, map_one, map_two, cv2.INTER_LINEAR)

    return image_rect


def get_camera_by_name(camera_name: str) -> pylon.InstantCamera:
    # Get all attached devices
    devices = pylon.TlFactory.GetInstance().EnumerateDevices()
    # Find the camera with the specified name
    for device in devices:
        if device.GetUserDefinedName() == camera_name:
            # Instantiate the camera by device info
            camera = pylon.InstantCamera(
                pylon.TlFactory.GetInstance().CreateDevice(device)
            )
            return camera

    # If no camera with the specified name was found, raise an error
    raise ValueError(f"Camera with name '{camera_name}' not found")


# TODO shift to cares_lib_ros2
def map_to_camera_info(yaml_map: dict) -> CameraInfo:
    camera_info = CameraInfo()
    camera_info.header.frame_id = yaml_map["header"]["frame_id"]
    camera_info.height = yaml_map["height"]
    camera_info.width = yaml_map["width"]
    camera_info.distortion_model = yaml_map["distortion_model"]
    camera_info.d = yaml_map["D"]
    camera_info.k = yaml_map["K"]
    camera_info.r = yaml_map["R"]
    camera_info.p = yaml_map["P"]

    # TODO read these from the camera directly
    # camera_info.binning_x = s_map["binning_x"]
    # camera_info.binning_y = s_map["binning_y"]

    # camera_info.roi.x_offset = s_map["roi"]["x_offset"]
    # camera_info.roi.y_offset = s_map["roi"]["y_offset"]
    # camera_info.roi.height = s_map["roi"]["height"]
    # camera_info.roi.width = s_map["roi"]["width"]
    # camera_info.roi.do_rectify = s_map["roi"]["do_rectify"]

    return camera_info


def load_camerainfo(filepath: str) -> CameraInfo:
    with open(filepath) as file:
        s_map = yaml.load(file, Loader=yaml.Loader)
        return map_to_camera_info(s_map)


class Camera:
    def __init__(
        self,
        camera_name: str,
        node: Node,
        trigger_mode: int = 0,
        master: bool = False,
        calibration_path: str = "",
    ):
        self.camera_name = camera_name
        self.camera = get_camera_by_name(camera_name)

        self.node = node

        self.trigger_mode = trigger_mode
        self.master = master

        self.calibration_path = calibration_path

        # Open the camera before accessing any parameters
        self.camera.Open()

        # Have to set GrabStrategy before the other parameters
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

        self.set_trigger_mode(self.trigger_mode, self.master)

        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        # Set up OpenCV bridge to convert to ROS 2 Image messages
        self.bridge = CvBridge()

        self.image_publisher = self.node.create_publisher(
            Image, f"{camera_name}/image_color", 1
        )

        self.camera_info: CameraInfo = CameraInfo()
        self.info_publisher: Publisher = None
        self.image_rect_publisher: Publisher = None

        self.info_publisher = self.node.create_publisher(
            CameraInfo, f"{camera_name}/camera_info", 1
        )

        self.calibrated = False
        if self.calibration_path != "":
            self.calibrated = True
            self.camera_info = load_camerainfo(
                f"{self.calibration_path}{self.camera_name}_calibration.yaml"
            )

            self.image_rect_publisher = self.node.create_publisher(
                Image, f"{camera_name}/image_rect_color", 1
            )

    def _toggle_trigger_pin(self) -> None:
        # //Use lineselector to select a GPIO port.
        # this->camera.LineSelector.SetValue(LineSelector_Line4);
        self.camera.LineSelector.SetValue("Line4")

        # //Use linemode to set the GPIO port to input or output
        # this->camera.LineMode.SetValue(LineMode_Output);
        self.camera.LineMode.SetValue("Output")

        # // Use the line source to choose a UserOutput channel (essentially connecting a Line (GPIO port) to a UserOutput channel).
        # this->camera.LineSource.SetValue(LineSource_UserOutput3);
        self.camera.LineSource.SetValue("UserOutput3")

        # //Use the UserOutputSelector to select the userOutput channel selected above
        # this->camera.UserOutputSelector.SetValue(UserOutputSelector_UserOutput3);
        self.camera.UserOutputSelector.SetValue("UserOutput3")

        # //Set the value of the user output channel. True and false enable a rising edge.
        # if(this->camera.LineStatus.GetValue()){
        if self.camera.LineStatus.GetValue():
            self.camera.UserOutputValue.SetValue(False)
            time.sleep(0.001)
            self.camera.UserOutputValue.SetValue(True)
            time.sleep(0.001)
            self.camera.UserOutputValue.SetValue(False)
        elif not self.camera.LineStatus.GetValue():
            self.camera.UserOutputValue.SetValue(True)
            time.sleep(0.001)
            self.camera.UserOutputValue.SetValue(False)

    def _hardware_trigger_mode(self) -> None:
        # Set the camera to hardware trigger mode

        self.camera.LineSelector.SetValue("Line3")
        self.camera.LineMode.SetValue("Input")

        # Set the trigger mode to On
        self.camera.TriggerMode.SetValue("On")

        # //Set the line source for the hardware trigger to be GPIO line two
        self.camera.TriggerSource.SetValue("Line3")

        # //Set trigger on rising edge
        self.camera.TriggerActivation.SetValue("RisingEdge")

    def _exposure_trigger_mode(self) -> None:
        # Set the camera to exposure trigger mode
        if self.master:
            self.camera.TriggerMode.SetValue("On")
            self.camera.TriggerSource.SetValue("Software")
            self.camera.LineSelector.SetValue("Line3")
            self.camera.LineMode.SetValue("Output")
            self.camera.LineSource.SetValue("ExposureActive")
        else:
            self.camera.TriggerMode.SetValue("On")
            self.camera.TriggerSource.SetValue("Line3")

    def _software_trigger_mode(self) -> None:
        # Set the camera to software trigger mode
        self.camera.TriggerMode.SetValue("On")
        self.camera.TriggerSource.SetValue("Software")

    def set_trigger_mode(self, trigger_mode: int, master: bool = False) -> None:
        self.trigger_mode = trigger_mode
        self.master = master

        if trigger_mode == 0:
            self.camera.TriggerMode.SetValue("Off")
        elif trigger_mode == 1:  # software trigger
            self._software_trigger_mode()
        elif trigger_mode == 2:  # hardware trigger
            self._hardware_trigger_mode()
        elif trigger_mode == 3:  # exposure trigger
            self._exposure_trigger_mode()
        else:
            raise ValueError(f"Invalid trigger mode: {trigger_mode}")

    def trigger(self) -> None:
        if self.trigger_mode == 1:
            self.camera.ExecuteSoftwareTrigger()
        elif self.trigger_mode == 2:
            if self.master:
                self._toggle_trigger_pin()
        elif self.trigger_mode == 3:
            if self.master:
                self.camera.ExecuteSoftwareTrigger()

    def get_image(
        self, timeout: int = 5000, display: bool = True
    ) -> tuple[np.ndarray, np.ndarray | None]:
        try:
            grab_result = self.camera.RetrieveResult(
                timeout, pylon.TimeoutHandling_ThrowException
            )
        except Exception:
            self.node.get_logger().error(
                f"Camera: {self.camera_name} failed to capture image due to timeout - using trigger mode: {self.trigger_mode}"
            )
            exit()

        if grab_result.GrabSucceeded():
            image = self.converter.Convert(grab_result)
            image_bgr = image.GetArray()
            grab_result.Release()

            rec_image_bgr = None
            if self.calibrated:
                rec_image_bgr = rectify_image(image_bgr, self.camera_info)

            if display:
                cv2.imshow(f"image-{self.camera_name}", image_bgr)

                if rec_image_bgr is not None:
                    cv2.imshow(f"image-rect-{self.camera_name}", rec_image_bgr)

                cv2.waitKey(10)

            return image_bgr, rec_image_bgr

        raise ValueError(
            f"Error {self.camera_name}: {grab_result.GetErrorCode()} {grab_result.GetErrorDescription()}"
        )

    def publish_data(self, time_stamp: Time, display: bool = True) -> None:
        image_bgr, rec_image_bgr = self.get_image(display=display)

        # Set the header information (optional, e.g., setting frame_id or timestamp)
        header = Header()
        header.stamp = time_stamp
        header.frame_id = f"{self.camera_name}_camera_frame"

        # Convert to ROS 2 Image message
        ros_image_msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")

        self.camera_info.header = header
        ros_image_msg.header = header

        # Publish the data
        self.image_publisher.publish(ros_image_msg)
        self.info_publisher.publish(self.camera_info)

        if rec_image_bgr is not None:
            ros_rec_image_msg = self.bridge.cv2_to_imgmsg(
                rec_image_bgr, encoding="bgr8"
            )
            ros_rec_image_msg.header = header

            self.image_rect_publisher.publish(ros_rec_image_msg)

    def close(self) -> None:
        self.camera.StopGrabbing()
        self.camera.Close()
