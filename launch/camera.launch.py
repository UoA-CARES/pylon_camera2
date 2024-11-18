from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_camera_names = DeclareLaunchArgument(
        "camera_names",
        default_value='["left","right"]',
        description="Names of the camera to use - first will be defaulted to master if trigger mode other than 0 is set",
    )

    declare_trigger_mode = DeclareLaunchArgument(
        "trigger_mode",
        default_value="3",
        description="Trigger Mode (0: Continuous Async Mode, 1: Software Aysnc Trigger 2: Hardware Sync Trigger 3: Exposure Sync Trigger) - default is 3",
    )

    declare_pub_frequency = DeclareLaunchArgument(
        "pub_frequency",
        default_value="30",
        description="Camera publish rate (hz) - will try to opreate at this frequency but will be limited by camera setup",
    )

    declare_calibration_path = DeclareLaunchArgument(
        "calibration_path",
        default_value="",
        description="Path to camera calibration files - expected to contain camera_name.yaml files",
    )

    declare_display = DeclareLaunchArgument(
        "display", default_value="true", description="Display images"
    )

    camera_names = LaunchConfiguration(declare_camera_names.name)
    trigger_mode = LaunchConfiguration(declare_trigger_mode.name)
    pub_frequency = LaunchConfiguration(declare_pub_frequency.name)
    calibration_path = LaunchConfiguration(declare_calibration_path.name)
    display = LaunchConfiguration(declare_display.name)

    return LaunchDescription(
        [
            declare_camera_names,
            declare_trigger_mode,
            declare_pub_frequency,
            declare_calibration_path,
            declare_display,
            Node(
                package="pylon_camera2",
                executable="pylon_camera_node",
                name="pylon_camera_node",
                output="screen",
                parameters=[
                    {
                        declare_camera_names.name: camera_names,
                        declare_trigger_mode.name: trigger_mode,
                        declare_pub_frequency.name: pub_frequency,
                        declare_calibration_path.name: calibration_path,
                        declare_display.name: display,
                    }
                ],
            ),
        ]
    )
