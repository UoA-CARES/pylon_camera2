# Pylon Camera ROS2 Package
Welcome to the Pylon Camera ROS2 package. This package provides a ROS2 interface for Basler cameras using the Pylon SDK. It allows for easy integration of Basler cameras into your ROS2 applications, enabling image capture, processing, and streaming within the ROS2 ecosystem. Fully supports hardware synchronized stereo cameras. 

## Installation Instructions
You will first need to install the Pylon software (>= 8.0) from [here](https://www.baslerweb.com/en/software/pylon/) 

`git clone`the repository into your desired ros2 workspace directory on your local machine.

Run `pip3 install -r requirements' in the **root directory** of this package.

`colcon build` your ros2 workspace.

# Usage
This node can handle N number of basler cameras connected to the same machine. The recommneded utilisation of this package is via the launch file.

```
ros2 launch pylon_camera2 camera.launch.py
```

## Run Parameters
The node can be configured with the parameters below.

**camera_names**: List of names for the cameras - first in the list is considered the `master` camera for synchronized cameras. See the section below for how to provide custom names to the cameras.

**trigger_mode**: Mode for triggering the cameras. Options are:

0. Continuous Async Mode
1. Software Trigger Async Mode
2. Hardware Sync Trigger
3. Exposure Sync Trigger

Further details on the synchronization methods are below.

**pub_frequency**: Desired frequency at which images are published.

**calibration_path**: Path to the camera calibration files (default no calibration "") - expected to have naming format `camera_name_calibration.yaml'.

**display**: Boolean to enable or disable image display (default True).

## Publishers
For each camera_name provided the image topics below will be published as below:

- `/camera_name/image_color`: Publishes color image data from the camera.
- `/camera_name/camera_info`: Publishes camera calibration information if provided
- `/camera_name/image_rect_color`': Rectified/Undistored image if calibration data is provided

## Configure Camera Names
This package utilises the custom name for the cameras to connect to the correct cameras. The instructions below explain how to name the cameras with unique names. 

1) Start up pylon viwer (located at /pylon/bin/pylonviewer)

2) Double click the camera in "Devices"

3) In the "Features - All" tab, find "Device control" and modify the "Device User ID" to the desired name

4) Still under "Device control", execute the "Device reset" and refresh the devices to see the change take effect.

## Trigger Modes
Pin out on Basler camera USB 3.0 GPIO. Pin 5 is 12 o'clock with USB input at 6 o'clock, numbers then rotate clockwise as shown in the figure below.

```
  Pin 6 -> DCon GND
  Pin 5 -> Opto GND
  Pin 4 -> Line 2 : Opto
  Pin 3 -> Line 4 : Dcon Output
  Pin 2 -> Line 1 : Opto
  Pin 1 -> Line 3 : Dcon Input
```

![Alt text](docs/basler-pin-out.png?raw=true "Pin-out")

### Continuous Async Mode (0)
Cameras are run in continuous capture mode without any triggering method. 

### Software Trigger Async Mode (1)
Asynchronous mode will run all cameras via the software triggers independently without the need for hardware synchronization. This does not produce syncronised images due to software delays processing each camera one by one. 

### Pylon Camera Exposure Sync Mode (2)
Similar to the Pin trigger but the master camera is software triggered. When the master camera is ready to capture (ready to expose) a pin is set high triggering the other cameras at the same time. 

The trigger output on the first camera is wired into the trigger input on the other cameras as in the diagram below. 

The hard-coded trigger pin is: 
```
Pin 1 -> Line 3.
```

![Alt text](docs/sync-mode-1.png?raw=true "Pin-out")

### Pylon Camera Pin Mode (3)
Pin mode sets a pin on each camera as an input that will trigger the cameras on a rising edge. The master camera has a pin setup that can be toggled via software to trigger the capture on each camera. The master camera toggles the trigger pin to trigger all cameras.
 
The trigger output on the master camera is wired into the trigger input for all cameras - including itself.

Trigger Output - Master: 
```
Pin 3 -> Line 4 : Dcon Output
```

Trigger Input:
```
Pin 1 -> Line 3 : Dcon Input
```

![Alt text](docs/sync-mode-2.png?raw=true "Pin-out")

## Calibration Information
Camera calibration information comes in two forms. The first is mono camera calibration which contains only the intrinsic calibration information. The second is stereo camera information for stereo pairs. This package can handle both forms of calibration information and will publish the rectified/undistored images accordingly. 