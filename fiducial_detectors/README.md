# `fiducial_detectors` 
ROS package wrapping some fiducial detectors in a common structure and publishing the detections in a common framework. Note that the fiducial detectors are based on the [ROS Camera Subscriber](https://docs.ros.org/en/api/image_transport/html/classimage__transport_1_1CameraSubscriber.html) class, which requires the image data to be published using this format: 

```
/common/camera/namespace/image_topic
/common/camera/namespace/camera_info
```

That is, the `image_topic` can be whatever you wish, but the `camera_info`-name cannot be changed, and they must have the same namespace. This allows the detectors to get access to the [`sensor_msgs/CameraInfo`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html) message, which might be useful for fiducial detection. Then the `camera_base_topic` in the config file will be `/common/camera/namespace/image_topic`. 

## Getting started
This must be installed for this package to compile / run as expected. 

### AprilTag 
Install according to [the AprilTag installation guide](https://github.com/AprilRobotics/apriltag#install). 

### ArUco
Install OpenCV and OpenCV contributions versions 3.1 or above. Tested using 3.2 and 4.5. 

### Python 3
To simplify some of the AprilTag interface, the header file `fiducial_detectors/apriltag_defined_families.h` is generated in the CMakeList using Python 3, which must be callable. The `sys`, `glob`, `pkgconfig` and `argparse` libraries must be available. 

## Running the detectors 
After installing the detectors as described above, clone this repo and put it in your catkin workspace. If you are working on multiple `ros` projects, I recommend using multiple catkin workspaces, and using symbolic links to link your `ros` packages to the relevant workspaces. 

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
# initialize workspace
catkin build # or catkin_make
ln -s $(~)/dev/ros_fiducials/fiducial_detectors $(~)/catkin_ws/src 
cd catkin_ws 
catkin build 
source devel/setup.bash 
roslaunch fiducial_detectors apriltag_detector.launch
```

## Using other detection parameters 
All default parameters used are found in the [config](config) folder. One may directly change the `*_default.yaml` parameter files, though this is not recommended. Instead, create a new `yaml` file containing only the parameters you need to change. The default parameters are good enough in many cases, and the `*_default.yaml` files include both several parameters which doesn't really need to be changed and a lot of verbose parameter comments. 

The new config file can obviously be placed wherever you want, but I'd recommend placing it wherever the lanch file you start the detector from. E.g. given this project structure: 

```
~
└───catkin_ws 
|   |
|   └───src
|      |
|      └───symbolic links to fiducial_detections
|      |  
|      └───symbolic links to my_package
|
└───dev
|   |
|   └───ros_fiducials
|       |
|       ...
|       └───fiducial_detections
|           |
|           ...
|           └───config
|               |   apriltag_detector_default.yaml
|               |   ...
|           └───launch
|               |   apriltag_detector.launch
|               |   ...
|   |
|   ...
|   └───my_package
|       |
|       ...
|       └───config
|           | 
|           my_apriltag_params.yaml
|           ...
|       └───launch
|           |
|           apriltag_my_way.launch
|           ...
```

Then `my_apriltag_params.yaml` might look like: 

```
camera_base_topic:     "/my_cam/image_raw"

tag_family:            "tag25h11"
```

And `apriltag_my_way.launch` might look like: 

```
<launch>
  ...
  <include file="$(find fiducial_detectors)/launch/apriltag_detector.launch">
    <arg name="param_path" value="$(find my_package)/config/my_apriltag_params.yaml"/>
  </include>
</launch>
```

