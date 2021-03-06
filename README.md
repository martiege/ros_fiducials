# `ros_fiducial`
Unofficial ROS package creating a common message structure for multiple fiducial marker detectors. Currently supporting the [AprilTag](https://april.eecs.umich.edu/software/apriltag) and [ArUco](https://www.uco.es/investiga/grupos/ava/node/26) detectors. 

# Getting started
See [fiducial_conversions](fiducial_conversions/README.md), [fiducial_detectors](fiducial_detectors/README.md), and [fiducial_msgs](fiducial_msgs/README.md) for their respective _Getting started_-sections. 

# TODOs

- [ ] Add functions for converting the detection arrays to _easier to use_ structures. 
- [ ] Create better build structure / CMakeList for the detectors. It is currently very dependent on the OpenCV version, Python version and Python libraries. 
- [ ] Related to both issues above: figure out best way to include conversions to data structures from other libraries such as Eigen, OpenCV, etc. without demanding these to be installed. Best probably one package for each library? 
- [ ] Another also kinda related, should the conversions be their own package or part of the `fiducial_msgs` package? Probably not, so clean the `fiducial_msgs` for the packages included that were intended for the conversion. 
- [ ] Add whitelist / blacklist functionality to the detectors. 
- [ ] Add other detectors. 
- [ ] Figure out better way of exporting other results from the detections than the current method, and include this for the ArUco detector. 
- [ ] Remove hardcoded `"bgr8"` encoding from the ArUco detector. 
- [ ] Further generalize `fiducial_detector.hpp`? 
- [ ] Complete parameter descriptions
- [x] Is the namespace too convoluted? Is it dishonest or confusing to use the `ros_`-prefix when this is not official? _Changed now_. 
