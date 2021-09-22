# `fiducial_detectors` 
ROS package wrapping some fiducial detectors in a common structure and publishing the detections in a common framework. 

## Getting started
This must be installed for this package to compile / run as expected. 

### AprilTag 
Install according to [the AprilTag installation guide](https://github.com/AprilRobotics/apriltag#install). 

### ArUco
Install OpenCV and OpenCV contributions version 3.* (Any subset of version 3 should do? At some point there are breaking changes to the ArUco parameters, but I am unsure when). 

### Python 3
To simplify some of the AprilTag interface, the header file `fiducial_detectors/apriltag_defined_families.h` is generated in the CMakeList using Python 3, which must be callable. The `sys`, `glob`, `pkgconfig` and `argparse` libraries must be available. 