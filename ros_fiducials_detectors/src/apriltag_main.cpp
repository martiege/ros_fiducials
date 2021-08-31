#include <ros/ros.h> 

#include <string> 
#include <vector> 

#include "ros_fiducials_detectors/apriltag_detector.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apriltag_detector_node"); 

  ros::NodeHandle pnh("~"); 

  std::vector<std::string> v; 
  pnh.getParamNames(v); 

  for (auto a : v)
    ROS_INFO_STREAM("param: " << a << '\n'); 

  std::string camera_base_topic; 
  int32_t camera_queue_size{0}; 
  if (pnh.getParam("camera_base_topic", camera_base_topic))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: camera_base_topic: " << camera_base_topic); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: camera_base_topic not found"); 
    return -1; 
  }
  if (pnh.getParam("camera_queue_size", camera_queue_size)) 
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: camera_queue_size: " << camera_queue_size); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: camera_queue_size not found"); 
    return -1; 
  }

  std::string detection_topic; 
  int32_t detection_queue_size{0}; 
  if (pnh.getParam("detection_topic", detection_topic)) 
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: detection_topic: " << detection_topic); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: detection_topic not found"); 
    return -1; 
  }
  if (pnh.getParam("detection_queue_size", detection_queue_size))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: detection_queue_size: " << detection_queue_size); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: detection_queue_size not found"); 
    return -1; 
  }

  std::string family; 
  if (pnh.getParam("family", family))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: family: " << family); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: family not found"); 
    return -1; 
  }

  double quad_decimate{0}; 
  if (pnh.getParam("detector_quad_decimate", quad_decimate))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: detector_quad_decimate: " << quad_decimate); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: detector_quad_decimate not found"); 
    return -1; 
  }

  double quad_sigma{0}; 
  if (pnh.getParam("detector_quad_sigma", quad_sigma))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: detector_quad_sigma: " << quad_sigma); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: detector_quad_sigma not found"); 
    return -1; 
  }

  int nthreads{0}; 
  if (pnh.getParam("detector_nthreads", nthreads))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: detector_nthreads: " << nthreads); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: detector_nthreads not found"); 
    return -1; 
  }

  bool debug{false}; 
  if (pnh.getParam("detector_debug", debug))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: detector_debug: " << debug); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: detector_debug not found"); 
    return -1; 
  }

  bool refine_edges{false};
  if (pnh.getParam("detector_refine_edges", refine_edges))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: refine_edges: " << refine_edges); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: detector_refine_edges not found"); 
    return -1; 
  }


  ros_fiducials_detectors::ApriltagDetector detector(
    pnh, 
    camera_base_topic, camera_queue_size, 
    detection_topic, detection_queue_size, 
    family, 
    quad_decimate, quad_sigma, 
    nthreads, debug, refine_edges
  ); 

  return 0;
}
