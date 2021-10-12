#include <ros/ros.h> 

#include <string> 


#include "fiducial_detectors/apriltag_detector.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "apriltag_detector_node"); 

  ros::NodeHandle pnh("~"); 

  std::string camera_base_topic; 
  int32_t camera_queue_size{0}; 
  if (pnh.getParam("camera_base_topic", camera_base_topic))
    ROS_INFO_STREAM("fiducial_detectors/apriltag_main: camera_base_topic: " << camera_base_topic); 
  else 
  {
    ROS_ERROR("fiducial_detectors/apriltag_main: camera_base_topic not found"); 
    return EXIT_FAILURE; 
  }
  if (pnh.getParam("camera_queue_size", camera_queue_size)) 
    ROS_INFO_STREAM("fiducial_detectors/apriltag_main: camera_queue_size: " << camera_queue_size); 
  else 
  {
    ROS_ERROR("fiducial_detectors/apriltag_main: camera_queue_size not found"); 
    return EXIT_FAILURE; 
  }

  std::string detections_topic; 
  int32_t detections_queue_size{0}; 
  if (pnh.getParam("detections_topic", detections_topic)) 
    ROS_INFO_STREAM("fiducial_detectors/apriltag_main: detections_topic: " << detections_topic); 
  else 
  {
    ROS_ERROR("fiducial_detectors/apriltag_main: detections_topic not found"); 
    return EXIT_FAILURE; 
  }
  if (pnh.getParam("detections_queue_size", detections_queue_size))
    ROS_INFO_STREAM("fiducial_detectors/apriltag_main: detections_queue_size: " << detections_queue_size); 
  else 
  {
    ROS_ERROR("fiducial_detectors/apriltag_main: detections_queue_size not found"); 
    return EXIT_FAILURE; 
  }

  bool visualise_detections; 
  std::string visualise_detection_topic; 
  int32_t visualise_detection_queue_size{0}; 
  if (pnh.getParam("visualise_detections", visualise_detections)) 
    ROS_INFO_STREAM("fiducial_detectors/aruco_main: visualise_detections: " << visualise_detections); 
  else 
  {
    ROS_ERROR("fiducial_detectors/aruco_main: visualise_detections not found"); 
    return EXIT_FAILURE; 
  }
  if (pnh.getParam("visualise_detection_topic", visualise_detection_topic)) 
    ROS_INFO_STREAM("fiducial_detectors/apriltag_main: visualise_detection_topic: " << visualise_detection_topic); 
  else 
  {
    ROS_ERROR("fiducial_detectors/apriltag_main: visualise_detection_topic not found"); 
    return EXIT_FAILURE; 
  }
  if (pnh.getParam("visualise_detection_queue_size", visualise_detection_queue_size))
    ROS_INFO_STREAM("fiducial_detectors/apriltag_main: visualise_detection_queue_size: " << visualise_detection_queue_size); 
  else 
  {
    ROS_ERROR("fiducial_detectors/apriltag_main: visualise_detection_queue_size not found"); 
    return EXIT_FAILURE; 
  } 

  std::string tag_family; 
  if (pnh.getParam("tag_family", tag_family))
    ROS_INFO_STREAM("fiducial_detectors/apriltag_main: tag_family: " << tag_family); 
  else 
  {
    ROS_ERROR("fiducial_detectors/apriltag_main: tag_family not found"); 
    return EXIT_FAILURE; 
  }

  double quad_decimate{0}; 
  if (pnh.getParam("quad_decimate", quad_decimate))
    ROS_INFO_STREAM("fiducial_detectors/apriltag_main: quad_decimate: " << quad_decimate); 
  else 
  {
    ROS_ERROR("fiducial_detectors/apriltag_main: quad_decimate not found"); 
    return EXIT_FAILURE; 
  }

  double quad_sigma{0}; 
  if (pnh.getParam("quad_sigma", quad_sigma))
    ROS_INFO_STREAM("fiducial_detectors/apriltag_main: quad_sigma: " << quad_sigma); 
  else 
  {
    ROS_ERROR("fiducial_detectors/apriltag_main: quad_sigma not found"); 
    return EXIT_FAILURE; 
  }

  int nthreads{0}; 
  if (pnh.getParam("nthreads", nthreads))
    ROS_INFO_STREAM("fiducial_detectors/apriltag_main: nthreads: " << nthreads); 
  else 
  {
    ROS_ERROR("fiducial_detectors/apriltag_main: nthreads not found"); 
    return EXIT_FAILURE; 
  }

  bool debug{false}; 
  if (pnh.getParam("debug", debug))
    ROS_INFO_STREAM("fiducial_detectors/apriltag_main: debug: " << debug); 
  else 
  {
    ROS_ERROR("fiducial_detectors/apriltag_main: debug not found"); 
    return EXIT_FAILURE; 
  }

  bool refine_edges{false};
  if (pnh.getParam("refine_edges", refine_edges))
    ROS_INFO_STREAM("fiducial_detectors/apriltag_main: refine_edges: " << refine_edges); 
  else 
  {
    ROS_ERROR("fiducial_detectors/apriltag_main: refine_edges not found"); 
    return EXIT_FAILURE; 
  }
  

  fiducial_detectors::ApriltagDetector detector(
    pnh, 
    camera_base_topic, camera_queue_size, 
    detections_topic, detections_queue_size, 
    visualise_detections, visualise_detection_topic, visualise_detection_queue_size,
    tag_family, 
    quad_decimate, quad_sigma, 
    nthreads, debug, refine_edges
  ); 

  ros::spin(); 

  return 0;
}

