#include <ros/ros.h> 

#include <string> 


#include "ros_fiducials_detectors/apriltag_detector.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "apriltag_detector_node"); 

  ros::NodeHandle pnh("~"); 

  std::string camera_base_topic; 
  int32_t camera_queue_size{0}; 
  if (pnh.getParam("camera_base_topic", camera_base_topic))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: camera_base_topic: " << camera_base_topic); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: camera_base_topic not found"); 
    return EXIT_FAILURE; 
  }
  if (pnh.getParam("camera_queue_size", camera_queue_size)) 
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: camera_queue_size: " << camera_queue_size); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: camera_queue_size not found"); 
    return EXIT_FAILURE; 
  }

  std::string detections_topic; 
  int32_t detections_queue_size{0}; 
  if (pnh.getParam("detections_topic", detections_topic)) 
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: detections_topic: " << detections_topic); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: detections_topic not found"); 
    return EXIT_FAILURE; 
  }
  if (pnh.getParam("detections_queue_size", detections_queue_size))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: detections_queue_size: " << detections_queue_size); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: detections_queue_size not found"); 
    return EXIT_FAILURE; 
  }

  std::string tag_family; 
  if (pnh.getParam("tag_family", tag_family))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: tag_family: " << tag_family); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: tag_family not found"); 
    return EXIT_FAILURE; 
  }

  double quad_decimate{0}; 
  if (pnh.getParam("quad_decimate", quad_decimate))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: quad_decimate: " << quad_decimate); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: quad_decimate not found"); 
    return EXIT_FAILURE; 
  }

  double quad_sigma{0}; 
  if (pnh.getParam("quad_sigma", quad_sigma))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: quad_sigma: " << quad_sigma); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: quad_sigma not found"); 
    return EXIT_FAILURE; 
  }

  int nthreads{0}; 
  if (pnh.getParam("nthreads", nthreads))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: nthreads: " << nthreads); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: nthreads not found"); 
    return EXIT_FAILURE; 
  }

  bool debug{false}; 
  if (pnh.getParam("debug", debug))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: debug: " << debug); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: debug not found"); 
    return EXIT_FAILURE; 
  }

  bool refine_edges{false};
  if (pnh.getParam("refine_edges", refine_edges))
    ROS_INFO_STREAM("ros_fiducials_detectors/apriltag_main: refine_edges: " << refine_edges); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/apriltag_main: refine_edges not found"); 
    return EXIT_FAILURE; 
  }
  

  ros_fiducials_detectors::ApriltagDetector detector(
    pnh, 
    camera_base_topic, camera_queue_size, 
    detections_topic, detections_queue_size, 
    tag_family, 
    quad_decimate, quad_sigma, 
    nthreads, debug, refine_edges
  ); 

  ros::spin(); 

  return 0;
}

/*

public: 
  ArucoDetector(
    const ros::NodeHandle& nh, 
    const std::string& camera_base_topic, uint32_t camera_queue_size, 
    const std::string& detections_topic,   uint32_t detections_queue_size, 
    const std::string& aruco_dictionary, 
    double adaptiveThreshConstant, int adaptiveThreshWinSizeMax, int	adaptiveThreshWinSizeMin, int	adaptiveThreshWinSizeStep,
    int	cornerRefinementMaxIterations, int cornerRefinementMethod, double cornerRefinementMinAccuracy, int cornerRefinementWinSize,
    double errorCorrectionRate,
    int	markerBorderBits,
    double maxErroneousBitsInBorderRate, double maxMarkerPerimeterRate,
    double minCornerDistanceRate, int minDistanceToBorder, double minMarkerDistanceRate, double minMarkerPerimeterRate, double minOtsuStdDev,
    double perspectiveRemoveIgnoredMarginPerCell, int perspectiveRemovePixelPerCell,
    double polygonalApproxAccuracyRate
  ) : FiducialDetector(
      nh, 
      camera_base_topic, camera_queue_size, 
      detections_topic, detections_queue_size
    )
  {
{    if (aruco_dictionary == "DICT_4X4_50")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    else if (aruco_dictionary == "DICT_4X4_100")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    else if (aruco_dictionary == "DICT_4X4_250")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    else if (aruco_dictionary == "DICT_4X4_1000")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
    else if (aruco_dictionary == "DICT_5X5_50")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    else if (aruco_dictionary == "DICT_5X5_100")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
    else if (aruco_dictionary == "DICT_5X5_250")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    else if (aruco_dictionary == "DICT_5X5_1000")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    else if (aruco_dictionary == "DICT_6X6_50")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    else if (aruco_dictionary == "DICT_6X6_100")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
    else if (aruco_dictionary == "DICT_6X6_250")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    else if (aruco_dictionary == "DICT_6X6_1000")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    else if (aruco_dictionary == "DICT_7X7_50")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
    else if (aruco_dictionary == "DICT_7X7_100")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
    else if (aruco_dictionary == "DICT_7X7_250")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
    else if (aruco_dictionary == "DICT_7X7_1000")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
    else if (aruco_dictionary == "DICT_ARUCO_ORIGINAL")
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    else 
    {
      ROS_ERROR_STREAM("ros_fiducials_detectors/aruco_detector: UNDEFINED ARUCO DICTIONARY: " << aruco_dictionary << ", CHOOSING: DICT_4X4_50"); 
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    }}

    parameters_ = cv::aruco::DetectorParameters::create(); 
    parameters_->adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax;
    parameters_->adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin;
    parameters_->adaptiveThreshWinSizeStep = adaptiveThreshWinSizeStep;
    parameters_->cornerRefinementMaxIterations = cornerRefinementMaxIterations;
    parameters_->cornerRefinementMinAccuracy = cornerRefinementMinAccuracy;
    parameters_->cornerRefinementWinSize = cornerRefinementWinSize;
    parameters_->errorCorrectionRate = errorCorrectionRate;
    parameters_->markerBorderBits = markerBorderBits;
    parameters_->maxErroneousBitsInBorderRate = maxErroneousBitsInBorderRate;
    parameters_->maxMarkerPerimeterRate = maxMarkerPerimeterRate;
    parameters_->minCornerDistanceRate = minCornerDistanceRate;
    parameters_->minDistanceToBorder = minDistanceToBorder;
    parameters_->minMarkerDistanceRate = minMarkerDistanceRate;
    parameters_->minMarkerPerimeterRate = minMarkerPerimeterRate;
    parameters_->minOtsuStdDev = minOtsuStdDev;
    parameters_->perspectiveRemoveIgnoredMarginPerCell = perspectiveRemoveIgnoredMarginPerCell;
    parameters_->perspectiveRemovePixelPerCell = perspectiveRemovePixelPerCell;
    parameters_->polygonalApproxAccuracyRate = polygonalApproxAccuracyRate;
  }
}; */