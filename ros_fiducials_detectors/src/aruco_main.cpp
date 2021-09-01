#include <ros/ros.h> 

#include <string> 


#include "ros_fiducials_detectors/aruco_detector.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "aruco_detector_node"); 
  
  ros::NodeHandle pnh("~"); 


  std::vector<std::string> v; 
  pnh.getParamNames(v); 

  for (auto a : v)
    ROS_INFO_STREAM("param: " << a << '\n'); 

  std::string camera_base_topic; 
  int32_t camera_queue_size{0}; 
  if (pnh.getParam("camera_base_topic", camera_base_topic))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: camera_base_topic: " << camera_base_topic); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: camera_base_topic not found"); 
    return EXIT_FAILURE; 
  }
  if (pnh.getParam("camera_queue_size", camera_queue_size)) 
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: camera_queue_size: " << camera_queue_size); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: camera_queue_size not found"); 
    return EXIT_FAILURE; 
  }

  std::string detection_topic; 
  int32_t detection_queue_size{0}; 
  if (pnh.getParam("detection_topic", detection_topic)) 
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: detection_topic: " << detection_topic); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: detection_topic not found"); 
    return EXIT_FAILURE; 
  }
  if (pnh.getParam("detection_queue_size", detection_queue_size))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: detection_queue_size: " << detection_queue_size); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: detection_queue_size not found"); 
    return EXIT_FAILURE; 
  }

  std::string aruco_dictionary; 
  if (pnh.getParam("aruco_dictionary", aruco_dictionary))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: aruco_dictionary: " << aruco_dictionary); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: aruco_dictionary not found"); 
    return EXIT_FAILURE; 
  }

  double adaptiveThreshConstant; 
  if (pnh.getParam("adaptiveThreshConstant", adaptiveThreshConstant))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: adaptiveThreshConstant: " << adaptiveThreshConstant); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: adaptiveThreshConstant not found"); 
    return EXIT_FAILURE; 
  }

  int adaptiveThreshWinSizeMax; 
  if (pnh.getParam("adaptiveThreshWinSizeMax", adaptiveThreshWinSizeMax))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: adaptiveThreshWinSizeMax: " << adaptiveThreshWinSizeMax); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: adaptiveThreshWinSizeMax not found"); 
    return EXIT_FAILURE; 
  }

  int	adaptiveThreshWinSizeMin; 
  if (pnh.getParam("adaptiveThreshWinSizeMin", adaptiveThreshWinSizeMin))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: adaptiveThreshWinSizeMin: " << adaptiveThreshWinSizeMin); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: adaptiveThreshWinSizeMin not found"); 
    return EXIT_FAILURE; 
  }

  int adaptiveThreshWinSizeStep; 
  if (pnh.getParam("adaptiveThreshWinSizeStep", adaptiveThreshWinSizeStep))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: adaptiveThreshWinSizeStep: " << adaptiveThreshWinSizeStep); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: adaptiveThreshWinSizeStep not found"); 
    return EXIT_FAILURE; 
  }

  int	cornerRefinementMaxIterations; 
  if (pnh.getParam("cornerRefinementMaxIterations", cornerRefinementMaxIterations))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: cornerRefinementMaxIterations: " << cornerRefinementMaxIterations); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: cornerRefinementMaxIterations not found"); 
    return EXIT_FAILURE; 
  }

  double cornerRefinementMinAccuracy; 
  if (pnh.getParam("cornerRefinementMinAccuracy", cornerRefinementMinAccuracy))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: cornerRefinementMinAccuracy: " << cornerRefinementMinAccuracy); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: cornerRefinementMinAccuracy not found"); 
    return EXIT_FAILURE; 
  }

  int cornerRefinementWinSize; 
  if (pnh.getParam("cornerRefinementWinSize", cornerRefinementWinSize))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: cornerRefinementWinSize: " << cornerRefinementWinSize); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: cornerRefinementWinSize not found"); 
    return EXIT_FAILURE; 
  }

  double errorCorrectionRate; 
  if (pnh.getParam("errorCorrectionRate", errorCorrectionRate))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: errorCorrectionRate: " << errorCorrectionRate); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: errorCorrectionRate not found"); 
    return EXIT_FAILURE; 
  }

  int	markerBorderBits; 
  if (pnh.getParam("markerBorderBits", markerBorderBits))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: markerBorderBits: " << markerBorderBits); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: markerBorderBits not found"); 
    return EXIT_FAILURE; 
  }

  double maxErroneousBitsInBorderRate; 
  if (pnh.getParam("maxErroneousBitsInBorderRate", maxErroneousBitsInBorderRate))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: maxErroneousBitsInBorderRate: " << maxErroneousBitsInBorderRate); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: maxErroneousBitsInBorderRate not found"); 
    return EXIT_FAILURE; 
  }

  double maxMarkerPerimeterRate; 
  if (pnh.getParam("maxMarkerPerimeterRate", maxMarkerPerimeterRate))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: maxMarkerPerimeterRate: " << maxMarkerPerimeterRate); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: maxMarkerPerimeterRate not found"); 
    return EXIT_FAILURE; 
  } 

  double minCornerDistanceRate; 
  if (pnh.getParam("minCornerDistanceRate", minCornerDistanceRate))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: minCornerDistanceRate: " << minCornerDistanceRate); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: minCornerDistanceRate not found"); 
    return EXIT_FAILURE; 
  } 

  int minDistanceToBorder; 
  if (pnh.getParam("minDistanceToBorder", minDistanceToBorder))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: minDistanceToBorder: " << minDistanceToBorder); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: minDistanceToBorder not found"); 
    return EXIT_FAILURE; 
  } 

  double minMarkerDistanceRate;
  if (pnh.getParam("minDistanceToBorder", minMarkerDistanceRate))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: minMarkerDistanceRate: " << minMarkerDistanceRate); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: minMarkerDistanceRate not found"); 
    return EXIT_FAILURE; 
  } 

  double minMarkerPerimeterRate; 
  if (pnh.getParam("minMarkerPerimeterRate", minMarkerPerimeterRate))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: minMarkerPerimeterRate: " << minMarkerPerimeterRate); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: minMarkerPerimeterRate not found"); 
    return EXIT_FAILURE; 
  } 

  double minOtsuStdDev; 
  if (pnh.getParam("minOtsuStdDev", minOtsuStdDev))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: minOtsuStdDev: " << minOtsuStdDev); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: minOtsuStdDev not found"); 
    return EXIT_FAILURE; 
  } 

  bool doCornerRefinement; 
  if (pnh.getParam("doCornerRefinement", doCornerRefinement))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: doCornerRefinement: " << doCornerRefinement); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: doCornerRefinement not found"); 
    return EXIT_FAILURE; 
  } 

  double perspectiveRemoveIgnoredMarginPerCell; 
  if (pnh.getParam("perspectiveRemoveIgnoredMarginPerCell", perspectiveRemoveIgnoredMarginPerCell))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: perspectiveRemoveIgnoredMarginPerCell: " << perspectiveRemoveIgnoredMarginPerCell); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: perspectiveRemoveIgnoredMarginPerCell not found"); 
    return EXIT_FAILURE; 
  } 

  int perspectiveRemovePixelPerCell; 
  if (pnh.getParam("perspectiveRemovePixelPerCell", perspectiveRemovePixelPerCell))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: perspectiveRemovePixelPerCell: " << perspectiveRemovePixelPerCell); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: perspectiveRemovePixelPerCell not found"); 
    return EXIT_FAILURE; 
  } 

  double polygonalApproxAccuracyRate; 
  if (pnh.getParam("polygonalApproxAccuracyRate", polygonalApproxAccuracyRate))
    ROS_INFO_STREAM("ros_fiducials_detectors/aruco_main: polygonalApproxAccuracyRate: " << polygonalApproxAccuracyRate); 
  else 
  {
    ROS_ERROR("ros_fiducials_detectors/aruco_main: polygonalApproxAccuracyRate not found"); 
    return EXIT_FAILURE; 
  } 

  ros_fiducials_detectors::ArucoDetector detector(
    pnh, 
    camera_base_topic, camera_queue_size, 
    detection_topic, detection_queue_size, 
    aruco_dictionary, 
    adaptiveThreshConstant, adaptiveThreshWinSizeMax, adaptiveThreshWinSizeMin, adaptiveThreshWinSizeStep,
    cornerRefinementMaxIterations, cornerRefinementMinAccuracy, cornerRefinementWinSize,
    errorCorrectionRate,
    markerBorderBits,
    maxErroneousBitsInBorderRate, maxMarkerPerimeterRate,
    minCornerDistanceRate, minDistanceToBorder, minMarkerDistanceRate, minMarkerPerimeterRate, minOtsuStdDev,
    doCornerRefinement,
    perspectiveRemoveIgnoredMarginPerCell, perspectiveRemovePixelPerCell,
    polygonalApproxAccuracyRate
  ); 

  ros::spin(); 

  return EXIT_SUCCESS; 
}
