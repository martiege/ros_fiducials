#pragma once

#include <memory> 
#include <opencv2/core/hal/interface.h>
#include <opencv2/imgproc.hpp>
#include <vector> 

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <fiducial_msgs/Detection.h>
#include <fiducial_msgs/DetectionArray.h>

#include <opencv2/aruco.hpp>

#include "fiducial_detectors/fiducial_detector.hpp"


namespace fiducial_detectors
{

class ArucoDetector : public FiducialDetector
{
private: 
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;

  std::vector<int> ids_; 
  std::vector<std::vector<cv::Point2f>> corners_, rejected_; 

  fiducial_msgs::DetectionArray detect_fiducials(
    const sensor_msgs::ImageConstPtr& image_msg, 
    const sensor_msgs::CameraInfoConstPtr& camera_info_msg
  ) override 
  {
    fiducial_msgs::DetectionArray detectionArray; 
    detectionArray.header = image_msg->header; 
    detectionArray.detector = "aruco";

    cv_bridge::CvImagePtr img_ptr; 
    try
    {
      // TODO: generalise? 
      img_ptr = cv_bridge::toCvCopy(image_msg, "passth"); 
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_WARN_STREAM("fiducial_detectors/aruco_detector: cv_bridge::toCvCopy exception in fiducial_detectors/apriltag_detector.hpp: " << e.what()); 
      return detectionArray; 
    }

    cv::Mat gray; 
    if (sensor_msgs::image_encodings::isBayer(image_msg->encoding))
      cv::cvtColor(img_ptr->image, gray, cv::COLOR_BGR2GRAY); 
    else if (sensor_msgs::image_encodings::isColor(image_msg->encoding))
      cv::cvtColor(img_ptr->image, gray, cv::COLOR_RGB2GRAY); 
    else if (sensor_msgs::image_encodings::isMono(image_msg->encoding))
    {
      if (sensor_msgs::image_encodings::bitDepth(image_msg->encoding) != 8)
        img_ptr->image.convertTo(gray, CV_16UC1, static_cast<double>(1 << 8) / static_cast<double>(1 << sensor_msgs::image_encodings::bitDepth(image_msg->encoding))); 
      else
        gray = img_ptr->image.clone();
    }
    else
    {
      ROS_WARN_STREAM("Unknown sensor_msgs::image_encodings in fiducial_detectors/apriltag_detector.hpp: " << img_ptr->encoding); 
      return detectionArray; 
    }

    cv::aruco::detectMarkers(
      img_ptr->image, 
      dictionary_, 
      corners_, ids_, 
      parameters_, 
      rejected_
    ); 

    if (! corners_.empty())
    {
      for (std::size_t i{0}; i < corners_.size(); ++i)
      {
        fiducial_msgs::Detection ros_det; 
        ros_det.id = ids_[i]; 

        ros_det.upper_left.x = corners_[i][0].x; 
        ros_det.upper_left.y = corners_[i][0].y; 

        ros_det.upper_right.x = corners_[i][1].x; 
        ros_det.upper_right.y = corners_[i][1].y;  

        ros_det.bottom_right.x = corners_[i][2].x; 
        ros_det.bottom_right.y = corners_[i][2].y; 

        ros_det.bottom_left.x = corners_[i][3].x; 
        ros_det.bottom_left.y = corners_[i][3].y; 

        detectionArray.detections.push_back(ros_det); 
      }
    }

    return detectionArray; 
  }

public: 
  ArucoDetector(
    const ros::NodeHandle& nh, 
    const std::string& camera_base_topic, uint32_t camera_queue_size, 
    const std::string& detection_topic,   uint32_t detection_queue_size, 
    bool visualiseDetections, const std::string& visualise_detection_topic, uint32_t visualise_detection_queue_size, 
    const std::string& aruco_dictionary, 
    double adaptiveThreshConstant, int adaptiveThreshWinSizeMax, int	adaptiveThreshWinSizeMin, int	adaptiveThreshWinSizeStep,
    int	cornerRefinementMaxIterations, double cornerRefinementMinAccuracy, int cornerRefinementWinSize,
    double errorCorrectionRate,
    int	markerBorderBits,
    double maxErroneousBitsInBorderRate, double maxMarkerPerimeterRate,
    double minCornerDistanceRate, int minDistanceToBorder, double minMarkerDistanceRate, double minMarkerPerimeterRate, double minOtsuStdDev,
    bool doCornerRefinement,
    double perspectiveRemoveIgnoredMarginPerCell, int perspectiveRemovePixelPerCell,
    double polygonalApproxAccuracyRate
  ) : FiducialDetector(
      nh, 
      camera_base_topic, camera_queue_size, 
      detection_topic, detection_queue_size, 
      visualiseDetections, visualise_detection_topic, visualise_detection_queue_size
    )
  {
    if (aruco_dictionary == "DICT_4X4_50")
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
      ROS_ERROR_STREAM("fiducial_detectors/aruco_detector: UNDEFINED ARUCO DICTIONARY: " << aruco_dictionary << ", CHOOSING: DICT_4X4_50"); 
      dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    }
    

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
    parameters_->doCornerRefinement = doCornerRefinement; 
    parameters_->perspectiveRemoveIgnoredMarginPerCell = perspectiveRemoveIgnoredMarginPerCell;
    parameters_->perspectiveRemovePixelPerCell = perspectiveRemovePixelPerCell;
    parameters_->polygonalApproxAccuracyRate = polygonalApproxAccuracyRate;
  }
}; 


} // namespace fiducial_detectors
