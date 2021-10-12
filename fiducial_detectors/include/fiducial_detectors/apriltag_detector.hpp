#pragma once 

#include <memory> 

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <fiducial_msgs/Detection.h>
#include <fiducial_msgs/DetectionArray.h>

#include "fiducial_detectors/fiducial_detector.hpp"
#include "fiducial_detectors/apriltag_defined_families.h"


namespace fiducial_detectors
{

class ApriltagDetector : public FiducialDetector
{
private: 
  ApriltagFamily id_; 
  apriltag_family_t* family_; 
  apriltag_detector_t* detector_; 
  
  fiducial_msgs::DetectionArray detect_fiducials(
    const sensor_msgs::ImageConstPtr& image_msg, 
    const sensor_msgs::CameraInfoConstPtr& camera_info_msg
  ) override 
  {
    fiducial_msgs::DetectionArray detectionArray; 
    detectionArray.header = image_msg->header; 
    detectionArray.detector = "apriltag"; 

    cv_bridge::CvImageConstPtr img_ptr; 
    try
    {
      img_ptr = cv_bridge::toCvShare(image_msg); 
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_WARN_STREAM("cv_bridge::toCvShare exception in fiducial_detectors/apriltag_detector.hpp: " << e.what()); 
      return detectionArray; 
    }

    // TODO: Optimize this? Demand grayscale input? 
    cv::Mat gray; 
    if (sensor_msgs::image_encodings::isBayer(image_msg->encoding))
      cv::cvtColor(img_ptr->image, gray, cv::COLOR_BGR2GRAY); 
    else if (sensor_msgs::image_encodings::isColor(image_msg->encoding))
      cv::cvtColor(img_ptr->image, gray, cv::COLOR_RGB2GRAY); 
    else if (sensor_msgs::image_encodings::isMono(image_msg->encoding))
    {
      if (sensor_msgs::image_encodings::bitDepth(image_msg->encoding) != 8)
        img_ptr->image.convertTo(
          gray, CV_8UC1, 
          static_cast<double>(1 << 8) / static_cast<double>(1 << sensor_msgs::image_encodings::bitDepth(image_msg->encoding))
        ); 
      else
        gray = img_ptr->image.clone();
    }
    else
    {
      ROS_WARN_STREAM("Unknown sensor_msgs::image_encodings in fiducial_detectors/apriltag_detector.hpp: " << img_ptr->encoding); 
      return detectionArray; 
    }

    image_u8_t apriltag_image = { 
      .width  = gray.cols,
      .height = gray.rows,
      .stride = gray.cols,
      .buf    = gray.data
    };

    apriltag_detection_t* det;
    zarray_t* detections = apriltag_detector_detect(detector_, &apriltag_image);

    for (int i{0}; i < zarray_size(detections); ++i)
    { 
      zarray_get(detections, i, &det);

      fiducial_msgs::Detection ros_det; 
      
      ros_det.id = det->id; 
      
      ros_det.bottom_left.x = det->p[0][0]; 
      ros_det.bottom_left.y = det->p[0][1]; 

      ros_det.bottom_right.x = det->p[1][0]; 
      ros_det.bottom_right.y = det->p[1][1]; 

      ros_det.upper_right.x = det->p[2][0]; 
      ros_det.upper_right.y = det->p[2][1]; 

      ros_det.upper_left.x = det->p[3][0]; 
      ros_det.upper_left.y = det->p[3][1]; 

      ros_det.data.push_back(det->decision_margin); 
      ros_det.data.push_back(det->hamming); 
      ros_det.data.push_back(det->c[0]); 
      ros_det.data.push_back(det->c[1]); 
      for (int j{0}; j < 3; ++j)
      {
        for (int k{0}; k < 3; ++k)
          ros_det.data.push_back(matd_get(det->H, j, k)); 
      }

      detectionArray.detections.push_back(ros_det); 
    }

    apriltag_detections_destroy(detections); 

    return detectionArray; 
  }
public: 
  ApriltagDetector(
    const ros::NodeHandle& nh, 
    const std::string& camera_base_topic, uint32_t camera_queue_size, 
    const std::string& detection_topic,   uint32_t detection_queue_size, 
    bool visualiseDetections, const std::string& visualise_detection_topic, uint32_t visualise_detection_queue_size, 
    const std::string& family, 
    double quad_decimate, double quad_sigma, 
    int nthreads, bool debug, bool refine_edges
  ) : FiducialDetector(
      nh, 
      camera_base_topic, camera_queue_size, 
      detection_topic, detection_queue_size, 
      visualiseDetections, visualise_detection_topic, visualise_detection_queue_size
    )
  {
    id_       = stringToApriltagFamily(family); 
    family_   = createApriltagFamily(id_); 
    detector_ = apriltag_detector_create(); 

    apriltag_detector_add_family(detector_, family_); 

    detector_->quad_decimate = quad_decimate; 
    detector_->quad_sigma    = quad_sigma; 
    detector_->nthreads      = nthreads; 
    detector_->debug         = debug; 
    detector_->refine_edges  = refine_edges; 
  }

  ~ApriltagDetector()
  {
    if (detector_)
    {
      apriltag_detector_destroy(detector_); 

      detector_ = nullptr; 
    }

    if (family_) 
    {  
      destroyApriltagFamily(family_, id_); 
      
      family_ = nullptr; 
    }
  }
};

} // namespace fiducial_detectors 
