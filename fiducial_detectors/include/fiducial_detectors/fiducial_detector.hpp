#pragma once 

#include "image_transport/publisher.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>

#include <ros/ros.h> 
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>

#include <fiducial_msgs/DetectionArray.h>

#include <fiducial_detectors/visualise_detection.hpp>


class FiducialDetector
{
private: 
  ros::NodeHandle nh_; 

  image_transport::ImageTransport imageTransport_;
  image_transport::CameraSubscriber cameraSubscriber_;

  ros::Publisher detectionPublisher_; 
  
  bool visualiseDetection_; 
  image_transport::Publisher visualiseDetectionPublisher_; 

  virtual fiducial_msgs::DetectionArray detect_fiducials(
    const sensor_msgs::ImageConstPtr& image_msg, 
    const sensor_msgs::CameraInfoConstPtr& camera_info_msg
  ) = 0; 
public: 
  FiducialDetector(
    const ros::NodeHandle& nh, 
    const std::string& camera_base_topic, uint32_t camera_queue_size, 
    const std::string& detection_topic, uint32_t detection_queue_size, 
    bool visualiseDetections, const std::string& visualise_detection_topic, uint32_t visualise_detection_queue_size
  ) : nh_{nh}, imageTransport_{nh}, visualiseDetection_(visualiseDetections)
  {
    cameraSubscriber_ = imageTransport_.subscribeCamera(
      camera_base_topic, camera_queue_size, &FiducialDetector::detector_callback, this
    ); 

    detectionPublisher_ = nh_.advertise<fiducial_msgs::DetectionArray>(
      detection_topic, detection_queue_size
    );

    visualiseDetectionPublisher_ = imageTransport_.advertise(
      visualise_detection_topic, visualise_detection_queue_size
    ); 
  }

  void detector_callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
  {
    fiducial_msgs::DetectionArray detections = detect_fiducials(image_msg, camera_info_msg); 
    
    detectionPublisher_.publish(detections); 

    if (visualiseDetection_) 
    {
      cv_bridge::CvImageConstPtr img_ptr = cv_bridge::toCvShare(image_msg); 

      cv::Mat image; 
      if (sensor_msgs::image_encodings::isMono(image_msg->encoding))
        cv::cvtColor(img_ptr->image, image, cv::COLOR_GRAY2BGR); 
      else  
        image = img_ptr->image.clone(); 
      image.convertTo(image, CV_64FC3, 1.0 / (1 << sensor_msgs::image_encodings::bitDepth(image_msg->encoding))); 
      fiducial_detectors::visualiseDetectionArray(
        image, detections, 
        cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0, 0, 0), 
        cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0, 0, 0), 
        cv::Scalar(1, 0, 0), cv::Scalar(0, 1, 0), cv::Scalar(1, 0, 0), cv::Scalar(0, 0, 1)
      ); 

      cv_bridge::CvImage vis(image_msg->header, "64FC3", image); 
      visualiseDetectionPublisher_.publish(vis.toImageMsg()); 
    }
  }
}; 
