#pragma once 

#include <string>

#include <ros/ros.h> 
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>

#include "fiducial_msgs/DetectionArray.h"


class FiducialDetector
{
private: 
  ros::NodeHandle nh_; 

  image_transport::ImageTransport imageTransport_;
  image_transport::CameraSubscriber cameraSubscriber_;

  ros::Publisher detectionPublisher_; 

  virtual fiducial_msgs::DetectionArray detect_fiducials(
    const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& camera_info_msg
  ) = 0; 
public: 
  FiducialDetector(
    const ros::NodeHandle& nh, 
    const std::string& camera_base_topic, uint32_t camera_queue_size, 
    const std::string& detection_topic, uint32_t detection_queue_size
  ) : nh_{nh}, imageTransport_{nh}
  {
    cameraSubscriber_ = imageTransport_.subscribeCamera(
      camera_base_topic, camera_queue_size, &FiducialDetector::detector_callback, this
    ); 

    detectionPublisher_ = nh_.advertise<fiducial_msgs::DetectionArray>(
      detection_topic, detection_queue_size
    );
  }

  void detector_callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
  {
    fiducial_msgs::DetectionArray detections = detect_fiducials(image_msg, camera_info_msg); 

    detectionPublisher_.publish(detections); 
  }
}; 
