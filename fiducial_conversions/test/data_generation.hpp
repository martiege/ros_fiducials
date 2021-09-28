#pragma once 

#include <fiducial_msgs/Point2D.h>
#include <fiducial_msgs/Detection.h>
#include <fiducial_msgs/DetectionArray.h>

#include <cstdlib>
#include <random>


fiducial_msgs::DetectionArray generateTestData(
  const std::random_device& device, 
  double image_width,          double image_height, 
  double detection_width_min,  double detection_width_max, 
  double detection_height_min, double detection_height_max, 
  std::size_t n_detections)
{
  fiducial_msgs::DetectionArray detections; 
  detections.detector = "apriltag"; 
  detections.header.seq = 10; 
  detections.header.stamp = ros::Time(100, 100); 
  detections.header.frame_id = "frame"; 

  std::uniform_int_distribution<int> id_dist(0, 100); 
  std::uniform_real_distribution<double> image_width_dist(0,  image_width - detection_width_max); 
  std::uniform_real_distribution<double> image_height_dist(0, image_height - detection_height_max); 
  std::uniform_real_distribution<double> detection_width_dist(detection_width_min, detection_width_max); 
  std::uniform_real_distribution<double> detection_height_dist(detection_height_min, detection_height_max); 

  for (std::size_t i{0}; i < n_detections; ++i)
  {
    fiducial_msgs::Detection detection; 
    detection.id = id_dist(device); 

    // TODO: Generate more realistic data? 
    // e.g. place a marker in a 3D world, then project 
    // onto an image
    double u = image_width_dist(device); 
    double v = image_height_dist(device); 

    detection.bottom_left.x = u; 
    detection.bottom_left.y = v; 

    detection.bottom_right.x = u + detection_width_dist(device); 
    detection.bottom_right.y = v; 

    detection.upper_right.x = u + detection_width_dist(device); 
    detection.upper_right.y = v + detection_height_dist(device); 

    detection.upper_left.x = u; 
    detection.upper_left.y = v + detection_height_dist(device); 
  }

  return detections; 
}