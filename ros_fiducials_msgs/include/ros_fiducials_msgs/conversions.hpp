#pragma once

#include <Eigen/Dense> 

#include <opencv2/core.hpp>

#include <map> 
#include <array> 
#include <vector> 

#include "ros_fiducials_msgs/Point2D.h"
#include "ros_fiducials_msgs/Detection.h"
#include "ros_fiducials_msgs/DetectionArray.h"



namespace ros_fiducials_msgs
{

std::vector<double>::iterator fromMsg(std::vector<double>::iterator it, const Point2D& point)
{
  *it = point.x; 
  it = std::next(it); 
  *it = point.y; 
  return std::next(it); 
}

std::vector<double>::iterator fromMsg(std::vector<double>::iterator it, const Detection& detection)
{
  
}

std::vector<double> fromMsg(const DetectionArray& detectionArray)
{
  std::vector<double> result(4 * detectionArray.detections.size()); 

  for (const Detection& detection : detectionArray.detections)
  {
    result.push_back(detection.bottom_left.x )
    result.push_back(detection.bottom_left.x )

    result.push_back(detection.bottom_left.x )
    result.push_back(detection.bottom_left.x )

    result.push_back(detection.bottom_left.x )
    result.push_back(detection.bottom_left.x )

    result.push_back(detection.bottom_left.x )
    result.push_back(detection.bottom_left.x )
  }
}



} // namespace ros_fiducials_msgs

