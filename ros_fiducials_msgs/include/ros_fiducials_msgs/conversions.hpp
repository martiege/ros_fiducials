#pragma once

// #include <Eigen/Dense> 

// #include <opencv2/core.hpp>

#include <map> 
#include <array> 
#include <vector> 

#include "ros_fiducials_msgs/Point2D.h"
#include "ros_fiducials_msgs/Detection.h"
#include "ros_fiducials_msgs/DetectionArray.h"



namespace ros_fiducials_msgs
{

std::vector<double>::iterator fromMsg(std::vector<double>::iterator it, std::vector<double>::iterator end, const Point2D& point)
{
  if (it == end)
    return end; 
  *it = point.x; 
  it = std::next(it); 
  
  if (it == end)
    return end; 
  *it = point.y; 
  return std::next(it); 
}

std::vector<std::array<double, 2>>::iterator fromMsg(std::vector<std::array<double, 2>>::iterator it, std::vector<std::array<double, 2>>::iterator end, const Point2D& point)
{
  if (it == end)
    return end; 
  (*it)[0] = point.x; 
  (*it)[1] = point.y; 
  return std::next(it); 
}

std::array<std::array<double, 2>, 4>::iterator fromMsg(std::array<std::array<double, 2>, 4>::iterator it, std::array<std::array<double, 2>, 4>::iterator end, const Point2D& point)
{
  if (it == end)
    return end; 
  (*it)[0] = point.x; 
  (*it)[1] = point.y; 
  return std::next(it); 
}

template <class T> 
T fromMsg(T it, T end, const Detection& detection)
{
  if (it == end)
    return end; 
  it = fromMsg(it, end, detection.bottom_left); 

  if (it == end)
    return end; 
  it = fromMsg(it, end, detection.bottom_right); 

  if (it == end)
    return end; 
  it = fromMsg(it, end, detection.upper_right); 

  if (it == end)
    return end; 
  it = fromMsg(it, end, detection.upper_left); 

  return it; 
}

std::vector<double>::iterator fromMsg(std::vector<double>::iterator it, std::vector<double>::iterator end, const Detection& detection)
{
  return fromMsg<std::vector<double>::iterator>(it, end, detection); 
}

std::vector<std::array<double, 2>>::iterator fromMsg(std::vector<std::array<double, 2>>::iterator it, std::vector<std::array<double, 2>>::iterator end, const Detection& detection)
{
  return fromMsg<std::vector<std::array<double, 2>>::iterator>(it, end, detection); 
}

std::array<std::array<double, 2>, 4>::iterator fromMsg(std::array<std::array<double, 2>, 4>::iterator it, std::array<std::array<double, 2>, 4>::iterator end, const Detection& detection)
{
  return fromMsg<std::array<std::array<double, 2>, 4>::iterator>(it, end, detection); 
}

template <class T> 
T fromMsg(T it, T end, const DetectionArray& detectionArray)
{
  for (const Detection& detection : detectionArray.detections)
  {
    if (it == end)
      return end; 
    it = fromMsg<T>(it, end, detection); 
  }

  return it; 
}

std::vector<double>::iterator fromMsg(std::vector<double>::iterator it, std::vector<double>::iterator end, const DetectionArray& detectionArray)
{
  return fromMsg<std::vector<double>::iterator>(it, end, detectionArray); 
}

std::vector<std::array<double, 2>>::iterator fromMsg(std::vector<std::array<double, 2>>::iterator it, std::vector<std::array<double, 2>>::iterator end, const DetectionArray& detectionArray)
{
  return fromMsg<std::vector<std::array<double, 2>>::iterator>(it, end, detectionArray); 
}

void fromMsg(std::array<std::array<double, 2>, 4>& result, const Detection& detection)
{
  fromMsg(result.begin(), result.end(), detection); 
}

void fromMsg(std::vector<double>& result, const DetectionArray& detectionArray)
{
  std::size_t n_origin = result.size(); 
  result.resize(n_origin + 2 * 4 * detectionArray.detections.size()); 

  fromMsg(std::next(result.begin(), n_origin), result.end(), detectionArray); 
}

void fromMsg(std::vector<std::array<double, 2>>& result, const DetectionArray& detectionArray)
{
  std::size_t n_origin = result.size(); 
  result.resize(n_origin + 4 * detectionArray.detections.size()); 

  fromMsg(std::next(result.begin(), n_origin), result.end(), detectionArray); 
}


} // namespace ros_fiducials_msgs

