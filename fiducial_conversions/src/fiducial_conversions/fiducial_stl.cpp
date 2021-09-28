#pragma once 

#include "fiducial_conversions/fiducial_stl.hpp"


namespace fiducial_conversions
{

std::vector<double>::iterator fromMsg(
  std::vector<double>::iterator it, 
  std::vector<double>::iterator end, 
  const fiducial_msgs::Point2D& point
)
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

std::vector<std::array<double, 2>>::iterator fromMsg(
  std::vector<std::array<double, 2>>::iterator it, 
  std::vector<std::array<double, 2>>::iterator end, 
  const fiducial_msgs::Point2D& point
)
{
  if (it == end)
    return end; 
  (*it)[0] = point.x; 
  (*it)[1] = point.y; 
  return std::next(it); 
}

std::array<std::array<double, 2>, 4>::iterator fromMsg(
  std::array<std::array<double, 2>, 4>::iterator it, 
  std::array<std::array<double, 2>, 4>::iterator end, 
  const fiducial_msgs::Point2D& point
)
{
  if (it == end)
    return end; 
  (*it)[0] = point.x; 
  (*it)[1] = point.y; 
  return std::next(it); 
}

template <class T> 
T fromMsg(T it, T end, const fiducial_msgs::Detection& detection)
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

std::vector<double>::iterator fromMsg(
  std::vector<double>::iterator it, 
  std::vector<double>::iterator end, 
  const fiducial_msgs::Detection& detection
)
{
  return fromMsg<std::vector<double>::iterator>(it, end, detection); 
}

std::vector<std::array<double, 2>>::iterator fromMsg(
  std::vector<std::array<double, 2>>::iterator it, 
  std::vector<std::array<double, 2>>::iterator end, 
  const fiducial_msgs::Detection& detection
)
{
  return fromMsg<std::vector<std::array<double, 2>>::iterator>(it, end, detection); 
}

std::array<std::array<double, 2>, 4>::iterator fromMsg(
  std::array<std::array<double, 2>, 4>::iterator it, 
  std::array<std::array<double, 2>, 4>::iterator end, 
  const fiducial_msgs::Detection& detection
)
{
  return fromMsg<std::array<std::array<double, 2>, 4>::iterator>(it, end, detection); 
}

template <class T> 
T fromMsg(T it, T end, const fiducial_msgs::DetectionArray& detectionArray)
{
  for (const fiducial_msgs::Detection& detection : detectionArray.detections)
  {
    if (it == end)
      return end; 
    it = fromMsg<T>(it, end, detection); 
  }

  return it; 
}

std::vector<double>::iterator fromMsg(
  std::vector<double>::iterator it, 
  std::vector<double>::iterator end, 
  const fiducial_msgs::DetectionArray& detectionArray
)
{
  return fromMsg<std::vector<double>::iterator>(it, end, detectionArray); 
}

std::vector<std::array<double, 2>>::iterator fromMsg(
  std::vector<std::array<double, 2>>::iterator it, 
  std::vector<std::array<double, 2>>::iterator end, 
  const fiducial_msgs::DetectionArray& detectionArray
)
{
  return fromMsg<std::vector<std::array<double, 2>>::iterator>(it, end, detectionArray); 
}

void fromMsg(
  std::array<std::array<double, 2>, 4>& result, 
  const fiducial_msgs::Detection& detection
)
{
  fromMsg(result.begin(), result.end(), detection); 
}

void fromMsg(
  std::vector<double>& result, 
  const fiducial_msgs::DetectionArray& detectionArray
)
{
  std::size_t n_origin = result.size(); 
  result.resize(n_origin + 2 * 4 * detectionArray.detections.size()); 

  fromMsg(std::next(result.begin(), n_origin), result.end(), detectionArray); 
}

void fromMsg(
  std::vector<std::array<double, 2>>& result, 
  const fiducial_msgs::DetectionArray& detectionArray
)
{
  std::size_t n_origin = result.size(); 
  result.resize(n_origin + 4 * detectionArray.detections.size()); 

  fromMsg(std::next(result.begin(), n_origin), result.end(), detectionArray); 
}

void fromMsg(
  std::map<std::size_t, std::array<double, 2>>& result, 
  const fiducial_msgs::DetectionArray& detectionArray
)
{
  result.clear(); 

  for (const fiducial_msgs::Detection& detection : detectionArray.detections)
  {
    result[4 * detection.id + 0][0] = detection.bottom_left.x;
    result[4 * detection.id + 0][1] = detection.bottom_left.y; 
    result[4 * detection.id + 1][0] = detection.bottom_right.x; 
    result[4 * detection.id + 1][1] = detection.bottom_right.y; 
    result[4 * detection.id + 2][0] = detection.upper_right.x; 
    result[4 * detection.id + 2][1] = detection.upper_right.y; 
    result[4 * detection.id + 3][0] = detection.upper_left.x; 
    result[4 * detection.id + 3][1] = detection.upper_left.y; 
  }
}


} // fiducial_conversions
