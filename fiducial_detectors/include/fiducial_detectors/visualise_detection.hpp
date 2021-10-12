// Copyright (c) 2021 Martin Eek Gerhardsen
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once 

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include <fiducial_msgs/DetectionArray.h>
#include <fiducial_msgs/Detection.h>
#include <fiducial_msgs/Point2D.h>

#include <sstream>


namespace fiducial_detectors 
{

void visualiseDetection(cv::Mat& image, const fiducial_msgs::Detection& detection, 
  int corner_fontFace, double corner_fontScale, cv::Scalar corner_color, 
  int id_fontFace, double id_fontScale, cv::Scalar id_color, 
  cv::Scalar bottom_color, cv::Scalar right_color, cv::Scalar upper_color, cv::Scalar left_color,
  int thickness=1, int lineType=cv::LINE_8, int shift=0
)
{
  cv::Point2i bottom_left  = {static_cast<int>(detection.bottom_left.x),  static_cast<int>(detection.bottom_left.y)};  
  cv::Point2i bottom_right = {static_cast<int>(detection.bottom_right.x), static_cast<int>(detection.bottom_right.y)}; 
  cv::Point2i upper_right  = {static_cast<int>(detection.upper_right.x),  static_cast<int>(detection.upper_right.y)};  
  cv::Point2i upper_left   = {static_cast<int>(detection.upper_left.x),   static_cast<int>(detection.upper_left.y)};   

  cv::putText(image, "0", bottom_left,  corner_fontFace, corner_fontScale, corner_color); 
  cv::putText(image, "1", bottom_right, corner_fontFace, corner_fontScale, corner_color); 
  cv::putText(image, "2", upper_right,  corner_fontFace, corner_fontScale, corner_color); 
  cv::putText(image, "3", upper_left,   corner_fontFace, corner_fontScale, corner_color); 

  std::stringstream ss; 
  ss << detection.id; 
  cv::putText(
    image, ss.str(), (bottom_left + bottom_right + upper_right + upper_left) / 4, 
    id_fontFace, id_fontScale, id_color
  );

  cv::line(image, bottom_left,  bottom_right, bottom_color); 
  cv::line(image, bottom_right, upper_right,  right_color); 
  cv::line(image, upper_right,  upper_left,   upper_color); 
  cv::line(image, upper_left,   bottom_left,  left_color); 
}

void visualiseDetectionArray(cv::Mat& image, const fiducial_msgs::DetectionArray& detections, 
  int corner_fontFace, double corner_fontScale, cv::Scalar corner_color, 
  int id_fontFace, double id_fontScale, cv::Scalar id_color, 
  cv::Scalar bottom_color, cv::Scalar right_color, cv::Scalar upper_color, cv::Scalar left_color,
  int thickness=1, int lineType=cv::LINE_8, int shift=0
)
{
  for (const fiducial_msgs::Detection& detection : detections.detections)
  {
    visualiseDetection(
      image, detection, corner_fontFace, corner_fontScale, corner_color, 
      id_fontFace, id_fontScale, id_color, 
      bottom_color, right_color, upper_color, left_color
    ); 
  }
}

} // fiducial_detectors
