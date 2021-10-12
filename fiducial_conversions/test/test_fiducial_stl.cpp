#include <ros/ros.h>

#include <fiducial_msgs/Point2D.h>
#include <fiducial_msgs/Detection.h>
#include <fiducial_msgs/DetectionArray.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <random>

#include "fiducial_conversions/fiducial_stl.hpp"



TEST(FiducialConversions, pointConversion)
{
  ASSERT_EQ(1, 1); 
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv); 
  
  ros::init(argc, argv, "testing"); 
  ros::NodeHandle nh; 

  return RUN_ALL_TESTS(); 
}
