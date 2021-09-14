cmake_minimum_required(VERSION 3.0.2)
project(ros_fiducials_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
)

add_message_files(
  FILES
  Point2D.msg 
  Detection.msg
  DetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

