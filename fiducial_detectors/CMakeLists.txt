cmake_minimum_required(VERSION 3.0.2)
project(fiducial_detectors)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# execute_process( COMMAND python3 scripts/apriltag_includes_update.py
# --output_path ${CMAKE_CURRENT_LIST_DIR}/include --namespace ${PROJECT_NAME}
# WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} RESULT_VARIABLE
# PYTHON_APRILTAG_FAMILY_GENERATE_RESULT )

# if (${PYTHON_APRILTAG_FAMILY_GENERATE_RESULT} EQUAL 0) message(STATUS
# "apriltag_defined_families.h created. ") else() message(FATAL_ERROR
# "apriltag_defined_families.h not created. ") endif()

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS cv_bridge
             geometry_msgs
             image_geometry
             image_transport
             roscpp
             rospy
             sensor_msgs
             std_msgs
             fiducial_msgs)

# System dependencies are found with CMake's conventions find_package(Boost
# REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(apriltag REQUIRED apriltag)
set(apriltag_INCLUDE_DIRS "${apriltag_INCLUDE_DIRS}/apriltag")
link_directories(${apriltag_LIBDIR})

find_package(Python 3 REQUIRED)
add_custom_target(
  apriltag_defined_families ALL
  COMMAND
    ${PYTHON_EXECUTABLE}
    ${CMAKE_CURRENT_LIST_DIR}/scripts/apriltag_includes_update.py --output_path
    ${CMAKE_CURRENT_LIST_DIR}/include --namespace ${PROJECT_NAME}
    --apriltag_include_dir ${apriltag_INCLUDE_DIRS}
  BYPRODUCTS
    ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/apriltag_defined_families.h
  DEPENDS ${CMAKE_CURRENT_LIST_DIR}/scripts/apriltag_includes_update.py
  COMMENT "Generating apriltag_defined_families.h")

# message("new command") add_custom_command( PRE_BUILD OUTPUT
# ${CMAKE_CURRENT_LIST_DIR}/include/${PROJECT_NAME}/apriltag_defined_families.h
# COMMAND python3 ${CMAKE_CURRENT_LIST_DIR}/scripts/apriltag_includes_update.py
# --output_path ${CMAKE_CURRENT_LIST_DIR}/include --namespace ${PROJECT_NAME}
# DEPENDS ${CMAKE_CURRENT_LIST_DIR}/scripts/apriltag_includes_update.py
# ${apriltag_LIBDIR} COMMENT "Creating custom header file depending on your
# installed apriltag families. " WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
# message("commanded")

# Uncomment this if the package has a setup.py. This macro ensures modules and
# global scripts declared therein get installed See
# http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ##############################################################################
# Declare ROS messages, services and actions ##
# ##############################################################################

# To declare and build messages, services or actions from within this package,
# follow these steps: * Let MSG_DEP_SET be the set of packages whose message
# types you use in your messages/services/actions (e.g. std_msgs,
# actionlib_msgs, ...). * In the file package.xml: * add a build_depend tag for
# "message_generation" * add a build_depend and a exec_depend tag for each
# package in MSG_DEP_SET * If MSG_DEP_SET isn't empty the following dependency
# has been pulled in but can be declared for certainty nonetheless: * add a
# exec_depend tag for "message_runtime" * In this file (CMakeLists.txt): * add
# "message_generation" and every package in MSG_DEP_SET to find_package(catkin
# REQUIRED COMPONENTS ...) * add "message_runtime" and every package in
# MSG_DEP_SET to catkin_package(CATKIN_DEPENDS ...) * uncomment the add_*_files
# sections below as needed and list every .msg/.srv/.action file to be processed
# * uncomment the generate_messages entry below * add every package in
# MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder add_message_files( FILES Message1.msg
# Message2.msg )

# Generate services in the 'srv' folder add_service_files( FILES Service1.srv
# Service2.srv )

# Generate actions in the 'action' folder add_action_files( FILES Action1.action
# Action2.action )

# Generate added messages and services with any dependencies listed here
# generate_messages( DEPENDENCIES geometry_msgs#   sensor_msgs#   std_msgs )

# ##############################################################################
# Declare ROS dynamic reconfigure parameters ##
# ##############################################################################

# To declare and build dynamic reconfigure parameters within this package,
# follow these steps: * In the file package.xml: * add a build_depend and a
# exec_depend tag for "dynamic_reconfigure" * In this file (CMakeLists.txt): *
# add "dynamic_reconfigure" to find_package(catkin REQUIRED COMPONENTS ...) *
# uncomment the "generate_dynamic_reconfigure_options" section below and list
# every .cfg file to be processed

# Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options( cfg/DynReconf1.cfg cfg/DynReconf2.cfg )

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################
# The catkin_package macro generates cmake config files for your package Declare
# things to be passed to dependent projects INCLUDE_DIRS: uncomment this if your
# package contains header files LIBRARIES: libraries you create in this project
# that dependent projects also need CATKIN_DEPENDS: catkin_packages dependent
# projects also need DEPENDS: system dependencies of this project that dependent
# projects also need
catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  fiducial_detectors
  CATKIN_DEPENDS
  cv_bridge
  geometry_msgs
  image_geometry
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
  # DEPENDS system_lib
)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${apriltag_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})

# Declare a C++ library add_library(${PROJECT_NAME}
# src/${PROJECT_NAME}/fiducial_detectors.cpp )

# Add cmake target dependencies of the library as an example, code may need to
# be generated before libraries either from message generation or dynamic
# reconfigure add_dependencies(${PROJECT_NAME}
# ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable With catkin_make all packages are built within a
# single CMake context The recommended prefix ensures that target names across
# packages don't collide add_executable(${PROJECT_NAME}_node
# src/fiducial_detectors_node.cpp)
add_executable(apriltag_detector_node src/apriltag_main.cpp)

add_dependencies(apriltag_detector_node ${catkin_EXPORTED_TARGETS}
                 apriltag_defined_families)

target_link_libraries(apriltag_detector_node ${catkin_LIBRARIES}
                      ${apriltag_LIBRARIES})

add_executable(aruco_detector_node src/aruco_main.cpp)

add_dependencies(aruco_detector_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(aruco_detector_node ${catkin_LIBRARIES}
                      ${apriltag_LIBRARIES} ${OpenCV_LIBRARIES})

# Rename C++ executable without prefix The above recommended prefix causes long
# target names, the following renames the target back to the shorter version for
# ease of user use e.g. "rosrun someones_pkg node" instead of "rosrun
# someones_pkg someones_pkg_node" set_target_properties(${PROJECT_NAME}_node
# PROPERTIES OUTPUT_NAME node PREFIX "")

# Add cmake target dependencies of the executable same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
# ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )

# ##############################################################################
# Install ##
# ##############################################################################

# all install targets should use catkin DESTINATION variables See
# http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation in contrast to
# setup.py, you can choose the destination catkin_install_python(PROGRAMS
# scripts/my_python_script DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Mark executables for installation See
# http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION
# ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Mark libraries for installation See
# http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION
# ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION
# ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION
# ${CATKIN_GLOBAL_BIN_DESTINATION} )

# Mark cpp header files for installation install(DIRECTORY
# include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE )

# Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES # myfile1 # myfile2 DESTINATION
# ${CATKIN_PACKAGE_SHARE_DESTINATION} )

# ##############################################################################
# Testing ##
# ##############################################################################

# Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fiducial_detectors.cpp)
# if(TARGET ${PROJECT_NAME}-test) target_link_libraries(${PROJECT_NAME}-test
# ${PROJECT_NAME}) endif()

# Add folders to be run by python nosetests catkin_add_nosetests(test)
