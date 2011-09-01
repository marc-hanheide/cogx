cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(OVERLAP)
SET (CMAKE_BUILD_TYPE Debug)
list(APPEND CMAKE_MODULE_PATH ${OVERLAP_SOURCE_DIR})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package( OpenCV REQUIRED )

set(USE_ROS FALSE)

if (USE_ROS)

  SET(PCL_ROS_PATH /home/aitor/test/perception_pcl_unstable)
  SET(ROS_PATH /opt/ros/diamondback)
  
  include_directories(${PCL_ROS_PATH}/pcl/include/pcl-1.2/ include
  /usr/include ${PCL_ROS_PATH}/pcl/msg_gen/cpp/include 
  ${ROS_PATH}/stacks/common_msgs/sensor_msgs/include 
  ${ROS_PATH}/stacks/common_msgs/sensor_msgs/msg_gen/cpp/include 
  ${ROS_PATH}/stacks/common_msgs/sensor_msgs/srv_gen/cpp/include 
  ${ROS_PATH}/stacks/common_msgs/geometry_msgs/msg_gen/cpp/include 
  ${ROS_PATH}/stacks/ros_comm/tools/rosbag/include 
  ${ROS_PATH}/stacks/ros_comm/tools/topic_tools/include 
  ${ROS_PATH}/stacks/ros_comm/tools/topic_tools/srv_gen/cpp/include 
  ${ROS_PATH}/stacks/ros_comm/tools/rostest/include 
  ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp/include 
  ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp/msg_gen/cpp/include 
  ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp/srv_gen/cpp/include 
  ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp_serialization/include 
  ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp_traits/include 
  ${ROS_PATH}/stacks/ros_comm/utilities/xmlrpcpp/src 
  ${ROS_PATH}/stacks/ros_comm/tools/rosconsole/include 
  ${ROS_PATH}/stacks/ros_comm/utilities/rostime/include 
  ${ROS_PATH}/stacks/ros_comm/utilities/cpp_common/include 
  ${ROS_PATH}/stacks/ros_comm/messages/rosgraph_msgs/msg_gen/cpp/include 
  ${ROS_PATH}/stacks/ros_comm/messages/std_msgs/include 
  ${ROS_PATH}/stacks/ros_comm/messages/std_msgs/msg_gen/cpp/include 
  ${ROS_PATH}/ros/core/roslib/msg_gen/cpp/include 
  ${ROS_PATH}/ros/core/roslib/include 
  ${ROS_PATH}/ros/tools/rospack 
  ${ROS_PATH}/ros/tools/rospack/include 
  ${ROS_PATH}/stacks/geometry/eigen/include 
  ${PCL_ROS_PATH}/cminpack/include 
  ${PCL_ROS_PATH}/flann/include /usr/include/openni/
  /home/aitor/v4r_svn/aldoma/generic_trainer_recognizer/include)
    
  link_directories(/usr/local/lib/vtk-5.6
  ${ROS_PATH}/stacks/common_msgs/sensor_msgs/lib ${ROS_PATH}/stacks/ros_comm/tools/rosbag/lib 
  ${ROS_PATH}/stacks/ros_comm/tools/topic_tools/lib ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp/lib 
  ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp_serialization/lib ${ROS_PATH}/stacks/ros_comm/utilities/xmlrpcpp/lib 
  ${ROS_PATH}/stacks/ros_comm/tools/rosconsole/lib ${ROS_PATH}/stacks/ros_comm/utilities/rostime/lib 
  ${ROS_PATH}/stacks/ros_comm/utilities/cpp_common/lib ${ROS_PATH}/ros/core/roslib/lib ${ROS_PATH}/ros/tools/rospack/lib 
  ${PCL_ROS_PATH}/flann/lib64 ${PCL_ROS_PATH}/flann/lib
  ${PCL_ROS_PATH}/cminpack/lib 
  ${PCL_ROS_PATH}/pcl/lib64 ${PCL_ROS_PATH}/pcl/lib
  /home/aitor/v4r_svn/aldoma/generic_trainer_recognizer/build)
  
  include (/home/aitor/test/perception_pcl_unstable/pcl/share/pcl-1.2/PCLConfig.cmake)
  
else (USE_ROS)

  find_package(PCL 1.2 REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS} include /home/aitor/v4r_svn/aldoma/generic_trainer_recognizer/include /usr/include/cminpack-1/ /usr/include/eigen3)
  link_directories(${PCL_LINK_DIRECTORIES} /home/aitor/v4r_svn/aldoma/generic_trainer_recognizer/build)
  add_definitions(${PCL_DEFINITIONS})
  
endif (USE_ROS)

OPTION(BUILD_SHARED_LIBS "turn OFF for .a libs" ON)

if (USE_ROS)
  add_library(overlap_likelihood src/overlap.cpp)
  target_link_libraries(overlap_likelihood boost_filesystem pcl_octree pcl_io pcl_visualization ${OpenCV_LIBS})

  add_executable(test_overlap src/test.cpp)
  target_link_libraries(test_overlap pcl_io trainer_recognizer overlap_likelihood pcl_apps)
else (USE_ROS)
  add_library(overlap_likelihood src/overlap.cpp)
  target_link_libraries(overlap_likelihood boost_filesystem pcl_visualization pcl_segmentation pcl_surface pcl_filters pcl_features pcl_octree ${OpenCV_LIBS})

  add_executable(test_overlap src/test.cpp)
  target_link_libraries(test_overlap trainer_recognizer overlap_likelihood pcl_apps)
endif (USE_ROS)