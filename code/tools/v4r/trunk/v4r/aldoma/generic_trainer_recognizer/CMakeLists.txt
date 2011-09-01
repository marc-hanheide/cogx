cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(TRAINER_RECOGNIZER)
SET (CMAKE_BUILD_TYPE RelWithDebInfo)
list(APPEND CMAKE_MODULE_PATH ${TRAINER_RECOGNIZER_SOURCE_DIR})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
find_package( OpenCV REQUIRED ) 

set(USE_ROS FALSE)

if (USE_ROS )

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
  ${PCL_ROS_PATH}/flann/include /usr/include/openni/)
    
  link_directories(/usr/local/lib/vtk-5.6
  ${ROS_PATH}/stacks/common_msgs/sensor_msgs/lib ${ROS_PATH}/stacks/ros_comm/tools/rosbag/lib 
  ${ROS_PATH}/stacks/ros_comm/tools/topic_tools/lib ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp/lib 
  ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp_serialization/lib ${ROS_PATH}/stacks/ros_comm/utilities/xmlrpcpp/lib 
  ${ROS_PATH}/stacks/ros_comm/tools/rosconsole/lib ${ROS_PATH}/stacks/ros_comm/utilities/rostime/lib 
  ${ROS_PATH}/stacks/ros_comm/utilities/cpp_common/lib ${ROS_PATH}/ros/core/roslib/lib ${ROS_PATH}/ros/tools/rospack/lib 
  ${PCL_ROS_PATH}/flann/lib64 ${PCL_ROS_PATH}/flann/lib
  ${PCL_ROS_PATH}/cminpack/lib 
  ${PCL_ROS_PATH}/pcl/lib64 ${PCL_ROS_PATH}/pcl/lib )
  
  include (${PCL_ROS_PATH}/pcl/share/pcl-1.2/PCLConfig.cmake)
  
  
else (USE_ROS)
  
  find_package(PCL 1.2 REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS} include /usr/include/cminpack-1/ /usr/include/eigen3)
  link_directories(${PCL_LINK_DIRECTORIES})
  add_definitions(${PCL_DEFINITIONS})
  
endif (USE_ROS)

OPTION(BUILD_SHARED_LIBS "turn OFF for .a libs" ON)

set(srcs src/recognizers/recognizer.cpp
    src/recognizers/roll_recognizer.cpp 
    src/model.cpp 
    src/trainers/trainer.cpp
    src/trainers/roll_trainer.cpp
    src/estimators/cvfh_estimator.cpp
    src/estimators/vfh_estimator.cpp
    src/estimators/roll_hist_estimator.cpp
    src/estimators/normal_estimator.cpp
    src/estimators/D2Edge_estimator.cpp
    src/estimators/D2CR_estimator.cpp
    src/estimators/ESF_estimator.cpp
    #src/estimators/SHOT_estimator.cpp
    #src/estimators/spin_image_estimator.cpp   
    #src/estimators/geodesic_estimator.cpp
    )

set(CMAKE_CXX_FLAGS -fopenmp)
set(CMAKE_C_FLAGS -fopenmp)

if (USE_ROS)
  add_library(trainer_recognizer ${srcs})
  target_link_libraries(trainer_recognizer boost_filesystem pcl_io pcl_visualization pcl_segmentation pcl_kdtree pcl_surface pcl_filters pcl_features ${OpenCV_LIBS} gomp sensor_msgs rosbag topic_tools ros boost_thread-mt boost_signals-mt roscpp_serialization XmlRpc rosconsole boost_thread-mt log4cxx rostime cpp_common roslib rospack rosstack cminpack flann flann_cpp hdf5_cpp hdf5)
  
  add_executable(train src/test.cpp)
  target_link_libraries(train trainer_recognizer pcl_apps)

else (USE_ROS)

  add_library(trainer_recognizer ${srcs})
  target_link_libraries(trainer_recognizer boost_filesystem pcl_visualization pcl_segmentation pcl_kdtree pcl_surface pcl_filters pcl_features ${OpenCV_LIBS} gomp hdf5_cpp hdf5)
  
  add_executable(train src/test.cpp)
  target_link_libraries(train trainer_recognizer pcl_apps gomp)

	add_executable(eval src/category_evaluation.cpp)
	target_link_libraries(eval trainer_recognizer pcl_apps gomp)


endif (USE_ROS)