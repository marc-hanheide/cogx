cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
find_package( OpenCV REQUIRED )

SET(PCL_ROS_PATH /home/aitor/test/perception_pcl_unstable)
SET(ROS_PATH /opt/ros/diamondback)

include_directories(/home/aitor/v4r_svn/aldoma/overlap_likelihood/include 
                    /home/aitor/v4r_svn/aldoma/generic_trainer_recognizer/include
                    ${PCL_ROS_PATH}/pcl/include/pcl-1.2/
                    /usr/include ${PCL_ROS_PATH}/pcl/msg_gen/cpp/include 
                    /usr/include/cminpack-1/ /usr/include/eigen3
                    ${ROS_PATH}/stacks/ros_comm/messages/std_msgs/msg_gen/cpp/include/ ${ROS_PATH}/stacks/ros_comm/utilities/rostime/include
                    ${ROS_PATH}/stacks/ros_comm/utilities/cpp_common/include/ ${ROS_PATH}/stacks/ros_comm/tools/rosconsole/include/ 
                    ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp_serialization/include ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp_traits/include/ 
                    ${ROS_PATH}/stacks/ros_comm/utilities/xmlrpcpp/src ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp/include/
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

link_directories( /usr/local/lib/vtk-5.6
                  /home/aitor/v4r_svn/aldoma/generic_trainer_recognizer/build /home/aitor/v4r_svn/aldoma/overlap_likelihood/build
                  ${ROS_PATH}/stacks/common_msgs/sensor_msgs/lib ${ROS_PATH}/stacks/ros_comm/tools/rosbag/lib 
                  ${ROS_PATH}/stacks/ros_comm/tools/topic_tools/lib ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp/lib 
                  ${ROS_PATH}/stacks/ros_comm/clients/cpp/roscpp_serialization/lib ${ROS_PATH}/stacks/ros_comm/utilities/xmlrpcpp/lib 
                  ${ROS_PATH}/stacks/ros_comm/tools/rosconsole/lib ${ROS_PATH}/stacks/ros_comm/utilities/rostime/lib 
                  ${ROS_PATH}/stacks/ros_comm/utilities/cpp_common/lib ${ROS_PATH}/ros/core/roslib/lib ${ROS_PATH}/ros/tools/rospack/lib 
                   ${PCL_ROS_PATH}/flann/lib64  ${PCL_ROS_PATH}/flann/lib
                   ${PCL_ROS_PATH}/cminpack/lib 
                   ${PCL_ROS_PATH}/pcl/lib64  ${PCL_ROS_PATH}/pcl/lib )
  

include (${PCL_ROS_PATH}/pcl/share/pcl-1.2/PCLConfig.cmake)

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher boost_filesystem trainer_recognizer overlap_likelihood pcl_apps pcl_visualization pcl_segmentation pcl_surface pcl_filters pcl_features pcl_octree ${OpenCV_LIBS} ros)