include_directories(.)
include_directories(${COGX_ROOT}/tools/castutils/src/c++)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${OPENCV_CFLAGS})

find_package(OpenNI REQUIRED)
include_directories(${OPENNI_INCLUDE_DIRS})

option(BUILD_KINECT_USER_DETECTOR "Build the Kinect User Generator in KinectPCServer." NO)
if (BUILD_KINECT_USER_DETECTOR)
   add_definitions(-DKINECT_USER_DETECTOR)
endif (BUILD_KINECT_USER_DETECTOR)

include_directories(
   .
   ${VISUALIZATION_INCLUDE_DIRS}
   )
link_directories(${VISUALIZATION_LIBRARY_DIRS})
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

  SET(HEADERS
     Capture.h
     Device.h
     Kinect.h
  )
  SET(SOURCES 
     Capture.cpp
     Device.cpp
     Kinect.cpp
  )
  add_cast_component(Kinect ${SOURCES})
  link_cast_component(${CAST_COMPONENT_NAME} OpenNI)

  #this is necessary because those openni guys are incompetent ;-) They should fix the way they include their own includes...
  #include_directories(/usr/include/openni)

  add_cast_component(KinectVideoServer KinectVideoServer.cpp)
  link_cast_component(${CAST_COMPONENT_NAME} Video VideoServer VideoUtils Kinect)

  add_cast_component(KinectPCServer KinectPCServer.cpp)

  link_cast_component(${CAST_COMPONENT_NAME} VideoUtils
     Kinect  OpenNI PointCloud PointCloudServer
     ${VISUALIZATION_LIBRARIES}
     ${OPENCV_LDFLAGS})
  if (BUILD_KINECT_USER_DETECTOR)
     link_cast_component(${CAST_COMPONENT_NAME} KinectPersonDetect)
  endif (BUILD_KINECT_USER_DETECTOR)

  add_cast_component(KinectStereoServer KinectStereoServer.cpp)
  link_cast_component(${CAST_COMPONENT_NAME} Video VideoUtils Kinect PointCloud PointCloudServer)

  add_cast_component(KinectStereoSeqServer KinectStereoSeqServer.cpp)
  link_cast_component(${CAST_COMPONENT_NAME} Video VideoUtils Kinect PointCloud PointCloudServer)

  if (HAVE_PCL_LIBRARY)
    find_package(PCL REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    add_cast_component(KinectSeqServer KinectSeqServer.cpp)
    link_cast_component(${CAST_COMPONENT_NAME} ${PCL_LIBRARIES} VideoUtils PointCloud PointCloudServer ${OPENCV_LDFLAGS} KinectStereoSeqServer)
  endif (HAVE_PCL_LIBRARY)
