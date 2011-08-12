find_package(OpenCV REQUIRED)

include_directories(.)
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${OPENCV_CFLAGS})

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
  add_cast_component(${PROJECT_NAME} ${SOURCES})
  link_cast_component(${CAST_COMPONENT_NAME} OpenNI)

  include_directories(/usr/include/ni)

  add_cast_component(KinectVideoServer KinectVideoServer.cpp)
  link_cast_component(${CAST_COMPONENT_NAME} Video VideoServer VideoUtils Kinect)

  add_cast_component(KinectPCServer KinectPCServer.cpp)
  link_cast_component(${CAST_COMPONENT_NAME} VideoUtils Kinect KinectPersonDetect OpenNI PointCloud PointCloudServer ${OPENCV_LDFLAGS})

  add_cast_component(KinectStereoServer KinectStereoServer.cpp)
  link_cast_component(${CAST_COMPONENT_NAME} Video VideoUtils Kinect PointCloud PointCloudServer)

  add_cast_component(KinectStereoSeqServer KinectStereoSeqServer.cpp)
  link_cast_component(${CAST_COMPONENT_NAME} Video VideoUtils Kinect PointCloud PointCloudServer)
