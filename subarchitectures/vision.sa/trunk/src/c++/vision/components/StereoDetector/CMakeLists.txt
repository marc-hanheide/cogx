include_directories(.)

add_definitions(-DHAVE_CAST)

# include openNI headers for kinect sensor
include_directories(/usr/include/ni)

include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/TomGine)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/TomGine/TomGine)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/TomGine/TomGineWraper)

include_directories(matas)
include_directories(math)
include_directories(utils)
include_directories(vs3)
include_directories(stereo)
include_directories(pcl)
include_directories(kinect)
include_directories(reasoner)
include_directories(CEdge)
include_directories(objRepresentation)

add_subdirectory(CEdge)
add_subdirectory(matas)
add_subdirectory(utils)
add_subdirectory(vs3)
add_subdirectory(stereo)
add_subdirectory(kinect)

option(HAVE_PCL_LIBRARY "have pcl library installed." YES)
if (HAVE_PCL_LIBRARY)
add_subdirectory(pcl)
endif (HAVE_PCL_LIBRARY)

# add_subdirectory(reasoner)
add_subdirectory(objRepresentation)


# Set the sources.
SET(HEADERS
	StereoDetector.h
)
SET(SOURCES 
	StereoDetector.cpp
  #Configuration.cpp
)
add_cast_component(StereoDetector ${SOURCES} ${HEADERS})

link_cast_component(${CAST_COMPONENT_NAME} SDUtils SDObjRepresentation SDStereo SDMatas SDvs3 SDCEdge SDPCL)
link_cast_component(${CAST_COMPONENT_NAME} freetype ftgl GLWindow TomGine TomGineWraper)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)



