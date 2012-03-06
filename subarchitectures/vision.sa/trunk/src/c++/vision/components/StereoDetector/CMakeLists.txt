include_directories(.)

add_definitions(-DHAVE_CAST)
add_definitions(-DLINUX)

# include openNI headers for kinect sensor
include_directories(/usr/include/ni)

# do we have pcl and v4r library
if (HAVE_PCL_LIBRARY)
option(HAVE_V4R "TUW v4r library is available" NO)
if (HAVE_V4R)
include(${COGX_ROOT}/cmake/FindV4R.cmake)
include_directories(${V4R_DIR})
link_directories(${V4R_DIR}/lib)


include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision)

include_directories(matas)
include_directories(math)
include_directories(utils)
include_directories(Gestalts3D)
include_directories(vs3)
include_directories(stereo)
# include_directories(kinect)
# include_directories(reasoner)
include_directories(CEdge)
# include_directories(objRepresentation)
# include_directories(Learner)
# include_directories(GraphCut)
# include_directories(libSVM)

add_subdirectory(CEdge)
add_subdirectory(matas)
add_subdirectory(utils)
add_subdirectory(Gestalts3D)
add_subdirectory(vs3)
add_subdirectory(stereo)
# add_subdirectory(kinect)
# add_subdirectory(Learner)
# add_subdirectory(GraphCut)
# add_subdirectory(reasoner)
# add_subdirectory(objRepresentation)
# add_subdirectory(libSVM)

# Set the sources.
SET(HEADERS
	StereoDetector.h
)
SET(SOURCES 
	StereoDetector.cpp
  #Configuration.cpp
)
#add_cast_component(StereoDetector ${SOURCES} ${HEADERS})
##link_cast_component(${CAST_COMPONENT_NAME} SDGestalts3D SDUtils SDObjRepresentation SDStereo SDMatas SDvs3 SDCEdge SDLearner SDGraphCut)
#link_cast_component(${CAST_COMPONENT_NAME} SDUtils SDMatas SDvs3 SDCEdge)
#link_cast_component(${CAST_COMPONENT_NAME} freetype ftgl)
#link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
#link_cast_component(${CAST_COMPONENT_NAME} PointCloud PointCloudClient)
#link_cast_component(${CAST_COMPONENT_NAME} v4rPCLAddOns v4rTomGine)

# target_link_libraries(${CAST_COMPONENT_NAME} v4rplanepopout v4rpclfunctions v4rtomgine)
endif (HAVE_V4R)
endif (HAVE_PCL_LIBRARY)

