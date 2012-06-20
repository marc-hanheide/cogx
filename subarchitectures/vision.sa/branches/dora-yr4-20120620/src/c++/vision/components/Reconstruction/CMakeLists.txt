include_directories(.)
include_directories(/opt/local/include)
include_directories(/usr/local/include/TooN)
link_directories (/opt/local/lib)

# Set the sources.
SET(SOURCES Reconstructor.cc GLWindow2.cc GLWindowMenu.cc ATANCamera.cc KeyFrame.cc MapPoint.cc Map.cc SmallBlurryImage.cc ShiTomasi.cc HomographyInit.cc MapMaker.cc Bundle.cc PatchFinder.cc Relocaliser.cc MiniPatch.cc ARDriver.cc Tracker.cc System.cc)

#Set the compile flag.
SET(CMAKE_CXX_FLAGS "-Wall -g -march=nocona -msse3 -fno-strict-aliasing -fno-tree-vectorize")


add_cast_component(Reconstructor ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} GVars3 cvd ${OPENCV_LDFLAGS})
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)

