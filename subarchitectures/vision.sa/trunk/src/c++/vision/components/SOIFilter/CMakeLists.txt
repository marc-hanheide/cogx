include_directories(.)

include_directories(${VISUALIZATION_INCLUDE_DIRS})
include_directories(${COGX_ROOT}/tools/hardware/ptz/src/c++)
include_directories(${COGX_ROOT}/tools/hardware/ptz/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/ptz/src/c++/components)
include_directories(${COGX_ROOT}/subarchitectures/spatial.sa/src/c++/autogen)

# TODO: add an option to turn manipulation on/off and define FEAT_TRACK_ARM.
# We need manipulation.sa/icefiles for SOIFilter to track the postion of the arm.
include_directories(${COGX_ROOT}/subarchitectures/manipulation.sa/src/c++/autogen)
add_definitions(-DFEAT_TRACK_ARM)

add_definitions(-DFEAT_VISUALIZATION_OPENCV)

find_package(LibPlot) # for visualization
set(HAS_LIBPLOT ${LIBPLOT_FOUND})
if(HAS_LIBPLOT)
   add_definitions(-DHAS_LIBPLOT)
   set(LIBPLOT_LIBS plotter)
endif(HAS_LIBPLOT)

# Set the sources.
SET(SOURCES
   SOIFilter.cpp
   TaskBase.cpp
   TaskReceiveSoi.cpp TaskMoveToViewCone.cpp TaskAnalyzePo.cpp 
   CmdGetSois.cpp CmdLearnerRecognize.cpp
   Snapper.cpp
   GraphCutSegmenter.cpp
   GCoptimization.cpp graph.cpp LinkedBlockList.cpp maxflow.cpp)

add_cast_component(SOIFilter ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData VideoUtils
   PointCloud PointCloudClient
   PTZ NavData
   ManipulationExeData
   ${VISUALIZATION_LIBRARIES}
   ${LIBPLOT_LIBS}
   )
