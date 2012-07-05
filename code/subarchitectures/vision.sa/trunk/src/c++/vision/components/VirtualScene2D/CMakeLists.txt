project(Vision.VirtualScene2D)

include_directories(.)
include_directories(${COGX_ROOT}/tools/cogxutils/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs+cast/src/cpp/autogen)
include_directories(${COGX_ROOT}/subarchitectures/binder.sa/src/c++/binder/autogen)

find_package(LibPlot) # for visualization
set(HAS_LIBPLOT ${LIBPLOT_FOUND})
if(HAS_LIBPLOT)
   add_definitions(-DHAS_LIBPLOT)
   set(LIBPLOT_LIBS plotter)
else(HAS_LIBPLOT)
   message("WARNING: libplot-dev not found. You should install it to see SVG graphics in Scene2D.")
endif(HAS_LIBPLOT)

ENABLE_COGX_VISUALIZATION(TRUE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Set the sources.
SET(SOURCES
   Scene2D.cpp
   belief_utils.cpp
   )

add_cast_component(VirtualScene2D ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData Video
   BeliefsCAST BeliefsCogX
   ${OPENCV_LDFLAGS}
   ${VISUALIZATION_LIBRARIES}
   ${LIBPLOT_LIBS}
   )

