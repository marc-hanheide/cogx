include_directories(.)

include_directories(${VISUALIZATION_INCLUDE_DIRS})

find_package(LibPlot) # for visualization
set(HAS_LIBPLOT ${LIBPLOT_FOUND})
if(HAS_LIBPLOT)
   add_definitions(-DHAS_LIBPLOT)
   set(LIBPLOT_LIBS plotter)
else(HAS_LIBPLOT)
   message("WARNING: libplot-dev not found. You should install it to see SVG graphics in Scene2D.")
endif(HAS_LIBPLOT)

# Set the sources.
SET(SOURCES Scene2D.cpp)

add_cast_component(VirtualScene2D ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData
   ${OPENCV_LDFLAGS}
   ${VISUALIZATION_LIBRARIES}
   ${LIBPLOT_LIBS}
   )

