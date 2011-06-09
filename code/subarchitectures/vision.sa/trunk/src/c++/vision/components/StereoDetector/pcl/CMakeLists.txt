include_directories(.)


find_package(PCL 1.0 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})

# Set the sources.
SET(HEADERS
  PCLCommonHeaders.h
  PCLFunctions.h
  PCLUtils.h
)
SET(SOURCES 
  PCLUtils.cpp
  PCLFunctions.cpp
)
add_cast_component(SDPCL ${SOURCES} ${HEADERS})
link_cast_component(${CAST_COMPONENT_NAME} ${PCL_LIBRARIES})

#target_link_libraries(pcd_write_test ${PCL_IO_LIBRARIES})


