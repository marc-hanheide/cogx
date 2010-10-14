
if(VISION_LIB_DIR)
   SET(LEVMAR_PATH "${VISION_LIB_DIR}/levmar/levmar-2.5")
   SET(HOMEST_PATH  "${VISION_LIB_DIR}/homest/homest-1.3")
else(VISION_LIB_DIR)
   SET(LEVMAR_PATH "/usr/local/src/levmar-2.5") # "path to Lourakis' levmar v2.5" 
   SET(HOMEST_PATH  "/usr/local/src/homest-1.3") # "path to Lourakis' homest v1.3"
endif(VISION_LIB_DIR)


include_directories(.)
include_directories(./DShape3D)
include_directories(${VISUALIZATION_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${HOMEST_PATH})
INCLUDE_DIRECTORIES(${LEVMAR_PATH})
LINK_DIRECTORIES(${HOMEST_PATH})
LINK_DIRECTORIES(${LEVMAR_PATH})

add_cast_component(ShapeDescriptor3D ShapeDescriptor3D.cpp PlaneRANSAC.cpp PlanarPatch.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Stereo StereoClient)
link_cast_component(${CAST_COMPONENT_NAME} VisionData VideoUtils DShape3D)
link_cast_component(${CAST_COMPONENT_NAME} ${VISUALIZATION_LIBRARIES})

add_subdirectory(DShape3D)

