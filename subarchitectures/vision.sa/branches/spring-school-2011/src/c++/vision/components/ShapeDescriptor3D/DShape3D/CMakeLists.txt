cmake_minimum_required(VERSION 2.6)

if(VISION_LIB_DIR)
   SET(LEVMAR_PATH "${VISION_LIB_DIR}/levmar/levmar-2.5")
   SET(HOMEST_PATH  "${VISION_LIB_DIR}/homest/homest-1.3")
else(VISION_LIB_DIR)
   SET(LEVMAR_PATH "/usr/local/src/levmar-2.5") # "path to Lourakis' levmar v2.5" 
   SET(HOMEST_PATH  "/usr/local/src/homest-1.3") # "path to Lourakis' homest v1.3"
endif(VISION_LIB_DIR)

INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(${HOMEST_PATH})
INCLUDE_DIRECTORIES(${LEVMAR_PATH})

LINK_DIRECTORIES(${HOMEST_PATH})
LINK_DIRECTORIES(${LEVMAR_PATH})

# DShape3D -----------------------------------------
PROJECT(DShape3D)
SET(SOURCES
    Vector2.cc
    Vector2I.cc
    Except.cc 
    Math.cc
    SDraw.cc
    Color.cc
    Homography.cc
    PoseCv.cc
    Hull2D.cc
    SPlane3D.cc
    SPolygon3D.cc
    STriangle3D.cc
    Keypoint.cc
    KeypointDescriptor.cc
    KDVertex.cc
    KPTree.cc
    Plane.cc
    SPlane.cc
    DShapeCore.cc
    RASDescriptor.cc)

LINK_LIBRARIES(homest lapack blas levmar rt pthread)
LINK_LIBRARIES(${OPENCV_LDFLAGS})

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

