include_directories(.)
include_directories(ferns_demo-1.0)
include_directories(/usr/include/opencv)

# Nick needed the below for opencv. Use a find macro later
include_directories(/opt/local/include)
include_directories(/opt/local/include/opencv)
link_directories(/opt/local/lib)

include_directories(
   ${COGX_ROOT}/subarchitectures/spatial.sa/src/c++/autogen
   ${VISUALIZATION_INCLUDE_DIRS}
   )
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

# Set the sources.
SET(SOURCES ObjectDetectorFERNS.cpp
ferns_demo-1.0/affine_image_generator06.cc
ferns_demo-1.0/affine_transformation_range.cc
ferns_demo-1.0/buffer_management.cc
ferns_demo-1.0/cmphomo.cc
ferns_demo-1.0/fern_based_point_classifier.cc
ferns_demo-1.0/ferns.cc
ferns_demo-1.0/fine_gaussian_pyramid.cc
ferns_demo-1.0/homography06.cc
ferns_demo-1.0/homography_estimator.cc
ferns_demo-1.0/main.cc
ferns_demo-1.0/mcv.cc
ferns_demo-1.0/mcvGaussianSmoothing.cc
ferns_demo-1.0/planar_pattern_detector_builder.cc
ferns_demo-1.0/planar_pattern_detector.cc
ferns_demo-1.0/pyr_yape06.cc
ferns_demo-1.0/template_matching_based_tracker.cc)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})


add_cast_component(ObjectDetectorFERNS ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils ${VISUALIZATION_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS} z)


