project(vision.ObjectDetectorFerns)

include_directories(.)
include_directories(ferns_demo-1.0)

include_directories(
   ${COGX_ROOT}/subarchitectures/spatial.sa/src/c++/autogen
   )

ENABLE_COGX_VISUALIZATION(TRUE OPENCV)

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

add_cast_component(ObjectDetectorFERNS ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils ${VISUALIZATION_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} z)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})


