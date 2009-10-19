include_directories(.)

add_cast_component(PlaceLaserProcessor LaserProcessor.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PlaceData PlaceShared)
link_cast_component(${CAST_COMPONENT_NAME} SequentialAdaBoost svm-mod)
