include_directories(.)

add_cast_component(PlacePropertyProvider PropertyProvider.cpp OutputsCache.cpp ConfidenceEstimator.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PlaceData PlaceShared)


