include_directories(.)

add_cast_component(PlacePeekabotClient PeekabotClient.cpp)
link_cast_component(${CAST_COMPONENT_NAME} PlaceData PlaceShared)
link_cast_component(${CAST_COMPONENT_NAME} peekabot)

