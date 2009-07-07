include_directories(../autogen)

Find_package(Player)

add_cast_component(SonarServerPlayer SonarServerPlayer.cpp)
target_link_libraries(SonarServerPlayer ${PLAYER_LIBRARIES})
target_link_libraries(SonarServerPlayer Sonar)

