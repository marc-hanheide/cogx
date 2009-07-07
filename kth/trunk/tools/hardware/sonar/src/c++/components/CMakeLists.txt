include_directories(../autogen)

Find_package(Player)

add_cast_component(SonarServerPlayer SonarServerPlayer.cpp)
target_link_libraries(SonarServerPlayer ${PLAYER_LIBRARIES})
target_link_libraries(SonarServerPlayer Sonar)


option(BUILD_HAL_SONAR_TESTS "Build hardware sonar tests" NO)
if(BUILD_HAL_SONAR_TESTS)
add_subdirectory (tests)
endif(BUILD_HAL_SONAR_TESTS)
