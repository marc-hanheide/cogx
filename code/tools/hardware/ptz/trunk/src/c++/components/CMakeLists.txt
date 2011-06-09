include_directories(..)
include_directories(.)

FIND_PACKAGE(Player REQUIRED)

# Set the sources.
set(SOURCES PTZServer.cpp)
set(HEADERS PTZServer.hpp)
add_library(PTZUtils SHARED ${SOURCES} ${HEADERS})

# Add libraries to link against.
target_link_libraries(PTZUtils PTZ CASTCore)
install(TARGETS PTZUtils LIBRARY DESTINATION lib)


option(BUILD_HAL_PTZ_PLAYER "Build hardware ptz: player version" YES)
if(BUILD_HAL_PTZ_PLAYER)
  find_package(Player REQUIRED)
  add_cast_component(PlayerPTZServer PlayerPTZServer.cpp PlayerPTZServer.hpp)
  link_cast_component(${CAST_COMPONENT_NAME} PTZUtils)
  link_cast_component(${CAST_COMPONENT_NAME} m ${PLAYER_LIBRARIES})

  add_cast_component(PlayerActServer PlayerActServer.cpp PlayerActServer.hpp)
  link_cast_component(${CAST_COMPONENT_NAME} PTZUtils)
  link_cast_component(${CAST_COMPONENT_NAME} m ${PLAYER_LIBRARIES})

endif(BUILD_HAL_PTZ_PLAYER)
