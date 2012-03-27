project(PTZ-Servers)

include_directories(..)
include_directories(.)
include_directories(${VISUALIZATION_INCLUDE_DIRS})
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
link_directories(${VISUALIZATION_LIBRARY_DIRS})

# Set the sources.
set(SOURCES PTZServer.cpp)
set(HEADERS PTZServer.hpp)
add_library(PTZUtils SHARED ${SOURCES} ${HEADERS})

# Add libraries to link against.
target_link_libraries(PTZUtils
   PTZ CASTCore
   ${VISUALIZATION_LIBRARIES}
   )
install(TARGETS PTZUtils LIBRARY DESTINATION lib)

add_custom_target (PTZUtils.Resources
   DEPENDS res/ptucontrol.inc)
add_custom_command (
   OUTPUT res/ptucontrol.inc
   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/res
   COMMENT "make PTZUtils.Resources"
   COMMAND make
)
add_dependencies(PTZUtils PTZUtils.Resources)

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
