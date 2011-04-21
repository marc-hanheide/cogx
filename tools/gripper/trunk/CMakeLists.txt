cmake_minimum_required(VERSION 2.4)

Find_package(Player REQUIRED)

add_executable(gripper gripper.cpp)
target_link_libraries(gripper ${PLAYER_LDFLAGS} )
INSTALL(TARGETS gripper RUNTIME DESTINATION bin)

