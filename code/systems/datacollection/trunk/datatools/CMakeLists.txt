cmake_minimum_required (VERSION 2.6)
set(CMAKE_BUILD_TYPE Release)
project (DataTools)
get_filename_component(DATATOOLS_ROOT . ABSOLUTE)

# use cmake files packaged with CAST as well
set(CMAKE_MODULE_PATH ${DATATOOLS_ROOT}/../cmake ${CMAKE_MODULE_PATH})

set(OUTPUT ${DATATOOLS_ROOT}/output)

find_package(Peekabot REQUIRED)
include_directories(${PEEKABOT_INSTALLROOT}/include)
link_directories(${PEEKABOT_INSTALLROOT}/lib)
add_executable(drawTrajectoryinPeekabot drawTrajectoryinPeekabot.cpp)
target_link_libraries(drawTrajectoryinPeekabot peekabot boost_filesystem )
install(TARGETS drawTrajectoryinPeekabot DESTINATION ${OUTPUT})
