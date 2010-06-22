# Author: Charles Gretton (charles.gretton@gmail.com)
#

include_directories(.)

include_directories(./dtpddl)

link_directories(./dtpddl)

# link_directories(${CMAKEDEMO_BINARY_DIR}/w01-intro)

add_definitions(-std=c++0x -mno-align-stringops -minline-all-stringops -march=native)

add_definitions(-pedantic -vg -Wall -Wextra)

add_cast_component(dtp-control dtp-control.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Planner)
link_cast_component(${CAST_COMPONENT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${CAST_COMPONENT_NAME} m)
target_link_libraries(${CAST_COMPONENT_NAME} pthread)
target_link_libraries(${CAST_COMPONENT_NAME} dtplanning)
