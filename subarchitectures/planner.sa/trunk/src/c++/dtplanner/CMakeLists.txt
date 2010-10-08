# Author: Charles Gretton (charles.gretton@gmail.com)
#
# link_directories(${CMAKEDEMO_BINARY_DIR}/w01-intro)

option(DT_PLANNER_LAO_STAR "Use LAO* heuristic in dt planner" NO)
if(DT_PLANNER_LAO_STAR)
  add_definitions(-DLAO_STAR)
endif(DT_PLANNER_LAO_STAR)

add_subdirectory(dtpddl)

include_directories(.)
include_directories(./dtpddl)

#link_directories(./dtpddl)

add_definitions(-std=c++0x -mno-align-stringops -minline-all-stringops -march=native -fPIC)

#add_definitions(-DLAO_STAR -std=c++0x -mno-align-stringops -minline-all-stringops -march=native -fPIC)

#add_definitions(-pedantic -pg -Wall -Wextra)

add_cast_component(DTPCONTROL dtp-control.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Planner)
link_cast_component(${CAST_COMPONENT_NAME} dtpddl)
link_cast_component(${CAST_COMPONENT_NAME} ${Boost_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} pthread)
link_cast_component(${CAST_COMPONENT_NAME} m)
#target_link_libraries(${CAST_COMPONENT_NAME} dtplanning)
