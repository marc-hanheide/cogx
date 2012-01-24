project (planner.sa)
cmake_minimum_required(VERSION 2.6)

set(BASE_PLANNER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/src/base_planners")
add_subdirectory(src/base_planners)

if(NOT PLANNER_STANDALONE)
  # use cmake files packaged with CAST
  include(UseCAST)

  # add a subdirectory containing another CMakeLists.txt file
  add_subdirectory(src/c++)
else()
  option(DT_PLANNER_LAO_STAR "Use LAO* heuristic in dt planner" NO)
  if(DT_PLANNER_LAO_STAR)
    add_definitions(-DLAO_STAR)
  endif(DT_PLANNER_LAO_STAR)
  option(STATIC_BUILD "build static versions" NO)

  if(STATIC_BUILD)
    set(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} -static -static-libgcc")
  endif(STATIC_BUILD)

  add_subdirectory(src/c++/dtplanner/dtpddl)
  add_subdirectory(src/c++/bnet)
endif()

add_subdirectory(src/python)


