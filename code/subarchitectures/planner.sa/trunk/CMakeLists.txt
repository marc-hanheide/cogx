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
  add_subdirectory(src/c++/dtplanner/dtpddl)
  add_subdirectory(src/c++/bnet)
endif()

add_subdirectory(src/python)


