project (planner.sa)

# use cmake files packaged with CAST
include(UseCAST)

set(BASE_PLANNER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/src/base_planners")

# add a subdirectory containing another CMakeLists.txt file
add_subdirectory(src/c++)
add_subdirectory(src/base_planners)
add_subdirectory(src/python)

