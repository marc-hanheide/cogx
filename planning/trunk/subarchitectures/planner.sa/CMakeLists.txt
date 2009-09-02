project (planner.sa)

# use cmake files packaged with CAST
include(UseCAST)

# add a subdirectory containing another CMakeLists.txt file
add_subdirectory(src/c++)
add_subdirectory(src/python)

