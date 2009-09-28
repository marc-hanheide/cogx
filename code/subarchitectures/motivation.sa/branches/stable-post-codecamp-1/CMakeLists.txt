project (Motivation)

# use cmake files packaged with CAST
include(UseCAST)

set(BINDER_DIR ${COGX_ROOT}/subarchitectures/binder)
set(BINDER_SLICE ${BINDER_DIR}/src/slice)
set(BINDER_INCLUDE ${BINDER_DIR}/src/c++/autogen)

set(PLANNER_DIR ${COGX_ROOT}/subarchitectures/planner.sa)
set(PLANNER_SLICE ${PLANNER_DIR}/src/slice)
set(PLANNER_INCLUDE ${PLANNER_DIR}/src/c++)

include_directories(src/c++)
include_directories(src/c++/autogen)
include_directories(${BINDER_INCLUDE})
include_directories(${PLANNER_INCLUDE})


# add a subdirectory containing another CMakeLists.txt file
add_subdirectory(src/c++)
add_subdirectory(src/c++/autogen)

