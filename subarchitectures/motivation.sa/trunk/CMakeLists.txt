project (Motivation)

# use cmake files packaged with CAST
include(UseCAST)

set(BINDER_DIR ${COGX_ROOT}/subarchitectures/binder)
set(BINDER_SLICE ${BINDER_DIR}/src/slice)
set(BINDER_INCLUDE ${BINDER_DIR}/src/c++/autogen)

set(BELIEF_DIR ${COGX_ROOT}/tools/beliefs)
set(BELIEF_SLICE ${BELIEF_DIR}/apis/beliefs/src/slice ${BELIEF_DIR}/apis/beliefs+cast/src/slice)
set(BELIEF_INCLUDE ${BELIEF_DIR}/apis/beliefs/src/cpp/autogen ${BELIEF_DIR}/apis/beliefs+cast/src/cpp/autogen)

set(PLANNER_DIR ${COGX_ROOT}/subarchitectures/planner.sa)
set(PLANNER_SLICE ${PLANNER_DIR}/src/slice)
set(PLANNER_INCLUDE ${PLANNER_DIR}/src/c++)

include_directories(src/c++)
include_directories(src/c++/autogen)
include_directories(${BELIEF_INCLUDE})
include_directories(${PLANNER_INCLUDE})


# add a subdirectory containing another CMakeLists.txt file
add_subdirectory(src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

