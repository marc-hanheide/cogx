cmake_minimum_required (VERSION 2.6)
project (george)
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

set(EXECUTION_DIR ${COGX_ROOT}/subarchitectures/execution.sa)
set(EXECUTION_SLICE ${EXECUTION_DIR}/src/slice)
set(EXECUTION_INCLUDE ${EXECUTION_DIR}/src/c++/autogen)

set(MOTIVATION_DIR ${COGX_ROOT}/subarchitectures/motivation.sa)
set(MOTIVATION_SLICE ${MOTIVATION_DIR}/src/slice)
set(MOTIVATION_INCLUDE ${MOTIVATION_DIR}/src/c++/autogen)


include_directories(src/c++)
include_directories(src/c++/autogen)
include_directories(${BELIEF_INCLUDE})
include_directories(${PLANNER_INCLUDE})
include_directories(${EXECUTION_INCLUDE})
include_directories(${MOTIVATION_INCLUDE})

add_subdirectory(src/c++/autogen)


