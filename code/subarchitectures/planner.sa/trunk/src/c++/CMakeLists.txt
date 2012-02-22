include_directories(.)
#include_directories(../../../fakebinder.sa/src/c++)
#include_directories(../../../binder/src/c++)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs+cast/src/cpp/autogen)
include_directories(${COGX_ROOT}/tools/cogxutils/src/c++/autogen)
include(Slice2Cpp)

cast_slice2cpp(GEN_CPP GEN_HPP Planner.ice ${CAST_SLICE_DIR} 
  ${COGX_ROOT}/tools/beliefs/apis/beliefs/src/slice
  ${COGX_ROOT}/tools/beliefs/apis/beliefs+cast/src/slice)

add_library(Planner SHARED ${GEN_CPP})
target_link_libraries(Planner Beliefs BeliefsCAST)
target_link_libraries(Planner CDL)
install(TARGETS Planner LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

if (DO_SLICE_ONLY)
   return()
endif()

find_package(Boost 1.35.0 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_cast_component(WMControl WMControl.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Planner BeliefsCogX)
link_cast_component(${CAST_COMPONENT_NAME} ${Boost_LIBRARIES})


option(BUILD_SA_PLANNER_DT "Build DT planner in planner.sa" YES)
if(BUILD_SA_PLANNER_DT)
  add_subdirectory(dtplanner)
endif(BUILD_SA_PLANNER_DT)

option(BUILD_SA_PLANNER_BNET "Build bayesian network evaluator" NO)
if(BUILD_SA_PLANNER_BNET)
  add_subdirectory(bnet)
endif(BUILD_SA_PLANNER_BNET)

