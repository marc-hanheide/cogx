project(DialogInteraction)

include_directories(..)
include_directories(.)
#include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
#include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(
   ${COGX_ROOT}/tools/castutils/src/c++/castutils
   ${COGX_ROOT}/tools/math/src/c++/autogen
   ${COGX_ROOT}/tools/math/src/c++/math
   # VisionData -> PointCloud -> Laser
   ${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen
   ${COGX_ROOT}/tools/hardware/laser/src/c++/autogen
   # Dialogue -> Beliefs, Abducer
   ${COGX_ROOT}/subarchitectures/dialogue.sa/apis/dialogue/src/c++/autogen
   ${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen
   ${COGX_ROOT}/tools/abducer/src/c++/autogen
   # motivation.sa
   ${COGX_ROOT}/subarchitectures/motivation.sa/src/c++/autogen
   # manipulation.sa
   ${COGX_ROOT}/subarchitectures/manipulation.sa/src/c++/autogen
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

ENABLE_COGX_VISUALIZATION(TRUE)

option(BUILD_GEORGE_V11N_TRACE "Build George Event Tracer using V11N" YES)
if(BUILD_GEORGE_V11N_TRACE)

   add_cast_component(GeorgeEventTrace TraceEvents.cpp TraceEvents.hpp)
   link_cast_component(${CAST_COMPONENT_NAME}
      ${VISUALIZATION_LIBRARIES}
      VisionData
      MotivationData
      DialogueIce
      ManipulationData
      )

   #add_custom_target (DialogInteraction.Resources
   #   DEPENDS res/dialogue_gen.inc)
   #add_custom_command (
   #   OUTPUT res/dialogue_gen.inc
   #   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/res
   #   COMMENT "make DialogInteraction.Resources"
   #   COMMAND make
   #   )
   #add_dependencies(DialogInteraction DialogInteraction.Resources)

endif()

