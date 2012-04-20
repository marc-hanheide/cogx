project(CogX.George.Testing)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(
   ${COGX_ROOT}/tools/castutils/src/c++
   )
set(SOURCES
   tester.cpp
   statemachine.cpp
   castmachine.cpp
   y4learning.cpp
   )

add_cast_component(GeorgeTester ${SOURCES})
#link_cast_component(${CAST_COMPONENT_NAME}
#   VisionData VideoUtils
#   DialogueData
#   Beliefs)

