project(CogX.George.Testing)

Find_package(Player)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(
   ${COGX_ROOT}/tools/castutils/src/c++
   ${COGX_ROOT}/tools/math/src/c++/autogen
   ${COGX_ROOT}/tools/math/src/c++/math
   )
set(SOURCES
   tester.cpp
   statemachine.cpp
   castmachine.cpp
   y4learning.cpp
   )

add_cast_component(GeorgeTester ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME}
   ${PLAYER_LDFLAGS}
#   VisionData VideoUtils
#   DialogueData
#   Beliefs
)

