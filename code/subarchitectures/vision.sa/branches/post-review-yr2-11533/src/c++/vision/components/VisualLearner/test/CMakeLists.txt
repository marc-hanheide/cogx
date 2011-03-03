include_directories(.)
include_directories(
   ${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen
   ${COGX_ROOT}/subarchitectures/dialogue.sa/apis/dialogue/src-generated/cpp
   )

# Set the sources.
SET(SOURCES
   TestLearner.cpp
   TestCases.cpp
   gy2article.cpp
   )

add_cast_component(TestVisualLearner ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData VideoUtils
   DialogueData
   Beliefs)

