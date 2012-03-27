include_directories(../../../../matlab/cogxBuild/Proxy)
include_directories(../../../../matlab/Proxy)

include_directories(${VISUALIZATION_INCLUDE_DIRS})

project(VisualLearner)
link_directories(${VISUALLEARNER_LIBRARY_DIRS}) # Set in VisualLearner.cmake (dir for compiled or prebuilt files)
link_directories(${VISUALIZATION_LIBRARY_DIRS})

if (BUILD_SA_VISION_VISLEARNER_FAKE)
   add_definitions(-DFEAT_FAKE_LEARNER)
endif()

set(SOURCES VisualLearner.cpp)
set(HEADERS VisualLearner.h)
add_cast_component(VisualLearner ${SOURCES})

link_cast_component(${CAST_COMPONENT_NAME}
   VisionData
   ${VISUALLEARNER_LIBRARIES} # Set in VisualLearner.cmake (VisualLearnerProxy[_fake] [VisualLearnerCtf] )
   ${VISUALIZATION_LIBRARIES} # Set in v11n.cmake
   )

add_subdirectory(test)

