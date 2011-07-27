# add a subdirectory containing another CMakeLists.txt file

option(BUILD_SA_PLANNER_FF "Build FF planner in planner.sa" NO)
if(BUILD_SA_PLANNER_FF)

add_subdirectory(ContinualAxFF)

endif(BUILD_SA_PLANNER_FF)

option(BUILD_SA_PLANNER_TFD "Build TFD planner in planner.sa" YES)
if(BUILD_SA_PLANNER_TFD)

set(TFD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tfd")

add_custom_target(tfd_preprocess
                   COMMAND make
                   WORKING_DIRECTORY ${TFD_DIR}/preprocess/
                   COMMENT "Building ${TFD_DIR}/preprocess/preprocess")
add_custom_target(tfd_search
                   COMMAND make
                   WORKING_DIRECTORY ${TFD_DIR}/search/
                   COMMENT "Building ${TFD_DIR}/search/search")

add_custom_target(tfd ALL)
add_dependencies(tfd tfd_preprocess tfd_search)

endif(BUILD_SA_PLANNER_TFD)

option(BUILD_SA_PLANNER_DOWNWARD "Build fast downward planner in planner.sa" YES)
if(BUILD_SA_PLANNER_DOWNWARD)

set(DOWNWARD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/downward")

add_custom_target(downward_preprocess
                   COMMAND make
                   WORKING_DIRECTORY ${DOWNWARD_DIR}/preprocess/
                   COMMENT "Building ${DOWNWARD_DIR}/preprocess/preprocess")
add_custom_target(downward_search
                   COMMAND make release CC=${CMAKE_CXX_COMPILER}
                   WORKING_DIRECTORY ${DOWNWARD_DIR}/search/
                   COMMENT "Building ${DOWNWARD_DIR}/search/search")

add_custom_target(downward ALL)
add_dependencies(downward downward_preprocess downward_search)

endif(BUILD_SA_PLANNER_DOWNWARD)

option(BUILD_SA_PLANNER_PROB_DOWNWARD "Build probabillistic fast downward planner in planner.sa" NO)
if(BUILD_SA_PLANNER_PROB_DOWNWARD)

set(PDOWNWARD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/prob_downward")

add_custom_target(pdownward_preprocess
                   COMMAND make
                   WORKING_DIRECTORY ${PDOWNWARD_DIR}/preprocess/
                   COMMENT "Building ${PDOWNWARD_DIR}/preprocess/preprocess")
add_custom_target(pdownward_search
                   COMMAND make release CC=${CMAKE_CXX_COMPILER}
                   WORKING_DIRECTORY ${PDOWNWARD_DIR}/search/
                   COMMENT "Building ${PDOWNWARD_DIR}/search/search")

add_custom_target(pdownward ALL)
add_dependencies(pdownward pdownward_preprocess pdownward_search)

endif(BUILD_SA_PLANNER_PROB_DOWNWARD)
