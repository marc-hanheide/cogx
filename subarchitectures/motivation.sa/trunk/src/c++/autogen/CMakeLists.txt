include_directories(.)


include(Slice2Cpp)

cast_slice2cpp(GEN_CPP_M GEN_HPP_M motivation.ice ${CAST_SLICE_DIR} ${PLANNER_SLICE} ${BELIEF_SLICE})
add_library(MotivationData SHARED ${GEN_CPP_M})
target_link_libraries(MotivationData CDL BeliefsCAST Planner)
install(TARGETS MotivationData LIBRARY DESTINATION lib)

