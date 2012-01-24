include_directories(.)


include(Slice2Cpp)


cast_slice2cpp(GEN_CPP_E GEN_HPP_E George.ice ${CAST_SLICE_DIR} ${EXECUTION_SLICE} ${MOTIVATION_SLICE} ${PLANNER_SLICE} ${BELIEF_SLICE})
add_library(GeorgeData SHARED ${GEN_CPP_E})
target_link_libraries(GeorgeData CDL Planner ExecutionData	MotivationData)
install(TARGETS GeorgeData LIBRARY DESTINATION lib)

