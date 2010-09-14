include_directories(.)


include(Slice2Cpp)


cast_slice2cpp(GEN_CPP_E GEN_HPP_E Execution.ice ${CAST_SLICE_DIR} ${PLANNER_SLICE} ${BELIEF_SLICE})
add_library(ExecutionData SHARED ${GEN_CPP_E})
target_link_libraries(ExecutionData CDL Planner)
install(TARGETS ExecutionData LIBRARY DESTINATION lib)

