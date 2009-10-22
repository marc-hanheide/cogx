include(Slice2Cpp)

include_directories(.)
include_directories(${COGX_ROOT}/tools/beliefmodels/src/c++/autogen)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP BeliefModels.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/beliefmodels/src/slice)

add_library(BeliefModels SHARED ${GEN_CPP})
target_link_libraries(BeliefModels CDL)
install(TARGETS BeliefModels LIBRARY DESTINATION lib)

cast_slice2cpp(GEN_CPP GEN_HPP DomainModel.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/beliefmodels/src/slice)

add_library(DomainModel SHARED ${GEN_CPP})
target_link_libraries(DomainModel BeliefModels)
install(TARGETS DomainModel LIBRARY DESTINATION lib)

