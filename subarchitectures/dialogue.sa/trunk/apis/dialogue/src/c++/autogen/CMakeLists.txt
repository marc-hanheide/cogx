include(Slice2Cpp)

include_directories(.)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen)

cast_slice2cpp(GEN_CPP GEN_HPP time.ice ../../slice)
add_library(TimeSlice SHARED ${GEN_CPP})
target_link_libraries(TimeSlice Beliefs CDL)

cast_slice2cpp(GEN_CPP GEN_HPP ref.ice ../../slice ${COGX_ROOT}/tools/beliefs/apis/beliefs/src/slice ${CAST_SLICE_DIR})
add_library(RefSlice SHARED ${GEN_CPP})

target_link_libraries(RefSlice Beliefs CDL)

install(TARGETS RefSlice LIBRARY DESTINATION lib)
install(TARGETS TimeSlice LIBRARY DESTINATION lib)
