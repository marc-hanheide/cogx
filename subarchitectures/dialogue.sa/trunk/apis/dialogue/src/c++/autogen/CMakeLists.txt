include(Slice2Cpp)

include_directories(.)

cast_slice2cpp(GEN_CPP GEN_HPP ref.ice ../../slice ${COGX_ROOT}/tools/beliefs/apis/beliefs/src/slice)

add_library(RefSlice SHARED ${GEN_CPP})
target_link_libraries(Beliefs)
install(TARGETS RefSlice LIBRARY DESTINATION lib)
