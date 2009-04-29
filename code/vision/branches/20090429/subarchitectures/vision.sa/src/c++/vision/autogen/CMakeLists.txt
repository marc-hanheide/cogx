include(Slice2Cpp)

include_directories(.)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP VisionData.ice ${CAST_SLICE_DIR} ${COGX_ROOT}/tools/math/src/slice)
add_library(VisionData SHARED ${GEN_CPP})
target_link_libraries(VisionData CDL)
target_link_libraries(VisionData Math)
install(TARGETS VisionData LIBRARY DESTINATION lib)


