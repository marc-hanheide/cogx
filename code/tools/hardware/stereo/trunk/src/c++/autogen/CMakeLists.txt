include(Slice2Cpp)

include_directories(.)
include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/autogen)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP Stereo.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/math/src/slice
    ${COGX_ROOT}/tools/hardware/video/src/slice
    ${COGX_ROOT}/subarchitectures/vision.sa/src/slice)
add_library(Stereo SHARED ${GEN_CPP})
target_link_libraries(Stereo VisionData CDL  Ice IceUtil Math)
install(TARGETS Stereo LIBRARY DESTINATION lib)


