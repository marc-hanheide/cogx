include_directories(.)
include(Slice2Cpp)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP KinectPersonDetect.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/math/src/slice)
add_library(KinectPersonDetect SHARED ${GEN_CPP})
target_link_libraries(KinectPersonDetect CDL  Ice IceUtil)
install(TARGETS KinectPersonDetect LIBRARY DESTINATION lib)


