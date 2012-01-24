include_directories(.)
include(Slice2Cpp)

include_directories(${COGX_ROOT}/tools/hardware/laser/src/slice)

#include_directories(${COGX_ROOT}/tools/math/src/c++/autogen/)
#include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen/)
#include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen/)
#include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/autogen)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP PointCloud.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/math/src/slice
    ${COGX_ROOT}/tools/hardware/video/src/slice
    ${COGX_ROOT}/tools/hardware/laser/src/slice)

#    ${COGX_ROOT}/subarchitectures/vision.sa/src/slice)
add_library(PointCloud SHARED ${GEN_CPP})
target_link_libraries(PointCloud CDL  Ice IceUtil Math Video Laser)
install(TARGETS PointCloud LIBRARY DESTINATION lib)


