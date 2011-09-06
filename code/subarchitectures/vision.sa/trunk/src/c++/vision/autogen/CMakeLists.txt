include(Slice2Cpp)

include_directories(.)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP VisionData.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/math/src/slice
    ${COGX_ROOT}/tools/hardware/laser/src/slice
    ${COGX_ROOT}/tools/hardware/pointCloud/src/slice
    ${COGX_ROOT}/tools/hardware/video/src/slice)


cast_slice2cpp(GEN_CPP_ML GEN_HPP_ML MatlabData.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/math/src/slice
    ${COGX_ROOT}/tools/hardware/video/src/slice)


cast_slice2cpp(GEN_CPP_OR GEN_HPP_OR ObjectRecognizerSrv.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/math/src/slice
    ${COGX_ROOT}/tools/hardware/video/src/slice)

add_library(VisionData SHARED ${GEN_CPP} ${GEN_CPP_ML} ${GEN_CPP_OR})
target_link_libraries(VisionData PointCloud Video Math CDL)
install(TARGETS VisionData LIBRARY DESTINATION lib)


