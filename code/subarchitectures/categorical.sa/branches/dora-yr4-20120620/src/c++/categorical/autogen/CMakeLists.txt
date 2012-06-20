include(Slice2Cpp)
include_directories(.)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP_SD GEN_HPP_SD CategoricalData.ice ${CAST_SLICE_DIR}
               ${COGX_ROOT}/subarchitectures/spatial.sa/src/slice
               ${COGX_ROOT}/tools/hardware/robotbase/src/slice
               ${COGX_ROOT}/tools/hardware/laser/src/slice)
add_library(CategoricalData SHARED ${GEN_CPP_SD})
target_link_libraries(CategoricalData CDL Robotbase Laser)
install(TARGETS CategoricalData LIBRARY DESTINATION lib)

