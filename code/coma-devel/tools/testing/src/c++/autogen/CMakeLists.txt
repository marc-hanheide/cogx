include(Slice2Cpp)

include_directories(.)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to search
cast_slice2cpp(GEN_CPP GEN_HPP TestingData.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/testing/src/slice)
add_library(TestingData SHARED ${GEN_CPP})
target_link_libraries(TestingData CDL)
install(TARGETS TestingData LIBRARY DESTINATION lib)



