include_directories(.)
include_directories(${COGX_ROOT}/subarchitectures/nav.sa/src/c++/autogen)
include(Slice2Cpp)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP SpatialData.ice ${CAST_SLICE_DIR})
cast_slice2cpp(GEN_CPP GEN_HPP FrontierInterface.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/subarchitectures/nav.sa/src/slice)
add_library(SpatialData SHARED ${GEN_CPP})
add_library(FrontierInterface SHARED ${GEN_CPP})
target_link_libraries(SpatialData CDL)
target_link_libraries(FrontierInterface CDL)
install(TARGETS SpatialData LIBRARY DESTINATION lib)
install(TARGETS FrontierInterface LIBRARY DESTINATION lib)


