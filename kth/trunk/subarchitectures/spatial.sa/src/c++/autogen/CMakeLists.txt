include_directories(.)
include_directories(${COGX_ROOT}/subarchitectures/nav.sa/src/c++/autogen)
include(Slice2Cpp)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP SpatialData.ice ${CAST_SLICE_DIR})
cast_slice2cpp(GEN_CPP GEN_HPP SpatialProperties.ice ${CAST_SLICE_DIR})
cast_slice2cpp(GEN_CPP GEN_HPP FrontierInterface.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/subarchitectures/nav.sa/src/slice
    ${COGX_ROOT}/tools/math/src/slice)

add_library(SpatialData SHARED ${GEN_CPP})
add_library(SpatialProperties SHARED ${GEN_CPP})
add_library(FrontierInterface SHARED ${GEN_CPP})

target_link_libraries(SpatialData CDL NavData)
target_link_libraries(SpatialProperties CDL NavData)
target_link_libraries(FrontierInterface CDL NavData)

install(TARGETS SpatialData LIBRARY DESTINATION lib)
install(TARGETS SpatialProperties LIBRARY DESTINATION lib)
install(TARGETS FrontierInterface LIBRARY DESTINATION lib)
if(BUILD_SA_BINDING)
  cast_slice2cpp(GEN_CPP GEN_HPP Marshalling.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/subarchitectures/binder/src/slice)
  add_library(Marshalling SHARED ${GEN_CPP})
  target_link_libraries(Marshalling CDL)
  install(TARGETS Marshalling LIBRARY DESTINATION lib)
endif(BUILD_SA_BINDING)


