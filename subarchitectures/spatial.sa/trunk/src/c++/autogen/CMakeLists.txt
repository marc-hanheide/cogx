include_directories(.)
include(Slice2Cpp)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP_SD GEN_HPP_SD SpatialData.ice ${CAST_SLICE_DIR} ${COGX_ROOT}/tools/math/src/slice )
add_library(SpatialData SHARED ${GEN_CPP_SD})
target_link_libraries(SpatialData CDL NavData Math)
install(TARGETS SpatialData LIBRARY DESTINATION lib)

cast_slice2cpp(GEN_CPP GEN_HPP NavData.ice ${CAST_SLICE_DIR}
				 ${COGX_ROOT}/tools/math/src/slice)
add_library(NavData SHARED ${GEN_CPP})
target_link_libraries(NavData CDL)
target_link_libraries(NavData Math)
install(TARGETS NavData LIBRARY DESTINATION lib)


cast_slice2cpp(GEN_CPP_SB GEN_HPP_SB SpatialProbabilities.ice ${CAST_SLICE_DIR})
add_library(SpatialProbabilities SHARED ${GEN_CPP_SB})
target_link_libraries(SpatialProbabilities CDL)
install(TARGETS SpatialProbabilities LIBRARY DESTINATION lib)


cast_slice2cpp(GEN_CPP_SP GEN_HPP_SP SpatialProperties.ice ${CAST_SLICE_DIR})
add_library(SpatialProperties SHARED ${GEN_CPP_SP})
target_link_libraries(SpatialProperties CDL NavData SpatialData)
install(TARGETS SpatialProperties LIBRARY DESTINATION lib)

cast_slice2cpp(GEN_CPP_FI GEN_HPP_FI FrontierInterface.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/subarchitectures/nav.sa/src/slice
    ${COGX_ROOT}/tools/math/src/slice)
add_library(FrontierInterface SHARED ${GEN_CPP_FI})
target_link_libraries(FrontierInterface CDL NavData SpatialData)
install(TARGETS FrontierInterface LIBRARY DESTINATION lib)

#if(BUILD_SA_BINDING)
#  cast_slice2cpp(GEN_CPP_BE GEN_HPP_BE Marshalling.ice ${CAST_SLICE_DIR}
#    ${COGX_ROOT}/subarchitectures/binder/src/slice)
#  add_library(Marshalling SHARED ${GEN_CPP_BE})
#  target_link_libraries(Marshalling CDL NavData BinderEssentials)
#  install(TARGETS Marshalling LIBRARY DESTINATION lib)
#endif(BUILD_SA_BINDING)


