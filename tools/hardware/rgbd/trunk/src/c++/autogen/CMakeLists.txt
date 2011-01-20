include_directories(.)
include(Slice2Cpp)

# expects to find .ice file under PROJECT/src/slice e.g. rgbd/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP RGBD.ice ${CAST_SLICE_DIR})
add_library(RGBD SHARED ${GEN_CPP})
target_link_libraries(RGBD CDL ${ICE_LIBS})
install(TARGETS RGBD LIBRARY DESTINATION lib)


