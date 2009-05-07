include_directories(.)
include(Slice2Cpp)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP NavData.ice ${CAST_SLICE_DIR})
add_library(NavData SHARED ${GEN_CPP})
target_link_libraries(NavData CDL)
install(TARGETS NavData LIBRARY DESTINATION lib)


