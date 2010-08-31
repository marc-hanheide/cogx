include_directories(.)
include(Slice2Cpp)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP CELM.ice ${CAST_SLICE_DIR})
add_library(CELM SHARED ${GEN_CPP})
target_link_libraries(CELM CDL)
install(TARGETS CELM LIBRARY DESTINATION lib)


