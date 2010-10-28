include_directories(.)
include(Slice2Cpp)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP BlobFinder.ice ${CAST_SLICE_DIR} ${COGX_ROOT}/tools/math/src/slice)
add_library(BlobFinder SHARED ${GEN_CPP})
target_link_libraries(BlobFinder CDL Math ${ICE_LIBS})
install(TARGETS BlobFinder LIBRARY DESTINATION lib)


