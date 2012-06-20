include_directories(.)
include(Slice2Cpp)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP Video.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/math/src/slice)
add_library(Video SHARED ${GEN_CPP})
target_link_libraries(Video CDL  Ice IceUtil Math)
install(TARGETS Video LIBRARY DESTINATION lib)


