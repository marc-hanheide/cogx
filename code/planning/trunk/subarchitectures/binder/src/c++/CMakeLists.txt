include_directories(.)
include(Slice2Cpp)

cast_slice2cpp(GEN_CPP GEN_HPP BinderEssentials.ice ${CAST_SLICE_DIR})
add_library(BinderEssentials SHARED ${GEN_CPP})
install(TARGETS BinderEssentials LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

