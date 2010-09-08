include(Slice2Cpp)

include_directories(.)

cast_slice2cpp(GEN_CPP GEN_HPP ComaData.ice ${CAST_SLICE_DIR})
add_library(ComaData SHARED ${GEN_CPP})
target_link_libraries(ComaData CDL) # Is this required?
install(TARGETS ComaData LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
