include(${CAST_ROOT}/cmake/Slice2Cpp.cmake)

set(slice_source CDL.ice)

cast_slice2cpp(GEN_CPP GEN_HPP ${slice_source})

add_library(CDL SHARED ${GEN_CPP} ${GEN_HPP})

install(TARGETS CDL LIBRARY DESTINATION lib/cast ARCHIVE DESTINATION lib/cast)
install(FILES ${GEN_HPP} DESTINATION include/cast/cdl)
install(FILES ${slice_source} DESTINATION share/cast/slice)
