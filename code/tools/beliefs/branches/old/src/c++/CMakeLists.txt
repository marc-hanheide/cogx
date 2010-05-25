include_directories(.)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP beliefmodels.ice ${CAST_SLICE_DIR})
add_library(beliefmodels SHARED ${GEN_CPP})
target_link_libraries(beliefmodels CDL)
install(TARGETS beliefmodels LIBRARY DESTINATION lib)
