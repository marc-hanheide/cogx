include_directories(.)

include_directories(/Users/plison/svn.cogx/binder-dev/tools/cast/cmake)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP BinderEssentials.ice ${CAST_SLICE_DIR}
				 ../../slice)
add_library(BinderEssentials SHARED ${GEN_CPP})
target_link_libraries(BinderEssentials CDL)
install(TARGETS BinderEssentials LIBRARY DESTINATION lib)
