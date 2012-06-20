include(Slice2Cpp)
include_directories(.)

# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP_SD GEN_HPP_SD ConceptualData.ice ${CAST_SLICE_DIR} 
               ${COGX_ROOT}/subarchitectures/spatial.sa/src/slice
               ${COGX_ROOT}/tools/math/src/slice)
add_library(ConceptualData SHARED ${GEN_CPP_SD})
target_link_libraries(ConceptualData CDL SpatialProbabilities SpatialData Math)
install(TARGETS ConceptualData LIBRARY DESTINATION lib)

