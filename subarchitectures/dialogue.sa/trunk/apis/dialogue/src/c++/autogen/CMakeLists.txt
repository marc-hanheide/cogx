include(Slice2Cpp)

include_directories(.)
include_directories(
   ${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen
   ${COGX_ROOT}/tools/abducer/src/c++/autogen
   )
set(SLICE_DIRS
   ${CAST_SLICE_DIR}
   ${COGX_ROOT}/tools/beliefs/apis/beliefs/src/slice
   ${COGX_ROOT}/tools/abducer/src/slice
   )

cast_slice2cpp(GEN_CPP_D GEN_HPP_D dialogue.ice ../../slice ${SLICE_DIRS})
cast_slice2cpp(GEN_CPP_T GEN_HPP_T time.ice ../../slice ${SLICE_DIRS})
cast_slice2cpp(GEN_CPP_R GEN_HPP_R ref.ice ../../slice ${SLICE_DIRS})
cast_slice2cpp(GEN_CPP_R GEN_HPP_L lf.ice ../../slice ${SLICE_DIRS})

add_library(DialogueIce SHARED ${GEN_CPP_D} ${GEN_CPP_T} ${GEN_CPP_R} ${GEN_HPP_L})
target_link_libraries(DialogueIce AbducerIce Beliefs CDL)

install(TARGETS DialogueIce LIBRARY DESTINATION lib)
