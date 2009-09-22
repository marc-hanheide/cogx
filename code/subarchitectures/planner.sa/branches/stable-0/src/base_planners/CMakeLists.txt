# add a subdirectory containing another CMakeLists.txt file

add_subdirectory(ContinualAxFF)

set(TFD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tfd")

add_custom_target(tfd_preprocess
                   COMMAND make
                   WORKING_DIRECTORY ${TFD_DIR}/preprocess/
                   COMMENT "Building ${TFD_DIR}/preprocess/preprocess")
add_custom_target(tfd_search
                   COMMAND make
                   WORKING_DIRECTORY ${TFD_DIR}/search/
                   COMMENT "Building ${TFD_DIR}/search/search")

add_custom_target(tfd ALL)
add_dependencies(tfd tfd_preprocess tfd_search)
