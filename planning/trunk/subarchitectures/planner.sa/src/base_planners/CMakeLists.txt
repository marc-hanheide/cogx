# add a subdirectory containing another CMakeLists.txt file

add_subdirectory(ContinualAxFF)

set(TFD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tfd")

add_custom_command(OUTPUT ${TFD_DIR}/preprocess/preprocess
                   COMMAND make
                   WORKING_DIRECTORY ${TFD_DIR}/preprocess/
                   COMMENT "Building ${TFD_DIR}/preprocess/preprocess")
add_custom_command(OUTPUT ${TFD_DIR}/search/search
                   COMMAND make
                   WORKING_DIRECTORY ${TFD_DIR}/search/
                   COMMENT "Building ${TFD_DIR}/search/search")

add_custom_target(tfd ALL 
                  DEPENDS ${TFD_DIR}/preprocess/preprocess
                          ${TFD_DIR}/search/search)