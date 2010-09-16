cmake_minimum_required (VERSION 2.6)
project (Dialogue)

# auto generated code
add_subdirectory (apis/cast/)
add_subdirectory (apis/dialogue/)

configure_file(${PROJECT_SOURCE_DIR}/build-abducer.sh.in ${PROJECT_SOURCE_DIR}/build-abducer.sh)

ADD_CUSTOM_TARGET( abducer_build ALL DEPENDS ./build-abducer.sh)
ADD_CUSTOM_COMMAND(
  TARGET  abducer_build
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMAND bash ./build-abducer.sh

)
