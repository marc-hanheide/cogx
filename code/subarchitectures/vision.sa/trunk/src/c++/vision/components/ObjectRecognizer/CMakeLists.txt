
find_package(PythonLibs)
get_filename_component(PYLIBNAME "${PYTHON_LIBRARY}" NAME)
include_directories(. ${PYTHON_INCLUDE_PATH})

add_cast_component(ObjectRecognizer Recognizer.cpp)
set_target_properties(ObjectRecognizer PROPERTIES
   COMPILE_FLAGS -DPYTHON_LIB_NAME="\\"${PYLIBNAME}\\""
   )
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData Video VideoUtils
   ${PYTHON_LIBRARY})

option(BUILD_SA_VISION_OBJRECOGNIZER_SRV "build object recognizer server" NO)
if (BUILD_SA_VISION_OBJRECOGNIZER_SRV)
add_cast_component(ObjectRecognizerSrv RecognizerSrv.cpp RecognizerClient.cpp pythonproxy.cpp)
set_target_properties(ObjectRecognizerSrv PROPERTIES
   COMPILE_FLAGS -DPYTHON_LIB_NAME="\\"${PYLIBNAME}\\""
   )
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData Video VideoUtils
   ${PYTHON_LIBRARY})
add_subdirectory(test) # TODO: move out, add if BUILD_SA_VISION_OBJRECOGNIZER_SRV in test/cmakelists
endif (BUILD_SA_VISION_OBJRECOGNIZER_SRV)

add_subdirectory(pysiftgpu/siftgpu)
add_subdirectory(pysiftgpu/siftcuda)

