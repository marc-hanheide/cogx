
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

add_cast_component(ObjectRecognizerSrv RecognizerSrv.cpp RecognizerClient.cpp pythonproxy.cpp)
set_target_properties(ObjectRecognizerSrv PROPERTIES
   COMPILE_FLAGS -DPYTHON_LIB_NAME="\\"${PYLIBNAME}\\""
   )
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData Video VideoUtils
   ${PYTHON_LIBRARY})

add_subdirectory(pysiftgpu/siftgpu)
add_subdirectory(pysiftgpu/siftcuda)
add_subdirectory(test)

