
find_package(PythonLibs)
include_directories(. ${PYTHON_INCLUDE_PATH})

add_cast_component(ObjectRecognizer Recognizer.cpp)
get_filename_component(PYLIBNAME "${PYTHON_LIBRARY}" NAME)
set_target_properties(ObjectRecognizer PROPERTIES
   COMPILE_FLAGS -DPYTHON_LIB_NAME="\\"${PYLIBNAME}\\""
   )

link_cast_component(${CAST_COMPONENT_NAME}
   VisionData Video VideoUtils
   ${PYTHON_LIBRARY})

add_cast_component(ObjectRecognizerSrv RecognizerSrv.cpp pythonproxy.cpp)
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData Video VideoUtils
   )

add_subdirectory(pysiftgpu/siftgpu)
add_subdirectory(pysiftgpu/siftcuda)
add_subdirectory(test)

