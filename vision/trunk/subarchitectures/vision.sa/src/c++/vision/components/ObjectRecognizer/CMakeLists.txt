
find_package(PythonLibs)
include_directories(. ${PYTHON_INCLUDE_PATH})

add_cast_component(ObjectRecognizer Recognizer.cpp)
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData Video VideoUtils VideoClient
   ${PYTHON_LIBRARY})

add_subdirectory(pysiftgpu/siftgpu)
add_subdirectory(pysiftgpu/siftcuda)

