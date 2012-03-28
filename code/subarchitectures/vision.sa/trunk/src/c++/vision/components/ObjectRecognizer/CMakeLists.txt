
project(ObjectRecognizer)

find_package(Cuda)

find_package(PythonLibs)
get_filename_component(PYLIBNAME "${PYTHON_LIBRARY}" NAME)
include_directories(.  sifts
   pysiftgpu
   ${PYTHON_INCLUDE_PATH}
   /usr/include/GL
   ../../libs
   ../../libs/SiftGPU/src/SiftGPU
   )

ENABLE_COGX_VISUALIZATION(TRUE)

link_directories(
   ${CUDA_INSTALLROOT}/lib
   )

add_definitions(-DDEBUG_TRACE)

#add_cast_component(ObjectRecognizer
#   Recognizer.cpp
#   convenience.cpp
#   )
#set_target_properties(ObjectRecognizer PROPERTIES
#   COMPILE_FLAGS -DPYTHON_LIB_NAME="\\"${PYLIBNAME}\\""
#   )
#link_cast_component(${CAST_COMPONENT_NAME}
#   VisionData Video VideoUtils
#   ${PYTHON_LIBRARY})

option(BUILD_SA_VISION_OBJRECOGNIZER_SRV "build object recognizer server" NO)
if (BUILD_SA_VISION_OBJRECOGNIZER_SRV)
add_cast_component(ObjectRecognizerSrv
   RecognizerSrv.cpp RecognizerClient.cpp
   #pythonproxy.cpp
   convenience.cpp
   sifts/Features.cpp
   sifts/ExtractorSiftGpu.cpp
   sifts/Matcher.cpp
   sifts/MatcherCudaSift.cpp
   models/ObjectModel.cpp
   models/ModelLoader.cpp
   )
set_target_properties(ObjectRecognizerSrv PROPERTIES
   COMPILE_FLAGS -DPYTHON_LIB_NAME="\\"${PYLIBNAME}\\""
   )
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData Video VideoUtils
   siftgpu cudasift
   sqlite3
   #${PYTHON_LIBRARY}
   )
link_cast_component(${CAST_COMPONENT_NAME} Video ${VISUALIZATION_LIBRARIES})
add_subdirectory(test) # TODO: move out, add if BUILD_SA_VISION_OBJRECOGNIZER_SRV in test/cmakelists
endif (BUILD_SA_VISION_OBJRECOGNIZER_SRV)

#add_subdirectory(pysiftgpu/siftgpu)
#add_subdirectory(pysiftgpu/siftcuda)

