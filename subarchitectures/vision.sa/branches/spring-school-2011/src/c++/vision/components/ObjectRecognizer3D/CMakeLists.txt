# needed to allow visualization to use IplImages 
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

include_directories(.)
include_directories(./ODetect3D)
include_directories(./matrix)
include_directories(./SiftGPU/src/SiftGPU)
include_directories(../ObjectTracker)
include_directories(../ObjectTracker/Tracker/include)
include_directories(/usr/include/opencv)
include_directories(${VISUALIZATION_INCLUDE_DIRS})

link_directories(${CUDA_INSTALLROOT}/lib)

add_subdirectory(matrix)
#add_subdirectory(SiftGPU/linux)

# Set the sources.
SET(SOURCES 
		ObjectRecognizer3D.cpp
		ODetect3D/ODetect3D.cc
		ODetect3D/CodebookEntry.cc
		ODetect3D/Object3D.cc
		ODetect3D/PoseCv.cc
		ODetect3D/ModelObject3D.cc
		ODetect3D/Keypoint.cc
		ODetect3D/KeypointDescriptor.cc
		ODetect3D/DetectGPUSIFT.cc
		ODetect3D/DetectSIFT.cc
		ODetect3D/Vector2.cc
		ODetect3D/SDraw.cc
		ODetect3D/SPolygon.cc
		ODetect3D/Homography.cc
		ODetect3D/Geometry.cc
		ODetect3D/Except.cc
		ODetect3D/Math.cc
		ODetect3D/Config.cc
		ODetect3D/ConfigFile.cc)

add_cast_component(ObjectRecognizer3D ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} ${VISUALIZATION_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} Tracker matrix siftgpu)
link_cast_component(${CAST_COMPONENT_NAME} cv highgui cxcore Cg CgGL GLEW glut IL)

