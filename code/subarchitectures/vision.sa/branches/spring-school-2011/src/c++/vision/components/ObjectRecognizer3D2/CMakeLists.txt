# needed to allow visualization to use IplImages 
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

FIND_PACKAGE( OpenCV REQUIRED )

include_directories(.)
include_directories(./ODetect3D)
include_directories(./matrix)
include_directories(./SiftGPU/src/SiftGPU)
include_directories(${VISUALIZATION_INCLUDE_DIRS})

link_directories(${CUDA_INSTALLROOT}/lib)

add_subdirectory(matrix)
#add_subdirectory(SiftGPU/linux)

# Set the sources.
SET(SOURCES 
		ObjectRecognizer3D2.cpp
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

add_cast_component(ObjectRecognizer3D2 ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData Stereo StereoClient VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} ${VISUALIZATION_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} matrix2 siftgpu)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS} Cg CgGL GLEW glut IL)

