include_directories(.)
include_directories(./ODetect3D)
include_directories(./matrix)
include_directories(./SiftGPU/SiftGPU/src)
include_directories(../ObjectTracker)
include_directories(../ObjectTracker/Tracker/include)
include_directories(../VirtualScene/TomGine/include)
include_directories(../VirtualScene)
include_directories(/usr/include/opencv)

add_subdirectory(matrix)
add_subdirectory(SiftGPU/Linux)


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

#link_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/ObjectRecognizer3D/matrix)
link_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/ObjectRecognizer3D/SiftGPU/Linux/bin)

add_cast_component(ObjectRecognizer3D ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} Tracker TomGine matrix siftgpu)
link_cast_component(${CAST_COMPONENT_NAME} cv highgui cxcore Cg CgGL GLEW glut IL)

