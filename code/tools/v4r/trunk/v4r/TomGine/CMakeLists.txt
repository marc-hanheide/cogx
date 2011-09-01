SET(V4R_TOMGINE 1 CACHE BOOL "OpenGL-GPU rendering, geometry and image processing library. (depends: GLWindow)" )
IF(V4R_TOMGINE)

SET(TOMGINE_CPP ply.c
	tgCamera.cpp
	tgCollission.cpp
	tgEngine.cpp
	tgError.cpp
	tgErrorMetric.cpp
	tgFont.cpp
	tgFrameBufferObject.cpp
	tgFrustum.cpp
	tgHistDesc2D.cpp
	tgHistogram.cpp
	tgImageProcessor.cpp
	tgLight.cpp
	tgMaterial.cpp
	tgMathlib.cpp
	tgModel.cpp
	tgModelLoader.cpp
	tgNurbsCurve.cpp
	tgNurbsSurface.cpp
	tgNurbsVolume.cpp
	tgPlot2D.cpp
	tgPose.cpp
	tgQuaternion.cpp
	tgRenderModel.cpp
	tgShader.cpp
	tgShapeCreator.cpp
	tgTexture.cpp
	tgTimer.cpp
	tgTomGineThread.cpp)
add_library(v4rtomgine SHARED ${TOMGINE_CPP})
target_link_libraries(v4rtomgine ${OPENCV_LIBRARIES} ftgl freetype v4rglwindow pthread rt)
add_subdirectory(example)

ENDIF(V4R_TOMGINE)
