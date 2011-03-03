
# -------------------- TomGine lib --------------------
PROJECT(SDTomGine)

FIND_PACKAGE( OpenCV REQUIRED )
#INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

ADD_DEFINITIONS(-DGL_GLEXT_PROTOTYPES)
ADD_DEFINITIONS(-DUSE_FTGL_FONT)
SET(TRUETYPE_FONT "/usr/share/fonts/truetype/freefont/FreeSans.ttf")
ADD_DEFINITIONS(-DTTF_FONT="${TRUETYPE_FONT}")

INCLUDE_DIRECTORIES(/usr/include/freetype2)

SET(SOURCES ply.c
	tgQuaternion.cpp
	tgCamera.cpp
	tgCollission.cpp
	tgEngine.cpp
	tgError.cpp
	tgErrorMetric.cpp
	tgFont.cpp
	tgFrustum.cpp
	tgGUI.cpp
	tgImageProcessor.cpp
	tgLabel.cpp
	tgLighting.cpp
	tgMaterial.cpp
	tgMatrix3.cpp
	tgModel.cpp
	tgModelLoader.cpp
	tgPlot2D.cpp
	tgPose.cpp
	tgRenderModel.cpp
	tgShader.cpp
	tgShapeCreator.cpp
	tgTexture.cpp
	tgVector3.cpp
	tgTimer.cpp)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
LINK_LIBRARIES(ftgl SDGLWindow rt)
#target_link_libraries(${PROJECT_NAME} SDGLWindow)
#target_link_libraries(${PROJECT_NAME} ${OPENCV_LDFLAGS})
#LINK_LIBRARIES(SDGLWindow)
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

# -------------------- TomGine executable --------------------
#PROJECT(SDTomGineDemo)
#LINK_LIBRARIES(${OpenCV_LIBS})
#LINK_LIBRARIES(SDTomGine ftgl SDGLWindow rt)
#target_link_libraries(${PROJECT_NAME} ${OPENCV_LDFLAGS})
#ADD_EXECUTABLE(${PROJECT_NAME} main.cpp)
#INSTALL(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

