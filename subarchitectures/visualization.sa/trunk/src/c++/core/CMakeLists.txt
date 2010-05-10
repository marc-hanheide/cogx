
project(VisualizationCore)
find_package(Qt4 REQUIRED) # http://qtnode.net/wiki/Qt4_with_cmake
include(${QT_USE_FILE} ${QT_USE_OPENGL})

set(TOMGINE_INCLUDE
   ${QT_QTOPENGL_INCLUDE_DIR}
   ${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/VirtualScene/TomGine/include
   /usr/include/GL
   /usr/include/freetype2
   )
set(TOMGINE_LIBS
   ${QT_QTOPENGL_LIBRARY}
   TomGine GL
   ftgl # requires package libftgl-dev
   )

add_definitions(-DDEBUG_TRACE)

set(SERVER_CPP
   ptrvector.cpp
   convenience.cpp
   CDisplayServer.cpp
   Model.cpp
   object/CSvgImage.cpp
   object/CRasterImage.cpp
   object/CTomGineModel.cpp
   )
set(GUI_CPP
   qtui/QCastApplication.cpp
   qtui/QCastMainFrame.cpp
   qtui/QCastView.cpp
   qtui/QCastViewGL.cpp
   qtui/QCustomGuiPanel.cpp
   qtui/ChangeSlot.cpp
   )
set(SERVER_MOC_HEADERS
   qtui/QCastApplication.hpp
   qtui/QCastMainFrame.hpp
   qtui/QCastView.hpp
   qtui/QCastViewGL.hpp
   qtui/ChangeSlot.hpp
   qtui/QCustomGuiPanel.hpp
   )
set(SERVER_UI_FILES
   qtui/design/castor.ui
   )
qt4_wrap_cpp (SERVER_MOC_OUTFILES ${SERVER_MOC_HEADERS})
qt4_wrap_ui  (SERVER_UIC_OUTFILES ${SERVER_UI_FILES})
include_directories( .  #./object
   ${VisualizationCore_BINARY_DIR}
   ${QT_QTSVG_INCLUDE_DIR}
   ${TOMGINE_INCLUDE}
   )

add_cast_component(DisplayServer
   ${SERVER_CPP} ${GUI_CPP}
   ${SERVER_MOC_OUTFILES} ${SERVER_UIC_OUTFILES})
link_cast_component(${CAST_COMPONENT_NAME}
   VisualizationData Math Video VideoUtils
   ${QT_LIBRARIES}
   ${QT_QTSVG_LIBRARY}
   ${TOMGINE_LIBS}
   )

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

add_cast_component(DisplayClient CDisplayClient.cpp)
link_cast_component(${CAST_COMPONENT_NAME}
   VisualizationData Math Video VideoUtils
   ${OPENCV_LDFLAGS}
   )

add_subdirectory(test)

