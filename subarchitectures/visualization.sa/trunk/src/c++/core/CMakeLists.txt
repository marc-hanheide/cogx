
project(VisualizationCore)
find_package(Qt4 REQUIRED) # http://qtnode.net/wiki/Qt4_with_cmake
include(${QT_USE_FILE} )
# message(" QQQQQQQQQQQ " ${QT_USE_FILE} ${QT_QTSVG_INCLUDE_DIR})

#find_package(PythonLibs)
#get_filename_component(PYLIBNAME "${PYTHON_LIBRARY}" NAME)
#include_directories(. ${PYTHON_INCLUDE_PATH})

add_definitions(-DDEBUG_TRACE)

set(SERVER_CPP
   ptrvector.cpp
   convenience.cpp
   CDisplayServer.cpp
   Model.cpp
   object/CSvgImage.cpp
   object/CRasterImage.cpp
   )
set(GUI_CPP
   qtui/QCastApplication.cpp
   qtui/QCastMainFrame.cpp
   qtui/QCastView.cpp
   qtui/QCustomGuiPanel.cpp
   qtui/ChangeSlot.cpp
   )
set(SERVER_MOC_HEADERS
   qtui/QCastApplication.hpp
   qtui/QCastMainFrame.hpp
   qtui/QCastView.hpp
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
   ${QT_QTSVG_INCLUDE_DIR})

add_cast_component(DisplayServer
   ${SERVER_CPP} ${GUI_CPP}
   ${SERVER_MOC_OUTFILES} ${SERVER_UIC_OUTFILES})
#set_target_properties(DisplayServer PROPERTIES
#   COMPILE_FLAGS -DPYTHON_LIB_NAME="\\"${PYLIBNAME}\\""
#   )
link_cast_component(${CAST_COMPONENT_NAME}
   VisualizationData Math Video VideoUtils
   ${QT_LIBRARIES}
   ${QT_QTSVG_LIBRARY}
   # ${PYTHON_LIBRARY}
   )

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

add_cast_component(DisplayClient CDisplayClient.cpp)
#set_target_properties(ObjectRecognizerSrv PROPERTIES
#   COMPILE_FLAGS -DPYTHON_LIB_NAME="\\"${PYLIBNAME}\\""
#   )
link_cast_component(${CAST_COMPONENT_NAME}
   VisualizationData Math Video VideoUtils
   ${OPENCV_LDFLAGS}
   # ${PYTHON_LIBRARY}
   )

add_subdirectory(test)

