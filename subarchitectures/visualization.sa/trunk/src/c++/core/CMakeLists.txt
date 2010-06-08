
project(VisualizationCore)
find_package(Qt4 REQUIRED) # http://qtnode.net/wiki/Qt4_with_cmake
include(${QT_USE_FILE} ${QT_USE_OPENGL})

set(EXPERIMENT 1)
if(EXPERIMENT)
   find_package(Lua51)
   find_library(LUA_LIB_TOLUA_51 tolua++5.1)
   if (LUA51_FOUND AND LUA_LIB_TOLUA_51)
      include_directories(object/gllua ${LUA_INCLUDE_DIR})
      set(EXPERIMENT_SRC
         object/gllua/glbind.cpp
         object/gllua/glubind.cpp
         object/gllua/gllbuffer.cpp
         object/gllua/gluaux.cpp
         object/CLuaGlScript.cpp
         )
      set(EXPERIMENT_LIB
         GL GLU
         ${LUA_LIBRARIES}
         ${LUA_LIB_TOLUA_51}
         )
      add_definitions(-DV11N_OBJECT_LUA_GL)
   endif(LUA51_FOUND AND LUA_LIB_TOLUA_51)
   if (NOT LUA51_FOUND)
      message ("Lua 5.1 not found. Please install liblua5.1-?-dev")
   endif (NOT LUA51_FOUND)
   if (NOT LUA_LIB_TOLUA_51)
      message ("libtolua++5.1 not found. Please install libtolua++5.1-dev")
   endif (NOT LUA_LIB_TOLUA_51)

endif(EXPERIMENT)

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
   GuiElements.cpp
   object/CSvgImage.cpp
   object/CRasterImage.cpp
   object/CTomGineModel.cpp
   ${EXPERIMENT_SRC}
   )
set(GUI_CPP
   qtui/QCastApplication.cpp
   qtui/QCastMainFrame.cpp
   qtui/QCastView.cpp
   qtui/QCastViewGL.cpp
   qtui/QCustomGuiPanel.cpp
   qtui/QViewContainer.cpp
   qtui/ChangeSlot.cpp
   )
set(SERVER_MOC_HEADERS
   qtui/QCastApplication.hpp
   qtui/QCastMainFrame.hpp
   qtui/QCastView.hpp
   qtui/QCastViewGL.hpp
   qtui/ChangeSlot.hpp
   qtui/QCustomGuiPanel.hpp
   qtui/QViewContainer.hpp
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
   boost_serialization-mt
   boost_iostreams-mt
   ${EXPERIMENT_LIB}
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

