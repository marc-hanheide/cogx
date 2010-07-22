
project(VisualizationCore)
find_package(Qt4 REQUIRED) # http://qtnode.net/wiki/Qt4_with_cmake
include(${QT_USE_FILE} ${QT_USE_OPENGL})

option(BUILD_SA_V11N_USE_LUAGL "Build with LuaGl script support (requres lua51 and tolua++51)" ON)
if(BUILD_SA_V11N_USE_LUAGL)
   find_package(Lua51)
   find_library(LUA_LIB_TOLUA_51 tolua++5.1)
   if (LUA51_FOUND AND LUA_LIB_TOLUA_51)
      include_directories(object/gllua ${LUA_INCLUDE_DIR})
      set(EXPERIMENT_SRC ${EXPERIMENT_SRC}
         object/gllua/glbind.cpp
         object/gllua/glubind.cpp
         object/gllua/gllbuffer.cpp
         object/gllua/gluaux.cpp
         object/gllua/v11n.cpp
         object/gllua/v11nbind.cpp
         object/CLuaGlScript.cpp
         )
      set(EXPERIMENT_LIB ${EXPERIMENT_LIB}
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
endif(BUILD_SA_V11N_USE_LUAGL)

option(BUILD_SA_V11N_USE_WEBKIT "Build with HTML support (Qt-WebKit)" ON)
option(BUILD_SA_V11N_USE_WEBKIT_PLUGINS "Enable plugins in HTML pages (Qt-WebKit)" ON)
if(BUILD_SA_V11N_USE_WEBKIT)
   set(EXPERIMENT_SRC ${EXPERIMENT_SRC}
      object/CHtmlObject.cpp
      qtui/QCastViewHtml.cpp
      qtui/html/formcap.cpp
      )
   set(EXPERIMENT_MOC ${EXPERIMENT_MOC}
      qtui/QCastViewHtml.hpp
      qtui/html/formcap.hpp
      )
   set(EXPERIMENT_LIB ${EXPERIMENT_LIB}
      ${QT_QTWEBKIT_LIBRARY}
      )
   include_directories(
      ${QT_QTWEBKIT_INCLUDE_DIR}
      )
   add_definitions(-DV11N_OBJECT_HTML)

   if (BUILD_SA_V11N_USE_WEBKIT_PLUGINS)
      set(EXPERIMENT_SRC ${EXPERIMENT_SRC}
         qtui/QCastPlugins.cpp
         )
      set(EXPERIMENT_MOC ${EXPERIMENT_MOC}
         qtui/QCastPlugins.hpp
         )
      add_definitions(-DV11N_OBJECT_HTML_PLUGINS)
   endif (BUILD_SA_V11N_USE_WEBKIT_PLUGINS)
endif(BUILD_SA_V11N_USE_WEBKIT)


option(BUILD_SA_V11N_USE_TOMGINE "Build with support for TomGine models (requires vision.sa)" OFF)
if(BUILD_SA_V11N_USE_TOMGINE)
   set(TOMGINE_SRC
      object/CTomGineModel.cpp
      )
   set(TOMGINE_INCLUDE
      ${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/VirtualScene/TomGine/include
      )
   set(TOMGINE_LIBS
      TomGine GL
      ftgl # requires package libftgl-dev
      )
   add_definitions(-DV11N_OBJECT_TOMGINE_MODEL)
endif(BUILD_SA_V11N_USE_TOMGINE)

option(BUILD_SA_V11N_DEBUG_TRACE "Output a lot of debugging information" OFF)
if(BUILD_SA_V11N_DEBUG_TRACE)
   add_definitions(-DDEBUG_TRACE)
endif(BUILD_SA_V11N_DEBUG_TRACE)


set(SERVER_CPP
   ptrvector.cpp
   convenience.cpp
   CDisplayServer.cpp
   Model.cpp
   GuiElements.cpp
   HtmlElements.cpp
   object/CSvgImage.cpp
   object/CRasterImage.cpp
   ${EXPERIMENT_SRC}
   ${TOMGINE_SRC}
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
set(SERVER_RESOURCE_FILES
   qtui/design/jquery.qrc
   )
qt4_wrap_cpp (SERVER_MOC_OUTFILES ${SERVER_MOC_HEADERS} ${EXPERIMENT_MOC})
qt4_wrap_ui  (SERVER_UIC_OUTFILES ${SERVER_UI_FILES})
qt4_add_resources (SERVER_RESOURCE_OUTFILES ${SERVER_RESOURCE_FILES})
include_directories( .  #./object
   ${VisualizationCore_BINARY_DIR}
   ${QT_QTSVG_INCLUDE_DIR}
   ${QT_QTOPENGL_INCLUDE_DIR}
   /usr/include/GL
   /usr/include/freetype2
   ${TOMGINE_INCLUDE}
   )

add_cast_component(DisplayServer
   ${SERVER_CPP} ${GUI_CPP}
   ${SERVER_MOC_OUTFILES}
   ${SERVER_UIC_OUTFILES}
   ${SERVER_RESOURCE_OUTFILES}
   )
link_cast_component(${CAST_COMPONENT_NAME}
   VisualizationData Math Video VideoUtils
   # CogxIceAutogen
   ${QT_LIBRARIES}
   ${QT_QTSVG_LIBRARY}
   ${QT_QTOPENGL_LIBRARY} GL GLU
   ${TOMGINE_LIBS}
   boost_serialization-mt
   boost_iostreams-mt
   ${EXPERIMENT_LIB}
   )

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

add_cast_component(DisplayClient CDisplayClient.cpp CFormValues.cpp)
link_cast_component(${CAST_COMPONENT_NAME}
   VisualizationData Math Video VideoUtils
   # CogxIceAutogen
   ${OPENCV_LDFLAGS}
   )

option(BUILD_SA_V11N_TESTS "Build the test component for V11N" OFF)
if(BUILD_SA_V11N_TESTS)
   add_subdirectory(test)
endif(BUILD_SA_V11N_TESTS)


