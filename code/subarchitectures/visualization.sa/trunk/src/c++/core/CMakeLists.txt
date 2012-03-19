
project(VisualizationCore)

#set(VERBOSE 1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

option(BUILD_SA_V11N_SERVER "Build the visualisation display server" YES)
if(BUILD_SA_V11N_SERVER)

find_package(Qt4 REQUIRED)  # http://qtnode.net/wiki/Qt4_with_cmake
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR})
include_directories(${QT_QTOPENGL_INCLUDE_DIR})

#nah: mac-specific hack as cmake appears to be generating the wrong include path for QtOpenGL
include_directories(/Library/Frameworks/QtOpenGL.framework/Headers)


#nah: need to find library paths for OpenGL on OSX as it's different from linux
find_package(OpenGL REQUIRED)  

set(Boost_USE_MULTITHREADED ON)
find_package(Boost 1.36.0 REQUIRED COMPONENTS serialization iostreams )


option(BUILD_SA_V11N_USE_LUAGL "Build with LuaGl script support (requres lua51 and tolua++51)" ON)
# This block is actually for glLuabindings, copied from doris.sourceforge.net.
# TODO: It should be replaced by LuaGL, which is also included below;
# clients will have to be changed (function names: glX -> gl.X)
if(BUILD_SA_V11N_USE_LUAGL)
   find_package(Lua51)
   find_library(LUA_LIB_TOLUA_51 tolua++5.1)
   if (LUA51_FOUND AND LUA_LIB_TOLUA_51)
      set(QT_USE_OPENGL 1)
      include_directories(object/gllua ${LUA_INCLUDE_DIR})
      set(LUAGL_EXPERIMENT_SRC
         object/gllua/glbind.cpp
         object/gllua/glubind.cpp
         object/gllua/gllbuffer.cpp
         object/gllua/gluaux.cpp
         object/gllua/v11n.cpp
         object/gllua/v11nbind.cpp
         object/CLuaGlScript.cpp
         )
      set(LUAGL_EXPERIMENT_LIB
	 ${QT_QTOPENGL_LIBRARY} 
         ${OPENGL_LIBRARIES}
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

# This block is for luagl.sourceforge.net
if(BUILD_SA_V11N_USE_LUAGL)
   include_directories(object/luagl ${LUA_INCLUDE_DIR})
   set(LUAGL_EXPERIMENT_SRC ${LUAGL_EXPERIMENT_SRC}
      object/luagl/luagl.c
      object/luagl/luaglu.c
      object/luagl/luagl_util.c
      )
endif(BUILD_SA_V11N_USE_LUAGL)

option(BUILD_SA_V11N_USE_OGRE3D "Build with OGRE 3D support (requires OGRE 1.7.3)" OFF)
if(BUILD_SA_V11N_USE_OGRE3D)
   set(QT_USE_OPENGL 1)
   message("CMAKE_MODULE_PATH:" ${CMAKE_MODULE_PATH})
   # CMake didn't find /usr/share/OGRE/cmake/modules for 1.7.3 ... '/modules' is redundant!
   set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
      /usr/share/OGRE/cmake/modules
      /usr/local/share/OGRE/cmake/modules
      # XXX THIS IS A HACK! The path is system-dependent.
      )
   find_package(OGRE)
   include_directories(${OGRE_INCLUDE_DIRS})
   set(OGRE3D_SRC
      qtui/ogre/OgreBase.cpp
      qtui/QCastViewOgre.cpp
      )
   set(OGRE3D_MOC
      qtui/QCastViewOgre.hpp
      )
   set(OGRE3D_LIB
      ${OGRE_LIBRARIES}
      )
endif(BUILD_SA_V11N_USE_OGRE3D)

option(BUILD_SA_V11N_USE_WEBKIT "Build with HTML support (Qt-WebKit)" ON)
option(BUILD_SA_V11N_USE_WEBKIT_PLUGINS "Enable plugins in HTML pages (Qt-WebKit)" ON)
if(BUILD_SA_V11N_USE_WEBKIT)
   set(WEBKIT_EXPERIMENT_SRC
      object/CHtmlObject.cpp
      qtui/QCastViewHtml.cpp
      qtui/html/formcap.cpp
      )
   set(WEBKIT_EXPERIMENT_MOC
      qtui/QCastViewHtml.hpp
      qtui/html/formcap.hpp
      )
   set(WEBKIT_EXPERIMENT_LIB
      ${QT_QTWEBKIT_LIBRARY}
      )
   include_directories(
      ${QT_QTWEBKIT_INCLUDE_DIR}
      )
   add_definitions(-DV11N_OBJECT_HTML)

   if (BUILD_SA_V11N_USE_WEBKIT_PLUGINS)
      set(WEBKIT_EXPERIMENT_SRC ${WEBKIT_EXPERIMENT_SRC}
         qtui/QCastPlugins.cpp
         )
      set(WEBKIT_EXPERIMENT_MOC ${WEBKIT_EXPERIMENT_MOC}
         qtui/QCastPlugins.hpp
         )
      add_definitions(-DV11N_OBJECT_HTML_PLUGINS)
   endif (BUILD_SA_V11N_USE_WEBKIT_PLUGINS)
endif(BUILD_SA_V11N_USE_WEBKIT)


option(BUILD_SA_V11N_USE_TOMGINE "Build with support for TomGine models (requires vision.sa)" OFF)
if(BUILD_SA_V11N_USE_TOMGINE)
   set(QT_USE_OPENGL 1)
   set(TOMGINE_SRC
      object/CTomGineModel.cpp
      )
   set(TOMGINE_INCLUDE
      ${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/components/VirtualScene/TomGine/include
      )
   set(TOMGINE_LIBS
      TomGine 
      ftgl # requires package libftgl-dev
      ${QT_QTOPENGL_LIBRARY} 
      ${OPENGL_LIBRARIES}
      ${Boost_LIBRARIES}
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
   View.cpp
   GuiElements.cpp
   HtmlElements.cpp
   object/CSvgImage.cpp
   object/CRasterImage.cpp
   ${LUAGL_EXPERIMENT_SRC}
   ${WEBKIT_EXPERIMENT_SRC}
   ${TOMGINE_SRC}
   ${OGRE3D_SRC}
   )
set(GUI_CPP
   qtui/QCastApplication.cpp
   qtui/QCastMainFrame.cpp
   qtui/QCastDialogFrame.cpp
   qtui/QCastView.cpp
   qtui/QCastViewScene.cpp
   qtui/QCustomGuiPanel.cpp
   qtui/QCustomToolBar.cpp
   qtui/QViewContainer.cpp
   qtui/ChangeSlot.cpp
   )
set(SERVER_MOC_HEADERS
   qtui/QCastApplication.hpp
   qtui/QCastMainFrame.hpp
   qtui/QCastDialogFrame.hpp
   qtui/QCastView.hpp
   qtui/QCastViewScene.hpp
   qtui/ChangeSlot.hpp
   qtui/QCustomGuiPanel.hpp
   qtui/QCustomToolBar.hpp
   qtui/QViewContainer.hpp
   ${WEBKIT_EXPERIMENT_MOC}
   ${OGRE3D_MOC}
   )
set(SERVER_UI_FILES
   qtui/design/castor.ui
   qtui/design/dialogwin.ui
   )
set(SERVER_RESOURCE_FILES
   qtui/design/jquery.qrc
   qtui/design/images.qrc
   )
if(QT_USE_OPENGL)
   add_definitions(-DV11N_VIEW_GL)
   set(GUI_CPP ${GUI_CPP}
      qtui/QCastViewGL.cpp
      )
   set(SERVER_MOC_HEADERS ${SERVER_MOC_HEADERS}
      qtui/QCastViewGL.hpp
      )
endif(QT_USE_OPENGL)

qt4_wrap_cpp (SERVER_MOC_OUTFILES ${SERVER_MOC_HEADERS})
qt4_wrap_ui  (SERVER_UIC_OUTFILES ${SERVER_UI_FILES})
qt4_add_resources (SERVER_RESOURCE_OUTFILES ${SERVER_RESOURCE_FILES})
include_directories( .  #./object
   ${VisualizationCore_BINARY_DIR}
   ${QT_QTSVG_INCLUDE_DIR}
   ${QT_QTSCRIPT_INCLUDE_DIR}
   ${QT_QTSCRIPTTOOLS_INCLUDE_DIR}
   ${QT_QTUITOOLS_INCLUDE_DIR}
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
   VisualizationData 
   # CogxIceAutogen   
   ${QT_LIBRARIES}
   ${QT_QTSVG_LIBRARY}
   ${QT_QTSCRIPT_LIBRARY}
   ${QT_QTSCRIPTTOOLS_LIBRARY}
   ${QT_QTUITOOLS_LIBRARY}
   ${TOMGINE_LIBS}
   ${LUAGL_EXPERIMENT_LIB} 
   ${WEBKIT_EXPERIMENT_LIB} 
   ${OGRE3D_LIB}
   )

endif(BUILD_SA_V11N_SERVER)

option(BUILD_SA_V11N_CLIENT "Build the visualisation display client library" YES)
if(BUILD_SA_V11N_CLIENT)
   find_package(OpenCV REQUIRED)
   include_directories(${OPENCV_INCLUDE_DIRS})
   include_directories(client)
   include_directories(${VISUALIZATION_INCLUDE_DIRS})
   link_directories(${OPENCV_LIBRARY_DIRS})
   add_definitions(-DFEAT_VISUALIZATION_OPENCV)

   add_cast_component(DisplayClient
      client/CDisplayClient.cpp
      client/CFormValues.cpp
      client/CMilliTimer.cpp
      )
   link_cast_component(${CAST_COMPONENT_NAME}
      ${VISUALIZATION_LIBRARIES}
      ${OPENCV_LDFLAGS}
      )
   if (HAS_COGX_MATH)
      link_cast_component(${CAST_COMPONENT_NAME} Math)
   endif()
   if (HAS_COGX_VIDEO)
      link_cast_component(${CAST_COMPONENT_NAME} Video VideoUtils)
   endif()
endif(BUILD_SA_V11N_CLIENT)

option(BUILD_SA_V11N_TESTS "Build the test component for V11N (requires the client library)" OFF)
if(BUILD_SA_V11N_TESTS)
   add_subdirectory(test)
endif(BUILD_SA_V11N_TESTS)


option(BUILD_SA_V11N_STANDALONE "Build the standalone display server for V11N" ON)

if(BUILD_SA_V11N_STANDALONE)
  set(sources StandaloneDisplayServer.cpp)

  add_executable (display-server ${sources}) 

  target_link_libraries(display-server DisplayServer)

  IF (APPLE)
     FIND_LIBRARY(COREFOUNDATION_LIBRARY CoreFoundation )
     target_link_libraries(display-server ${COREFOUNDATION_LIBRARY})
  ENDIF (APPLE)

install(TARGETS display-server RUNTIME DESTINATION bin)

endif(BUILD_SA_V11N_STANDALONE)
