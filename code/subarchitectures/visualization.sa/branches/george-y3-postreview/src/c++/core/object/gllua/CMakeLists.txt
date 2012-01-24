# This file is NOT supposed to be included in CogX build
# It creates a library that can be imported by Lua.
# It generates the files glbind.cpp and glubind.cpp
# that are used in visualization.sa.

cmake_minimum_required(VERSION 2.8)
find_package(Lua51)

if (LUA51_FOUND)
   find_program(CMD_TOLUA NAMES tolua++5.1 tolua++)
   set(LUA_BIND_GL_CPP ${CMAKE_CURRENT_BINARY_DIR}/glbind.cpp)
   set(LUA_BIND_GLU_CPP ${CMAKE_CURRENT_BINARY_DIR}/glubind.cpp)
   set(LUA_BIND_GL_H ${CMAKE_CURRENT_BINARY_DIR}/glbind.h)
   set(LUA_BIND_GLU_H ${CMAKE_CURRENT_BINARY_DIR}/glubind.h)
   set(LUA_BIND_V11N_CPP ${CMAKE_CURRENT_BINARY_DIR}/v11nbind.cpp)
   set(LUA_BIND_V11N_H ${CMAKE_CURRENT_BINARY_DIR}/v11nbind.h)

   include_directories(.
      ${LUA_INCLUDE_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}
      )

   add_custom_command(OUTPUT ${LUA_BIND_GL_CPP}
      COMMAND ${CMD_TOLUA} -n gl -o ${LUA_BIND_GL_CPP} -H ${LUA_BIND_GL_H} ${CMAKE_CURRENT_SOURCE_DIR}/gl.pkg
      COMMAND 
         sed -e \"s/\\\( *unsigned int\\* [a-zA-Z]* = Mtolua_new_dim\(\\\)int/\\1unsigned int/\" -i ${LUA_BIND_GL_CPP}
      COMMAND 
         sed -e \"s/\\\( *unsigned char\\* [a-zA-Z]* = Mtolua_new_dim\(\\\)char/\\1unsigned char/\" -i ${LUA_BIND_GL_CPP}
      DEPENDS gl.pkg
      )
   add_custom_command(OUTPUT ${LUA_BIND_GLU_CPP}
      COMMAND ${CMD_TOLUA} -n glu -o ${LUA_BIND_GLU_CPP} -H ${LUA_BIND_GLU_H} ${CMAKE_CURRENT_SOURCE_DIR}/glu.pkg
      DEPENDS glu.pkg
      )
   add_custom_command(OUTPUT ${LUA_BIND_V11N_CPP}
     COMMAND ${CMD_TOLUA} -n v11n -o ${LUA_BIND_V11N_CPP} -H ${LUA_BIND_V11N_H} ${CMAKE_CURRENT_SOURCE_DIR}/v11n.pkg
     DEPENDS v11n.pkg
     )
   add_library(gllua SHARED
      ${LUA_BIND_GLU_CPP}
      ${LUA_BIND_GL_CPP} 
      ${LUA_BIND_V11N_CPP} 
      gllbuffer.cpp gluaux.cpp
      )
   target_link_libraries(gllua
      GL GLU
      ${LUA_LIBRARIES}
      tolua++5.1
      )
else(LUA51_FOUND)
   message("LUA51 not found")
endif(LUA51_FOUND)
