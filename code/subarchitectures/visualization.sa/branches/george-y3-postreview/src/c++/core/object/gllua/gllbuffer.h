/*
** Auxiliary functions to support OpenGL-Lua binding.
** It implements functions to manipulate a C buffer.
** Waldemar Celes
** TeCGraf/PUC-Rio
** Ago 1998
*/

#ifndef gllbuffer_h
#define gllbuffer_h

typedef unsigned char GLLbuffer;

/* exported functions */
GLLbuffer* gllCreateBuffer1 (GLsizei n,GLenum type,lua_Number* values);
GLLbuffer* gllCreateBuffer2 (GLsizei n,GLenum type);
void gllDeleteBuffer (GLLbuffer* b);
void gllSetBuffer (GLLbuffer* b, int i, lua_Number v);
lua_Number gllGetBuffer (GLLbuffer* b, int i);

#endif
