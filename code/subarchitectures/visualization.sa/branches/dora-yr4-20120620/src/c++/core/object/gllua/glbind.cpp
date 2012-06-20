/*
** Lua binding: gl
** Generated automatically by tolua++-1.0.93 on Wed Jul 20 12:46:09 2011.
*/

#ifndef __cplusplus
#include "stdlib.h"
#endif
#include "string.h"

#include "tolua++.h"

/* Exported function */
TOLUA_API int  tolua_gl_open (lua_State* tolua_S);

#ifdef WIN32
#include <windows.h>
#pragma warning (disable:4244)  /* double to int warning */
#endif
#include <GL/gl.h>
#include "gllbuffer.h"

/* function to register type */
static void tolua_reg_types (lua_State* tolua_S)
{
 tolua_usertype(tolua_S,"GLLbuffer");
 tolua_usertype(tolua_S,"GLbitfield");
}

#ifdef GL_VERSION_1_1

#endif

/* function: glClearIndex */
#ifndef TOLUA_DISABLE_tolua_gl_glClearIndex00
static int tolua_gl_glClearIndex00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   float c = ((  float)  tolua_tonumber(tolua_S,1,0));
  {
   glClearIndex(c);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glClearIndex'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glClearColor */
#ifndef TOLUA_DISABLE_tolua_gl_glClearColor00
static int tolua_gl_glClearColor00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   float red = ((  float)  tolua_tonumber(tolua_S,1,0));
   float green = ((  float)  tolua_tonumber(tolua_S,2,0));
   float blue = ((  float)  tolua_tonumber(tolua_S,3,0));
   float alpha = ((  float)  tolua_tonumber(tolua_S,4,0));
  {
   glClearColor(red,green,blue,alpha);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glClearColor'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glClear */
#ifndef TOLUA_DISABLE_tolua_gl_glClear00
static int tolua_gl_glClear00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double mask = ((  double)  tolua_tonumber(tolua_S,1,0));
  {
   glClear(mask);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glClear'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glIndexMask */
#ifndef TOLUA_DISABLE_tolua_gl_glIndexMask00
static int tolua_gl_glIndexMask00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned int mask = (( unsigned int)  tolua_tonumber(tolua_S,1,0));
  {
   glIndexMask(mask);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glIndexMask'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glColorMask */
#ifndef TOLUA_DISABLE_tolua_gl_glColorMask00
static int tolua_gl_glColorMask00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned char red = (( unsigned char)  tolua_tonumber(tolua_S,1,0));
  unsigned char green = (( unsigned char)  tolua_tonumber(tolua_S,2,0));
  unsigned char blue = (( unsigned char)  tolua_tonumber(tolua_S,3,0));
  unsigned char alpha = (( unsigned char)  tolua_tonumber(tolua_S,4,0));
  {
   glColorMask(red,green,blue,alpha);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glColorMask'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glAlphaFunc */
#ifndef TOLUA_DISABLE_tolua_gl_glAlphaFunc00
static int tolua_gl_glAlphaFunc00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double func = ((   double)  tolua_tonumber(tolua_S,1,0));
   float ref = ((  float)  tolua_tonumber(tolua_S,2,0));
  {
   glAlphaFunc(func,ref);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glAlphaFunc'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glBlendFunc */
#ifndef TOLUA_DISABLE_tolua_gl_glBlendFunc00
static int tolua_gl_glBlendFunc00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double sfactor = ((   double)  tolua_tonumber(tolua_S,1,0));
    double dfactor = ((   double)  tolua_tonumber(tolua_S,2,0));
  {
   glBlendFunc(sfactor,dfactor);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glBlendFunc'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glLogicOp */
#ifndef TOLUA_DISABLE_tolua_gl_glLogicOp00
static int tolua_gl_glLogicOp00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double opcode = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   glLogicOp(opcode);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glLogicOp'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glCullFace */
#ifndef TOLUA_DISABLE_tolua_gl_glCullFace00
static int tolua_gl_glCullFace00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double mode = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   glCullFace(mode);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glCullFace'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glFrontFace */
#ifndef TOLUA_DISABLE_tolua_gl_glFrontFace00
static int tolua_gl_glFrontFace00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double mode = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   glFrontFace(mode);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glFrontFace'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPointSize */
#ifndef TOLUA_DISABLE_tolua_gl_glPointSize00
static int tolua_gl_glPointSize00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   float size = ((  float)  tolua_tonumber(tolua_S,1,0));
  {
   glPointSize(size);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPointSize'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glLineWidth */
#ifndef TOLUA_DISABLE_tolua_gl_glLineWidth00
static int tolua_gl_glLineWidth00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   float width = ((  float)  tolua_tonumber(tolua_S,1,0));
  {
   glLineWidth(width);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glLineWidth'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glLineStipple */
#ifndef TOLUA_DISABLE_tolua_gl_glLineStipple00
static int tolua_gl_glLineStipple00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int factor = ((  int)  tolua_tonumber(tolua_S,1,0));
  unsigned short pattern = (( unsigned short)  tolua_tonumber(tolua_S,2,0));
  {
   glLineStipple(factor,pattern);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glLineStipple'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPolygonMode */
#ifndef TOLUA_DISABLE_tolua_gl_glPolygonMode00
static int tolua_gl_glPolygonMode00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double face = ((   double)  tolua_tonumber(tolua_S,1,0));
    double mode = ((   double)  tolua_tonumber(tolua_S,2,0));
  {
   glPolygonMode(face,mode);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPolygonMode'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPolygonStipple */
#ifndef TOLUA_DISABLE_tolua_gl_glPolygonStipple00
static int tolua_gl_glPolygonStipple00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isstring(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned char* mask = ((unsigned char*)  tolua_tostring(tolua_S,1,0));
  {
   glPolygonStipple(mask);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPolygonStipple'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetPolygonStipple */
#ifndef TOLUA_DISABLE_tolua_gl_glGetPolygonStipple00
static int tolua_gl_glGetPolygonStipple00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isstring(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned char* mask = ((unsigned char*)  tolua_tostring(tolua_S,1,0));
  {
   glGetPolygonStipple(mask);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGetPolygonStipple'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEdgeFlag */
#ifndef TOLUA_DISABLE_tolua_gl_glEdgeFlag00
static int tolua_gl_glEdgeFlag00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned char flag = (( unsigned char)  tolua_tonumber(tolua_S,1,0));
  {
   glEdgeFlag(flag);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glEdgeFlag'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glScissor */
#ifndef TOLUA_DISABLE_tolua_gl_glScissor00
static int tolua_gl_glScissor00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int x = ((  int)  tolua_tonumber(tolua_S,1,0));
   int y = ((  int)  tolua_tonumber(tolua_S,2,0));
   int width = ((  int)  tolua_tonumber(tolua_S,3,0));
   int height = ((  int)  tolua_tonumber(tolua_S,4,0));
  {
   glScissor(x,y,width,height);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glScissor'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glClipPlane */
#ifndef TOLUA_DISABLE_tolua_gl_glClipPlane00
static int tolua_gl_glClipPlane00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_istable(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double plane = ((   double)  tolua_tonumber(tolua_S,1,0));
   double equation[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,2,4,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    equation[i] = ((double)  tolua_tofieldnumber(tolua_S,2,i+1,0));
   }
  }
  {
   glClipPlane(plane,equation);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,2,i+1,(lua_Number) equation[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glClipPlane'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetClipPlane */
#ifndef TOLUA_DISABLE_tolua_gl_glGetClipPlane00
static int tolua_gl_glGetClipPlane00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_istable(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double plane = ((   double)  tolua_tonumber(tolua_S,1,0));
   double equation[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,2,4,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    equation[i] = ((double)  tolua_tofieldnumber(tolua_S,2,i+1,0));
   }
  }
  {
   glGetClipPlane(plane,equation);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,2,i+1,(lua_Number) equation[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGetClipPlane'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glDrawBuffer */
#ifndef TOLUA_DISABLE_tolua_gl_glDrawBuffer00
static int tolua_gl_glDrawBuffer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double mode = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   glDrawBuffer(mode);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glDrawBuffer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glReadBuffer */
#ifndef TOLUA_DISABLE_tolua_gl_glReadBuffer00
static int tolua_gl_glReadBuffer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double mode = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   glReadBuffer(mode);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glReadBuffer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEnable */
#ifndef TOLUA_DISABLE_tolua_gl_glEnable00
static int tolua_gl_glEnable00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double cap = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   glEnable(cap);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glEnable'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glDisable */
#ifndef TOLUA_DISABLE_tolua_gl_glDisable00
static int tolua_gl_glDisable00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double cap = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   glDisable(cap);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glDisable'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glIsEnabled */
#ifndef TOLUA_DISABLE_tolua_gl_glIsEnabled00
static int tolua_gl_glIsEnabled00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double cap = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   unsigned char tolua_ret = ( unsigned char)  glIsEnabled(cap);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glIsEnabled'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetDoublev */
#ifndef TOLUA_DISABLE_tolua_gl_glGet00
static int tolua_gl_glGet00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,1,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double pname = ((   double)  tolua_tonumber(tolua_S,1,0));
   double params = ((  double)  tolua_tonumber(tolua_S,2,0));
  {
   glGetDoublev(pname,&params);
   tolua_pushnumber(tolua_S,(lua_Number)params);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGet'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetDoublev */
#ifndef TOLUA_DISABLE_tolua_gl_glGet01
static int tolua_gl_glGet01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_istable(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
    double pname = ((   double)  tolua_tonumber(tolua_S,1,0));
   double params[16];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,2,16,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<16;i++)
    params[i] = ((double)  tolua_tofieldnumber(tolua_S,2,i+1,0));
   }
  }
  {
   glGetDoublev(pname,params);
  }
  {
   int i;
   for(i=0; i<16;i++)
    tolua_pushfieldnumber(tolua_S,2,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glGet00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPushAttrib */
#ifndef TOLUA_DISABLE_tolua_gl_glPushAttrib00
static int tolua_gl_glPushAttrib00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     (tolua_isvaluenil(tolua_S,1,&tolua_err) || !tolua_isusertype(tolua_S,1,"GLbitfield",0,&tolua_err)) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLbitfield mask = *((GLbitfield*)  tolua_tousertype(tolua_S,1,0));
  {
   glPushAttrib(mask);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPushAttrib'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPopAttrib */
#ifndef TOLUA_DISABLE_tolua_gl_glPopAttrib00
static int tolua_gl_glPopAttrib00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnoobj(tolua_S,1,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   glPopAttrib();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPopAttrib'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glRenderMode */
#ifndef TOLUA_DISABLE_tolua_gl_glRenderMode00
static int tolua_gl_glRenderMode00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double mode = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
    int tolua_ret = (  int)  glRenderMode(mode);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glRenderMode'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetError */
#ifndef TOLUA_DISABLE_tolua_gl_glGetError00
static int tolua_gl_glGetError00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnoobj(tolua_S,1,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
     double tolua_ret = (   double)  glGetError();
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGetError'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetString */
#ifndef TOLUA_DISABLE_tolua_gl_glGetString00
static int tolua_gl_glGetString00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double name = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   unsigned const char* tolua_ret = ( unsigned const char*)  glGetString(name);
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGetString'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glFinish */
#ifndef TOLUA_DISABLE_tolua_gl_glFinish00
static int tolua_gl_glFinish00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnoobj(tolua_S,1,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   glFinish();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glFinish'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glFlush */
#ifndef TOLUA_DISABLE_tolua_gl_glFlush00
static int tolua_gl_glFlush00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnoobj(tolua_S,1,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   glFlush();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glFlush'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glHint */
#ifndef TOLUA_DISABLE_tolua_gl_glHint00
static int tolua_gl_glHint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
    double mode = ((   double)  tolua_tonumber(tolua_S,2,0));
  {
   glHint(target,mode);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glHint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glClearDepth */
#ifndef TOLUA_DISABLE_tolua_gl_glClearDepth00
static int tolua_gl_glClearDepth00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double depth = ((  double)  tolua_tonumber(tolua_S,1,0));
  {
   glClearDepth(depth);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glClearDepth'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glDepthFunc */
#ifndef TOLUA_DISABLE_tolua_gl_glDepthFunc00
static int tolua_gl_glDepthFunc00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double func = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   glDepthFunc(func);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glDepthFunc'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glDepthMask */
#ifndef TOLUA_DISABLE_tolua_gl_glDepthMask00
static int tolua_gl_glDepthMask00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned char flag = (( unsigned char)  tolua_tonumber(tolua_S,1,0));
  {
   glDepthMask(flag);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glDepthMask'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glDepthRange */
#ifndef TOLUA_DISABLE_tolua_gl_glDepthRange00
static int tolua_gl_glDepthRange00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double near_val = ((  double)  tolua_tonumber(tolua_S,1,0));
   double far_val = ((  double)  tolua_tonumber(tolua_S,2,0));
  {
   glDepthRange(near_val,far_val);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glDepthRange'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glClearAccum */
#ifndef TOLUA_DISABLE_tolua_gl_glClearAccum00
static int tolua_gl_glClearAccum00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   float red = ((  float)  tolua_tonumber(tolua_S,1,0));
   float green = ((  float)  tolua_tonumber(tolua_S,2,0));
   float blue = ((  float)  tolua_tonumber(tolua_S,3,0));
   float alpha = ((  float)  tolua_tonumber(tolua_S,4,0));
  {
   glClearAccum(red,green,blue,alpha);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glClearAccum'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glAccum */
#ifndef TOLUA_DISABLE_tolua_gl_glAccum00
static int tolua_gl_glAccum00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double op = ((   double)  tolua_tonumber(tolua_S,1,0));
   float value = ((  float)  tolua_tonumber(tolua_S,2,0));
  {
   glAccum(op,value);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glAccum'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glMatrixMode */
#ifndef TOLUA_DISABLE_tolua_gl_glMatrixMode00
static int tolua_gl_glMatrixMode00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double mode = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   glMatrixMode(mode);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glMatrixMode'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glOrtho */
#ifndef TOLUA_DISABLE_tolua_gl_glOrtho00
static int tolua_gl_glOrtho00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,7,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double left = ((  double)  tolua_tonumber(tolua_S,1,0));
   double right = ((  double)  tolua_tonumber(tolua_S,2,0));
   double bottom = ((  double)  tolua_tonumber(tolua_S,3,0));
   double top = ((  double)  tolua_tonumber(tolua_S,4,0));
   double near_val = ((  double)  tolua_tonumber(tolua_S,5,0));
   double far_val = ((  double)  tolua_tonumber(tolua_S,6,0));
  {
   glOrtho(left,right,bottom,top,near_val,far_val);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glOrtho'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glFrustum */
#ifndef TOLUA_DISABLE_tolua_gl_glFrustum00
static int tolua_gl_glFrustum00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,7,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double left = ((  double)  tolua_tonumber(tolua_S,1,0));
   double right = ((  double)  tolua_tonumber(tolua_S,2,0));
   double bottom = ((  double)  tolua_tonumber(tolua_S,3,0));
   double top = ((  double)  tolua_tonumber(tolua_S,4,0));
   double near_val = ((  double)  tolua_tonumber(tolua_S,5,0));
   double far_val = ((  double)  tolua_tonumber(tolua_S,6,0));
  {
   glFrustum(left,right,bottom,top,near_val,far_val);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glFrustum'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glViewport */
#ifndef TOLUA_DISABLE_tolua_gl_glViewport00
static int tolua_gl_glViewport00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int x = ((  int)  tolua_tonumber(tolua_S,1,0));
   int y = ((  int)  tolua_tonumber(tolua_S,2,0));
   int width = ((  int)  tolua_tonumber(tolua_S,3,0));
   int height = ((  int)  tolua_tonumber(tolua_S,4,0));
  {
   glViewport(x,y,width,height);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glViewport'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPushMatrix */
#ifndef TOLUA_DISABLE_tolua_gl_glPushMatrix00
static int tolua_gl_glPushMatrix00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnoobj(tolua_S,1,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   glPushMatrix();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPushMatrix'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPopMatrix */
#ifndef TOLUA_DISABLE_tolua_gl_glPopMatrix00
static int tolua_gl_glPopMatrix00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnoobj(tolua_S,1,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   glPopMatrix();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPopMatrix'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glLoadIdentity */
#ifndef TOLUA_DISABLE_tolua_gl_glLoadIdentity00
static int tolua_gl_glLoadIdentity00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnoobj(tolua_S,1,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   glLoadIdentity();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glLoadIdentity'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glLoadMatrixd */
#ifndef TOLUA_DISABLE_tolua_gl_glLoadMatrix00
static int tolua_gl_glLoadMatrix00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double m[16];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,16,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<16;i++)
    m[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,0));
   }
  }
  {
   glLoadMatrixd(m);
  }
  {
   int i;
   for(i=0; i<16;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) m[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glLoadMatrix'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glMultMatrixd */
#ifndef TOLUA_DISABLE_tolua_gl_glMultMatrix00
static int tolua_gl_glMultMatrix00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double m[16];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,16,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<16;i++)
    m[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,0));
   }
  }
  {
   glMultMatrixd(m);
  }
  {
   int i;
   for(i=0; i<16;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) m[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glMultMatrix'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glRotated */
#ifndef TOLUA_DISABLE_tolua_gl_glRotate00
static int tolua_gl_glRotate00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double angle = ((  double)  tolua_tonumber(tolua_S,1,0));
   double x = ((  double)  tolua_tonumber(tolua_S,2,0));
   double y = ((  double)  tolua_tonumber(tolua_S,3,0));
   double z = ((  double)  tolua_tonumber(tolua_S,4,0));
  {
   glRotated(angle,x,y,z);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glRotate'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glScaled */
#ifndef TOLUA_DISABLE_tolua_gl_glScale00
static int tolua_gl_glScale00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,1,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double x = ((  double)  tolua_tonumber(tolua_S,1,0));
   double y = ((  double)  tolua_tonumber(tolua_S,2,0));
   double z = ((  double)  tolua_tonumber(tolua_S,3,1.0));
  {
   glScaled(x,y,z);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glScale'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTranslated */
#ifndef TOLUA_DISABLE_tolua_gl_glTranslate00
static int tolua_gl_glTranslate00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,1,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double x = ((  double)  tolua_tonumber(tolua_S,1,0));
   double y = ((  double)  tolua_tonumber(tolua_S,2,0));
   double z = ((  double)  tolua_tonumber(tolua_S,3,0.0));
  {
   glTranslated(x,y,z);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glTranslate'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glIsList */
#ifndef TOLUA_DISABLE_tolua_gl_glIsList00
static int tolua_gl_glIsList00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned int list = (( unsigned int)  tolua_tonumber(tolua_S,1,0));
  {
   unsigned char tolua_ret = ( unsigned char)  glIsList(list);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glIsList'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glDeleteLists */
#ifndef TOLUA_DISABLE_tolua_gl_glDeleteLists00
static int tolua_gl_glDeleteLists00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned int list = (( unsigned int)  tolua_tonumber(tolua_S,1,0));
   int range = ((  int)  tolua_tonumber(tolua_S,2,0));
  {
   glDeleteLists(list,range);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glDeleteLists'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGenLists */
#ifndef TOLUA_DISABLE_tolua_gl_glGenLists00
static int tolua_gl_glGenLists00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int range = ((  int)  tolua_tonumber(tolua_S,1,0));
  {
   unsigned int tolua_ret = ( unsigned int)  glGenLists(range);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGenLists'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glNewList */
#ifndef TOLUA_DISABLE_tolua_gl_glNewList00
static int tolua_gl_glNewList00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned int list = (( unsigned int)  tolua_tonumber(tolua_S,1,0));
    double mode = ((   double)  tolua_tonumber(tolua_S,2,0));
  {
   glNewList(list,mode);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glNewList'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEndList */
#ifndef TOLUA_DISABLE_tolua_gl_glEndList00
static int tolua_gl_glEndList00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnoobj(tolua_S,1,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   glEndList();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glEndList'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glCallList */
#ifndef TOLUA_DISABLE_tolua_gl_glCallList00
static int tolua_gl_glCallList00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned int list = (( unsigned int)  tolua_tonumber(tolua_S,1,0));
  {
   glCallList(list);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glCallList'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glListBase */
#ifndef TOLUA_DISABLE_tolua_gl_glListBase00
static int tolua_gl_glListBase00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned int base = (( unsigned int)  tolua_tonumber(tolua_S,1,0));
  {
   glListBase(base);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glListBase'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glBegin */
#ifndef TOLUA_DISABLE_tolua_gl_glBegin00
static int tolua_gl_glBegin00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double mode = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   glBegin(mode);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glBegin'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEnd */
#ifndef TOLUA_DISABLE_tolua_gl_glEnd00
static int tolua_gl_glEnd00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnoobj(tolua_S,1,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   glEnd();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glEnd'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glVertex3dv */
#ifndef TOLUA_DISABLE_tolua_gl_glVertex00
static int tolua_gl_glVertex00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double v[3];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,3,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<3;i++)
    v[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,0));
   }
  }
  {
   glVertex3dv(v);
  }
  {
   int i;
   for(i=0; i<3;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) v[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glVertex'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glVertex4dv */
#ifndef TOLUA_DISABLE_tolua_gl_glVertex01
static int tolua_gl_glVertex01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   double v[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,4,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    v[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,0));
   }
  }
  {
   glVertex4dv(v);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) v[i]);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glVertex00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glVertex4d */
#ifndef TOLUA_DISABLE_tolua_gl_glVertex02
static int tolua_gl_glVertex02(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,1,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,1,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   double x = ((  double)  tolua_tonumber(tolua_S,1,0));
   double y = ((  double)  tolua_tonumber(tolua_S,2,0));
   double z = ((  double)  tolua_tonumber(tolua_S,3,0));
   double w = ((  double)  tolua_tonumber(tolua_S,4,1));
  {
   glVertex4d(x,y,z,w);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glVertex01(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glNormal3dv */
#ifndef TOLUA_DISABLE_tolua_gl_glNormal00
static int tolua_gl_glNormal00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double v[3];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,3,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<3;i++)
    v[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,0));
   }
  }
  {
   glNormal3dv(v);
  }
  {
   int i;
   for(i=0; i<3;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) v[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glNormal'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glNormal3d */
#ifndef TOLUA_DISABLE_tolua_gl_glNormal01
static int tolua_gl_glNormal01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,1,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   double nx = ((  double)  tolua_tonumber(tolua_S,1,0));
   double ny = ((  double)  tolua_tonumber(tolua_S,2,0));
   double nz = ((  double)  tolua_tonumber(tolua_S,3,0));
  {
   glNormal3d(nx,ny,nz);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glNormal00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glIndexdv */
#ifndef TOLUA_DISABLE_tolua_gl_glIndex00
static int tolua_gl_glIndex00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double c[1];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,1,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<1;i++)
    c[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,0));
   }
  }
  {
   glIndexdv(c);
  }
  {
   int i;
   for(i=0; i<1;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) c[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glIndex'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glIndexd */
#ifndef TOLUA_DISABLE_tolua_gl_glIndex01
static int tolua_gl_glIndex01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   double c = ((  double)  tolua_tonumber(tolua_S,1,0));
  {
   glIndexd(c);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glIndex00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glColor4dv */
#ifndef TOLUA_DISABLE_tolua_gl_glColor00
static int tolua_gl_glColor00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double v[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,4,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    v[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,1.0));
   }
  }
  {
   glColor4dv(v);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) v[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glColor'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glColor4d */
#ifndef TOLUA_DISABLE_tolua_gl_glColor01
static int tolua_gl_glColor01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,1,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   double red = ((  double)  tolua_tonumber(tolua_S,1,0));
   double green = ((  double)  tolua_tonumber(tolua_S,2,0));
   double blue = ((  double)  tolua_tonumber(tolua_S,3,0));
   double alpha = ((  double)  tolua_tonumber(tolua_S,4,1.0));
  {
   glColor4d(red,green,blue,alpha);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glColor00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexCoord3dv */
#ifndef TOLUA_DISABLE_tolua_gl_glTexCoord00
static int tolua_gl_glTexCoord00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double v[3];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,3,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<3;i++)
    v[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,0.0));
   }
  }
  {
   glTexCoord3dv(v);
  }
  {
   int i;
   for(i=0; i<3;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) v[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glTexCoord'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexCoord4dv */
#ifndef TOLUA_DISABLE_tolua_gl_glTexCoord01
static int tolua_gl_glTexCoord01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   double v[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,4,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    v[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,0));
   }
  }
  {
   glTexCoord4dv(v);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) v[i]);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glTexCoord00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexCoord4d */
#ifndef TOLUA_DISABLE_tolua_gl_glTexCoord02
static int tolua_gl_glTexCoord02(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,1,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,1,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,1,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   double s = ((  double)  tolua_tonumber(tolua_S,1,0));
   double t = ((  double)  tolua_tonumber(tolua_S,2,0.0));
   double r = ((  double)  tolua_tonumber(tolua_S,3,0.0));
   double q = ((  double)  tolua_tonumber(tolua_S,4,1.0));
  {
   glTexCoord4d(s,t,r,q);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glTexCoord01(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glRasterPos3dv */
#ifndef TOLUA_DISABLE_tolua_gl_glRasterPos00
static int tolua_gl_glRasterPos00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double v[3];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,3,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<3;i++)
    v[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,0.0));
   }
  }
  {
   glRasterPos3dv(v);
  }
  {
   int i;
   for(i=0; i<3;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) v[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glRasterPos'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glRasterPos4dv */
#ifndef TOLUA_DISABLE_tolua_gl_glRasterPos01
static int tolua_gl_glRasterPos01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   double v[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,4,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    v[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,0));
   }
  }
  {
   glRasterPos4dv(v);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) v[i]);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glRasterPos00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glRasterPos4d */
#ifndef TOLUA_DISABLE_tolua_gl_glRasterPos02
static int tolua_gl_glRasterPos02(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,1,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,1,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   double x = ((  double)  tolua_tonumber(tolua_S,1,0));
   double y = ((  double)  tolua_tonumber(tolua_S,2,0));
   double z = ((  double)  tolua_tonumber(tolua_S,3,0.0));
   double w = ((  double)  tolua_tonumber(tolua_S,4,1.0));
  {
   glRasterPos4d(x,y,z,w);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glRasterPos01(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glRectdv */
#ifndef TOLUA_DISABLE_tolua_gl_glRect00
static int tolua_gl_glRect00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_istable(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double v1[2];
   double v2[2];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,2,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<2;i++)
    v1[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,0));
   }
  }
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,2,2,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<2;i++)
    v2[i] = ((double)  tolua_tofieldnumber(tolua_S,2,i+1,0));
   }
  }
  {
   glRectdv(v1,v2);
  }
  {
   int i;
   for(i=0; i<2;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) v1[i]);
  }
  {
   int i;
   for(i=0; i<2;i++)
    tolua_pushfieldnumber(tolua_S,2,i+1,(lua_Number) v2[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glRect'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glRectd */
#ifndef TOLUA_DISABLE_tolua_gl_glRect01
static int tolua_gl_glRect01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   double x1 = ((  double)  tolua_tonumber(tolua_S,1,0));
   double y1 = ((  double)  tolua_tonumber(tolua_S,2,0));
   double x2 = ((  double)  tolua_tonumber(tolua_S,3,0));
   double y2 = ((  double)  tolua_tonumber(tolua_S,4,0));
  {
   glRectd(x1,y1,x2,y2);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glRect00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glShadeModel */
#ifndef TOLUA_DISABLE_tolua_gl_glShadeModel00
static int tolua_gl_glShadeModel00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double mode = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   glShadeModel(mode);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glShadeModel'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glLightfv */
#ifndef TOLUA_DISABLE_tolua_gl_glLight00
static int tolua_gl_glLight00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double light = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float params[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,4,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    params[i] = ((float)  tolua_tofieldnumber(tolua_S,3,i+1,0.0));
   }
  }
  {
   glLightfv(light,pname,params);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glLight'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glLightf */
#ifndef TOLUA_DISABLE_tolua_gl_glLight01
static int tolua_gl_glLight01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
    double light = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float param = ((  float)  tolua_tonumber(tolua_S,3,0));
  {
   glLightf(light,pname,param);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glLight00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetLightfv */
#ifndef TOLUA_DISABLE_tolua_gl_glGetLight00
static int tolua_gl_glGetLight00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double light = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float params[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,4,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    params[i] = ((float)  tolua_tofieldnumber(tolua_S,3,i+1,0.0));
   }
  }
  {
   glGetLightfv(light,pname,params);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGetLight'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetLightfv */
#ifndef TOLUA_DISABLE_tolua_gl_glGetLight01
static int tolua_gl_glGetLight01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,1,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
    double light = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float params = ((  float)  tolua_tonumber(tolua_S,3,0.0));
  {
   glGetLightfv(light,pname,&params);
   tolua_pushnumber(tolua_S,(lua_Number)params);
  }
 }
 return 1;
tolua_lerror:
 return tolua_gl_glGetLight00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glLightModelfv */
#ifndef TOLUA_DISABLE_tolua_gl_glLightModel00
static int tolua_gl_glLightModel00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_istable(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double pname = ((   double)  tolua_tonumber(tolua_S,1,0));
   float params[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,2,4,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    params[i] = ((float)  tolua_tofieldnumber(tolua_S,2,i+1,0.0));
   }
  }
  {
   glLightModelfv(pname,params);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,2,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glLightModel'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glLightModelf */
#ifndef TOLUA_DISABLE_tolua_gl_glLightModel01
static int tolua_gl_glLightModel01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
    double pname = ((   double)  tolua_tonumber(tolua_S,1,0));
   float param = ((  float)  tolua_tonumber(tolua_S,2,0));
  {
   glLightModelf(pname,param);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glLightModel00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glMaterialfv */
#ifndef TOLUA_DISABLE_tolua_gl_glMaterial00
static int tolua_gl_glMaterial00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double face = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float params[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,4,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    params[i] = ((float)  tolua_tofieldnumber(tolua_S,3,i+1,0.0));
   }
  }
  {
   glMaterialfv(face,pname,params);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glMaterial'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glMaterialf */
#ifndef TOLUA_DISABLE_tolua_gl_glMaterial01
static int tolua_gl_glMaterial01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
    double face = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float param = ((  float)  tolua_tonumber(tolua_S,3,0));
  {
   glMaterialf(face,pname,param);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glMaterial00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetMaterialfv */
#ifndef TOLUA_DISABLE_tolua_gl_glGetMaterial00
static int tolua_gl_glGetMaterial00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,1,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double face = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float params = ((  float)  tolua_tonumber(tolua_S,3,0.0));
  {
   glGetMaterialfv(face,pname,&params);
   tolua_pushnumber(tolua_S,(lua_Number)params);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGetMaterial'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetMaterialfv */
#ifndef TOLUA_DISABLE_tolua_gl_glGetMaterial01
static int tolua_gl_glGetMaterial01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
    double face = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float params[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,4,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    params[i] = ((float)  tolua_tofieldnumber(tolua_S,3,i+1,0.0));
   }
  }
  {
   glGetMaterialfv(face,pname,params);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glGetMaterial00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glColorMaterial */
#ifndef TOLUA_DISABLE_tolua_gl_glColorMaterial00
static int tolua_gl_glColorMaterial00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double face = ((   double)  tolua_tonumber(tolua_S,1,0));
    double mode = ((   double)  tolua_tonumber(tolua_S,2,0));
  {
   glColorMaterial(face,mode);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glColorMaterial'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPixelZoom */
#ifndef TOLUA_DISABLE_tolua_gl_glPixelZoom00
static int tolua_gl_glPixelZoom00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   float xfactor = ((  float)  tolua_tonumber(tolua_S,1,0));
   float yfactor = ((  float)  tolua_tonumber(tolua_S,2,0));
  {
   glPixelZoom(xfactor,yfactor);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPixelZoom'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPixelStoref */
#ifndef TOLUA_DISABLE_tolua_gl_glPixelStore00
static int tolua_gl_glPixelStore00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double pname = ((   double)  tolua_tonumber(tolua_S,1,0));
   float param = ((  float)  tolua_tonumber(tolua_S,2,0));
  {
   glPixelStoref(pname,param);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPixelStore'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPixelTransferf */
#ifndef TOLUA_DISABLE_tolua_gl_glPixelTransfer00
static int tolua_gl_glPixelTransfer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double pname = ((   double)  tolua_tonumber(tolua_S,1,0));
   float param = ((  float)  tolua_tonumber(tolua_S,2,0));
  {
   glPixelTransferf(pname,param);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPixelTransfer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPixelMapfv */
#ifndef TOLUA_DISABLE_tolua_gl_glPixelMap00
static int tolua_gl_glPixelMap00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double map = ((   double)  tolua_tonumber(tolua_S,1,0));
   int mapsize = ((  int)  tolua_tonumber(tolua_S,2,0));
#ifdef __cplusplus
   float* values = Mtolua_new_dim(float, mapsize);
#else
   float* values = (float*) malloc((mapsize)*sizeof(float));
#endif
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,mapsize,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<mapsize;i++)
    values[i] = ((float)  tolua_tofieldnumber(tolua_S,3,i+1,0));
   }
  }
  {
   glPixelMapfv(map,mapsize,values);
  }
  {
   int i;
   for(i=0; i<mapsize;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) values[i]);
  }
#ifdef __cplusplus
  Mtolua_delete_dim(values);
#else
  free(values);
#endif
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPixelMap'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glBitmap */
#ifndef TOLUA_DISABLE_tolua_gl_glBitmap00
static int tolua_gl_glBitmap00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isstring(tolua_S,7,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,8,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int width = ((  int)  tolua_tonumber(tolua_S,1,0));
   int height = ((  int)  tolua_tonumber(tolua_S,2,0));
   float xorig = ((  float)  tolua_tonumber(tolua_S,3,0));
   float yorig = ((  float)  tolua_tonumber(tolua_S,4,0));
   float xmove = ((  float)  tolua_tonumber(tolua_S,5,0));
   float ymove = ((  float)  tolua_tonumber(tolua_S,6,0));
  unsigned char* bitmap = ((unsigned char*)  tolua_tostring(tolua_S,7,0));
  {
   glBitmap(width,height,xorig,yorig,xmove,ymove,bitmap);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glBitmap'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glReadPixels */
#ifndef TOLUA_DISABLE_tolua_gl_glReadPixels00
static int tolua_gl_glReadPixels00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,7,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,8,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int x = ((  int)  tolua_tonumber(tolua_S,1,0));
   int y = ((  int)  tolua_tonumber(tolua_S,2,0));
   int width = ((  int)  tolua_tonumber(tolua_S,3,0));
   int height = ((  int)  tolua_tonumber(tolua_S,4,0));
    double format = ((   double)  tolua_tonumber(tolua_S,5,0));
    double type = ((   double)  tolua_tonumber(tolua_S,6,0));
  GLLbuffer* pixels = ((GLLbuffer*)  tolua_tousertype(tolua_S,7,0));
  {
   glReadPixels(x,y,width,height,format,type,pixels);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glReadPixels'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glDrawPixels */
#ifndef TOLUA_DISABLE_tolua_gl_glDrawPixels00
static int tolua_gl_glDrawPixels00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,5,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,6,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int width = ((  int)  tolua_tonumber(tolua_S,1,0));
   int height = ((  int)  tolua_tonumber(tolua_S,2,0));
    double format = ((   double)  tolua_tonumber(tolua_S,3,0));
    double type = ((   double)  tolua_tonumber(tolua_S,4,0));
  GLLbuffer* pixels = ((GLLbuffer*)  tolua_tousertype(tolua_S,5,0));
  {
   glDrawPixels(width,height,format,type,pixels);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glDrawPixels'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glCopyPixels */
#ifndef TOLUA_DISABLE_tolua_gl_glCopyPixels00
static int tolua_gl_glCopyPixels00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,6,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int x = ((  int)  tolua_tonumber(tolua_S,1,0));
   int y = ((  int)  tolua_tonumber(tolua_S,2,0));
   int width = ((  int)  tolua_tonumber(tolua_S,3,0));
   int height = ((  int)  tolua_tonumber(tolua_S,4,0));
    double type = ((   double)  tolua_tonumber(tolua_S,5,0));
  {
   glCopyPixels(x,y,width,height,type);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glCopyPixels'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glStencilFunc */
#ifndef TOLUA_DISABLE_tolua_gl_glStencilFunc00
static int tolua_gl_glStencilFunc00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double func = ((   double)  tolua_tonumber(tolua_S,1,0));
   int ref = ((  int)  tolua_tonumber(tolua_S,2,0));
  unsigned int mask = (( unsigned int)  tolua_tonumber(tolua_S,3,0));
  {
   glStencilFunc(func,ref,mask);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glStencilFunc'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glStencilMask */
#ifndef TOLUA_DISABLE_tolua_gl_glStencilMask00
static int tolua_gl_glStencilMask00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned int mask = (( unsigned int)  tolua_tonumber(tolua_S,1,0));
  {
   glStencilMask(mask);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glStencilMask'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glStencilOp */
#ifndef TOLUA_DISABLE_tolua_gl_glStencilOp00
static int tolua_gl_glStencilOp00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double fail = ((   double)  tolua_tonumber(tolua_S,1,0));
    double zfail = ((   double)  tolua_tonumber(tolua_S,2,0));
    double zpass = ((   double)  tolua_tonumber(tolua_S,3,0));
  {
   glStencilOp(fail,zfail,zpass);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glStencilOp'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glClearStencil */
#ifndef TOLUA_DISABLE_tolua_gl_glClearStencil00
static int tolua_gl_glClearStencil00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int s = ((  int)  tolua_tonumber(tolua_S,1,0));
  {
   glClearStencil(s);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glClearStencil'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexGendv */
#ifndef TOLUA_DISABLE_tolua_gl_glTexGen00
static int tolua_gl_glTexGen00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double coord = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   double params[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,4,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    params[i] = ((double)  tolua_tofieldnumber(tolua_S,3,i+1,0));
   }
  }
  {
   glTexGendv(coord,pname,params);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glTexGen'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexGend */
#ifndef TOLUA_DISABLE_tolua_gl_glTexGen01
static int tolua_gl_glTexGen01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
    double coord = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   double param = ((  double)  tolua_tonumber(tolua_S,3,0));
  {
   glTexGend(coord,pname,param);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glTexGen00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetTexGendv */
#ifndef TOLUA_DISABLE_tolua_gl_glGetTexGen00
static int tolua_gl_glGetTexGen00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double coord = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   double params[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,4,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    params[i] = ((double)  tolua_tofieldnumber(tolua_S,3,i+1,0));
   }
  }
  {
   glGetTexGendv(coord,pname,params);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGetTexGen'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexEnvf */
#ifndef TOLUA_DISABLE_tolua_gl_glTexEnv00
static int tolua_gl_glTexEnv00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float param = ((  float)  tolua_tonumber(tolua_S,3,0));
  {
   glTexEnvf(target,pname,param);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glTexEnv'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexEnvfv */
#ifndef TOLUA_DISABLE_tolua_gl_glTexEnvfv00
static int tolua_gl_glTexEnvfv00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float params[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,4,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    params[i] = ((float)  tolua_tofieldnumber(tolua_S,3,i+1,0));
   }
  }
  {
   glTexEnvfv(target,pname,params);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glTexEnvfv'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetTexEnvfv */
#ifndef TOLUA_DISABLE_tolua_gl_glGetTexEnv00
static int tolua_gl_glGetTexEnv00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float params[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,4,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    params[i] = ((float)  tolua_tofieldnumber(tolua_S,3,i+1,0));
   }
  }
  {
   glGetTexEnvfv(target,pname,params);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGetTexEnv'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexParameterfv */
#ifndef TOLUA_DISABLE_tolua_gl_glTexParameter00
static int tolua_gl_glTexParameter00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float params[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,4,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    params[i] = ((float)  tolua_tofieldnumber(tolua_S,3,i+1,0.0));
   }
  }
  {
   glTexParameterfv(target,pname,params);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glTexParameter'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexParameterf */
#ifndef TOLUA_DISABLE_tolua_gl_glTexParameter01
static int tolua_gl_glTexParameter01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float param = ((  float)  tolua_tonumber(tolua_S,3,0));
  {
   glTexParameterf(target,pname,param);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glTexParameter00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetTexParameterfv */
#ifndef TOLUA_DISABLE_tolua_gl_glGetTexParameter00
static int tolua_gl_glGetTexParameter00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,2,0));
   float params[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,4,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    params[i] = ((float)  tolua_tofieldnumber(tolua_S,3,i+1,0.0));
   }
  }
  {
   glGetTexParameterfv(target,pname,params);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGetTexParameter'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetTexLevelParameterfv */
#ifndef TOLUA_DISABLE_tolua_gl_glGetTexLevelParameter00
static int tolua_gl_glGetTexLevelParameter00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_istable(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
   int level = ((  int)  tolua_tonumber(tolua_S,2,0));
    double pname = ((   double)  tolua_tonumber(tolua_S,3,0));
   float params[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,4,4,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    params[i] = ((float)  tolua_tofieldnumber(tolua_S,4,i+1,0.0));
   }
  }
  {
   glGetTexLevelParameterfv(target,level,pname,params);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,4,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGetTexLevelParameter'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexImage1D */
#ifndef TOLUA_DISABLE_tolua_gl_glTexImage1D00
static int tolua_gl_glTexImage1D00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,7,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,8,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,9,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
   int level = ((  int)  tolua_tonumber(tolua_S,2,0));
   int components = ((  int)  tolua_tonumber(tolua_S,3,0));
   int width = ((  int)  tolua_tonumber(tolua_S,4,0));
   int border = ((  int)  tolua_tonumber(tolua_S,5,0));
    double format = ((   double)  tolua_tonumber(tolua_S,6,0));
    double type = ((   double)  tolua_tonumber(tolua_S,7,0));
  GLLbuffer* pixels = ((GLLbuffer*)  tolua_tousertype(tolua_S,8,0));
  {
   glTexImage1D(target,level,components,width,border,format,type,pixels);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glTexImage1D'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexImage2D */
#ifndef TOLUA_DISABLE_tolua_gl_glTexImage2D00
static int tolua_gl_glTexImage2D00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,7,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,8,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,9,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,10,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
   int level = ((  int)  tolua_tonumber(tolua_S,2,0));
   int components = ((  int)  tolua_tonumber(tolua_S,3,0));
   int width = ((  int)  tolua_tonumber(tolua_S,4,0));
   int height = ((  int)  tolua_tonumber(tolua_S,5,0));
   int border = ((  int)  tolua_tonumber(tolua_S,6,0));
    double format = ((   double)  tolua_tonumber(tolua_S,7,0));
    double type = ((   double)  tolua_tonumber(tolua_S,8,0));
  GLLbuffer* pixels = ((GLLbuffer*)  tolua_tousertype(tolua_S,9,0));
  {
   glTexImage2D(target,level,components,width,height,border,format,type,pixels);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glTexImage2D'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetTexImage */
#ifndef TOLUA_DISABLE_tolua_gl_glGetTexImage00
static int tolua_gl_glGetTexImage00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,5,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,6,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
   int level = ((  int)  tolua_tonumber(tolua_S,2,0));
    double format = ((   double)  tolua_tonumber(tolua_S,3,0));
    double type = ((   double)  tolua_tonumber(tolua_S,4,0));
  GLLbuffer* pixels = ((GLLbuffer*)  tolua_tousertype(tolua_S,5,0));
  {
   glGetTexImage(target,level,format,type,pixels);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGetTexImage'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glMap1d */
#ifndef TOLUA_DISABLE_tolua_gl_glMap100
static int tolua_gl_glMap100(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_istable(tolua_S,6,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,7,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
   double u1 = ((  double)  tolua_tonumber(tolua_S,2,0));
   double u2 = ((  double)  tolua_tonumber(tolua_S,3,0));
   int stride = ((  int)  tolua_tonumber(tolua_S,4,0));
   int order = ((  int)  tolua_tonumber(tolua_S,5,0));
#ifdef __cplusplus
   double* points = Mtolua_new_dim(double, order);
#else
   double* points = (double*) malloc((order)*sizeof(double));
#endif
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,6,order,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<order;i++)
    points[i] = ((double)  tolua_tofieldnumber(tolua_S,6,i+1,0));
   }
  }
  {
   glMap1d(target,u1,u2,stride,order,points);
  }
  {
   int i;
   for(i=0; i<order;i++)
    tolua_pushfieldnumber(tolua_S,6,i+1,(lua_Number) points[i]);
  }
#ifdef __cplusplus
  Mtolua_delete_dim(points);
#else
  free(points);
#endif
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glMap1'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glMap2d */
#ifndef TOLUA_DISABLE_tolua_gl_glMap200
static int tolua_gl_glMap200(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,7,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,8,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,9,0,&tolua_err) ||
     !tolua_istable(tolua_S,10,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,11,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
   double u1 = ((  double)  tolua_tonumber(tolua_S,2,0));
   double u2 = ((  double)  tolua_tonumber(tolua_S,3,0));
   int ustride = ((  int)  tolua_tonumber(tolua_S,4,0));
   int uorder = ((  int)  tolua_tonumber(tolua_S,5,0));
   double v1 = ((  double)  tolua_tonumber(tolua_S,6,0));
   double v2 = ((  double)  tolua_tonumber(tolua_S,7,0));
   int vstride = ((  int)  tolua_tonumber(tolua_S,8,0));
   int vorder = ((  int)  tolua_tonumber(tolua_S,9,0));
#ifdef __cplusplus
   double* points = Mtolua_new_dim(double, uorder*vorder);
#else
   double* points = (double*) malloc((uorder*vorder)*sizeof(double));
#endif
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,10,uorder*vorder,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<uorder*vorder;i++)
    points[i] = ((double)  tolua_tofieldnumber(tolua_S,10,i+1,0));
   }
  }
  {
   glMap2d(target,u1,u2,ustride,uorder,v1,v2,vstride,vorder,points);
  }
  {
   int i;
   for(i=0; i<uorder*vorder;i++)
    tolua_pushfieldnumber(tolua_S,10,i+1,(lua_Number) points[i]);
  }
#ifdef __cplusplus
  Mtolua_delete_dim(points);
#else
  free(points);
#endif
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glMap2'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEvalCoord1dv */
#ifndef TOLUA_DISABLE_tolua_gl_glEvalCoord00
static int tolua_gl_glEvalCoord00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double u[1];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,1,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<1;i++)
    u[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,0));
   }
  }
  {
   glEvalCoord1dv(u);
  }
  {
   int i;
   for(i=0; i<1;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) u[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glEvalCoord'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEvalCoord2dv */
#ifndef TOLUA_DISABLE_tolua_gl_glEvalCoord01
static int tolua_gl_glEvalCoord01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_istable(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   double u[2];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,1,2,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<2;i++)
    u[i] = ((double)  tolua_tofieldnumber(tolua_S,1,i+1,0));
   }
  }
  {
   glEvalCoord2dv(u);
  }
  {
   int i;
   for(i=0; i<2;i++)
    tolua_pushfieldnumber(tolua_S,1,i+1,(lua_Number) u[i]);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glEvalCoord00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEvalCoord1d */
#ifndef TOLUA_DISABLE_tolua_gl_glEvalCoord02
static int tolua_gl_glEvalCoord02(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   double u = ((  double)  tolua_tonumber(tolua_S,1,0));
  {
   glEvalCoord1d(u);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glEvalCoord01(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEvalCoord2d */
#ifndef TOLUA_DISABLE_tolua_gl_glEvalCoord03
static int tolua_gl_glEvalCoord03(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   double u = ((  double)  tolua_tonumber(tolua_S,1,0));
   double v = ((  double)  tolua_tonumber(tolua_S,2,0));
  {
   glEvalCoord2d(u,v);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glEvalCoord02(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glMapGrid1d */
#ifndef TOLUA_DISABLE_tolua_gl_glMapGrid00
static int tolua_gl_glMapGrid00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int un = ((  int)  tolua_tonumber(tolua_S,1,0));
   double u1 = ((  double)  tolua_tonumber(tolua_S,2,0));
   double u2 = ((  double)  tolua_tonumber(tolua_S,3,0));
  {
   glMapGrid1d(un,u1,u2);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glMapGrid'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glMapGrid2d */
#ifndef TOLUA_DISABLE_tolua_gl_glMapGrid01
static int tolua_gl_glMapGrid01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,7,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   int un = ((  int)  tolua_tonumber(tolua_S,1,0));
   double u1 = ((  double)  tolua_tonumber(tolua_S,2,0));
   double u2 = ((  double)  tolua_tonumber(tolua_S,3,0));
   int vn = ((  int)  tolua_tonumber(tolua_S,4,0));
   double v1 = ((  double)  tolua_tonumber(tolua_S,5,0));
   double v2 = ((  double)  tolua_tonumber(tolua_S,6,0));
  {
   glMapGrid2d(un,u1,u2,vn,v1,v2);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glMapGrid00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEvalPoint1 */
#ifndef TOLUA_DISABLE_tolua_gl_glEvalPoint00
static int tolua_gl_glEvalPoint00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int i = ((  int)  tolua_tonumber(tolua_S,1,0));
  {
   glEvalPoint1(i);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glEvalPoint'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEvalPoint2 */
#ifndef TOLUA_DISABLE_tolua_gl_glEvalPoint01
static int tolua_gl_glEvalPoint01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
   int i = ((  int)  tolua_tonumber(tolua_S,1,0));
   int j = ((  int)  tolua_tonumber(tolua_S,2,0));
  {
   glEvalPoint2(i,j);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glEvalPoint00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEvalMesh1 */
#ifndef TOLUA_DISABLE_tolua_gl_glEvalMesh00
static int tolua_gl_glEvalMesh00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double mode = ((   double)  tolua_tonumber(tolua_S,1,0));
   int i1 = ((  int)  tolua_tonumber(tolua_S,2,0));
   int i2 = ((  int)  tolua_tonumber(tolua_S,3,0));
  {
   glEvalMesh1(mode,i1,i2);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glEvalMesh'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEvalMesh2 */
#ifndef TOLUA_DISABLE_tolua_gl_glEvalMesh01
static int tolua_gl_glEvalMesh01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,6,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
    double mode = ((   double)  tolua_tonumber(tolua_S,1,0));
   int i1 = ((  int)  tolua_tonumber(tolua_S,2,0));
   int i2 = ((  int)  tolua_tonumber(tolua_S,3,0));
   int j1 = ((  int)  tolua_tonumber(tolua_S,4,0));
   int j2 = ((  int)  tolua_tonumber(tolua_S,5,0));
  {
   glEvalMesh2(mode,i1,i2,j1,j2);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glEvalMesh00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glFogfv */
#ifndef TOLUA_DISABLE_tolua_gl_glFog00
static int tolua_gl_glFog00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_istable(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double pname = ((   double)  tolua_tonumber(tolua_S,1,0));
   float params[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,2,4,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    params[i] = ((float)  tolua_tofieldnumber(tolua_S,2,i+1,0));
   }
  }
  {
   glFogfv(pname,params);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,2,i+1,(lua_Number) params[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glFog'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glFogf */
#ifndef TOLUA_DISABLE_tolua_gl_glFog01
static int tolua_gl_glFog01(lua_State* tolua_S)
{
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
 {
    double pname = ((   double)  tolua_tonumber(tolua_S,1,0));
   float param = ((  float)  tolua_tonumber(tolua_S,2,0));
  {
   glFogf(pname,param);
  }
 }
 return 0;
tolua_lerror:
 return tolua_gl_glFog00(tolua_S);
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPassThrough */
#ifndef TOLUA_DISABLE_tolua_gl_glPassThrough00
static int tolua_gl_glPassThrough00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   float token = ((  float)  tolua_tonumber(tolua_S,1,0));
  {
   glPassThrough(token);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPassThrough'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glInitNames */
#ifndef TOLUA_DISABLE_tolua_gl_glInitNames00
static int tolua_gl_glInitNames00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnoobj(tolua_S,1,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   glInitNames();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glInitNames'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glLoadName */
#ifndef TOLUA_DISABLE_tolua_gl_glLoadName00
static int tolua_gl_glLoadName00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned int name = (( unsigned int)  tolua_tonumber(tolua_S,1,0));
  {
   glLoadName(name);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glLoadName'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPushName */
#ifndef TOLUA_DISABLE_tolua_gl_glPushName00
static int tolua_gl_glPushName00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned int name = (( unsigned int)  tolua_tonumber(tolua_S,1,0));
  {
   glPushName(name);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPushName'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPopName */
#ifndef TOLUA_DISABLE_tolua_gl_glPopName00
static int tolua_gl_glPopName00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnoobj(tolua_S,1,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   glPopName();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPopName'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPolygonOffset */
#ifndef TOLUA_DISABLE_tolua_gl_glPolygonOffset00
static int tolua_gl_glPolygonOffset00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   float factor = ((  float)  tolua_tonumber(tolua_S,1,0));
   float units = ((  float)  tolua_tonumber(tolua_S,2,0));
  {
   glPolygonOffset(factor,units);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPolygonOffset'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEnableClientState */
#ifndef TOLUA_DISABLE_tolua_gl_glEnableClientState00
static int tolua_gl_glEnableClientState00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double cap = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   glEnableClientState(cap);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glEnableClientState'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glDisableClientState */
#ifndef TOLUA_DISABLE_tolua_gl_glDisableClientState00
static int tolua_gl_glDisableClientState00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double cap = ((   double)  tolua_tonumber(tolua_S,1,0));
  {
   glDisableClientState(cap);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glDisableClientState'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPushClientAttrib */
#ifndef TOLUA_DISABLE_tolua_gl_glPushClientAttrib00
static int tolua_gl_glPushClientAttrib00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     (tolua_isvaluenil(tolua_S,1,&tolua_err) || !tolua_isusertype(tolua_S,1,"GLbitfield",0,&tolua_err)) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLbitfield mask = *((GLbitfield*)  tolua_tousertype(tolua_S,1,0));
  {
   glPushClientAttrib(mask);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPushClientAttrib'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPopClientAttrib */
#ifndef TOLUA_DISABLE_tolua_gl_glPopClientAttrib00
static int tolua_gl_glPopClientAttrib00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnoobj(tolua_S,1,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  {
   glPopClientAttrib();
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPopClientAttrib'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glVertexPointer */
#ifndef TOLUA_DISABLE_tolua_gl_glVertexPointer00
static int tolua_gl_glVertexPointer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,4,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int size = ((  int)  tolua_tonumber(tolua_S,1,0));
    double type = ((   double)  tolua_tonumber(tolua_S,2,0));
   int stride = ((  int)  tolua_tonumber(tolua_S,3,0));
  GLLbuffer* ptr = ((GLLbuffer*)  tolua_tousertype(tolua_S,4,0));
  {
   glVertexPointer(size,type,stride,ptr);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glVertexPointer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glNormalPointer */
#ifndef TOLUA_DISABLE_tolua_gl_glNormalPointer00
static int tolua_gl_glNormalPointer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,3,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double type = ((   double)  tolua_tonumber(tolua_S,1,0));
   int stride = ((  int)  tolua_tonumber(tolua_S,2,0));
  GLLbuffer* ptr = ((GLLbuffer*)  tolua_tousertype(tolua_S,3,0));
  {
   glNormalPointer(type,stride,ptr);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glNormalPointer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glColorPointer */
#ifndef TOLUA_DISABLE_tolua_gl_glColorPointer00
static int tolua_gl_glColorPointer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,4,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int size = ((  int)  tolua_tonumber(tolua_S,1,0));
    double type = ((   double)  tolua_tonumber(tolua_S,2,0));
   int stride = ((  int)  tolua_tonumber(tolua_S,3,0));
  GLLbuffer* ptr = ((GLLbuffer*)  tolua_tousertype(tolua_S,4,0));
  {
   glColorPointer(size,type,stride,ptr);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glColorPointer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glIndexPointer */
#ifndef TOLUA_DISABLE_tolua_gl_glIndexPointer00
static int tolua_gl_glIndexPointer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,3,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double type = ((   double)  tolua_tonumber(tolua_S,1,0));
   int stride = ((  int)  tolua_tonumber(tolua_S,2,0));
  GLLbuffer* ptr = ((GLLbuffer*)  tolua_tousertype(tolua_S,3,0));
  {
   glIndexPointer(type,stride,ptr);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glIndexPointer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexCoordPointer */
#ifndef TOLUA_DISABLE_tolua_gl_glTexCoordPointer00
static int tolua_gl_glTexCoordPointer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,4,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int size = ((  int)  tolua_tonumber(tolua_S,1,0));
    double type = ((   double)  tolua_tonumber(tolua_S,2,0));
   int stride = ((  int)  tolua_tonumber(tolua_S,3,0));
  GLLbuffer* ptr = ((GLLbuffer*)  tolua_tousertype(tolua_S,4,0));
  {
   glTexCoordPointer(size,type,stride,ptr);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glTexCoordPointer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glEdgeFlagPointer */
#ifndef TOLUA_DISABLE_tolua_gl_glEdgeFlagPointer00
static int tolua_gl_glEdgeFlagPointer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int stride = ((  int)  tolua_tonumber(tolua_S,1,0));
  unsigned char ptr = (( unsigned char)  tolua_tonumber(tolua_S,2,0));
  {
   glEdgeFlagPointer(stride,&ptr);
   tolua_pushnumber(tolua_S,(lua_Number)ptr);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glEdgeFlagPointer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGetPointerv */
#ifndef TOLUA_DISABLE_tolua_gl_glGetPointerv00
static int tolua_gl_glGetPointerv00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isuserdata(tolua_S,2,1,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double pname = ((   double)  tolua_tonumber(tolua_S,1,0));
  void* params = ((void*)  tolua_touserdata(tolua_S,2,0));
  {
   glGetPointerv(pname,&params);
   tolua_pushuserdata(tolua_S,(void*)params);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGetPointerv'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glArrayElement */
#ifndef TOLUA_DISABLE_tolua_gl_glArrayElement00
static int tolua_gl_glArrayElement00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int i = ((  int)  tolua_tonumber(tolua_S,1,0));
  {
   glArrayElement(i);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glArrayElement'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glDrawArrays */
#ifndef TOLUA_DISABLE_tolua_gl_glDrawArrays00
static int tolua_gl_glDrawArrays00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double mode = ((   double)  tolua_tonumber(tolua_S,1,0));
   int first = ((  int)  tolua_tonumber(tolua_S,2,0));
   int count = ((  int)  tolua_tonumber(tolua_S,3,0));
  {
   glDrawArrays(mode,first,count);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glDrawArrays'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glDrawElements */
#ifndef TOLUA_DISABLE_tolua_gl_glDrawElements00
static int tolua_gl_glDrawElements00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,4,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double mode = ((   double)  tolua_tonumber(tolua_S,1,0));
   int count = ((  int)  tolua_tonumber(tolua_S,2,0));
    double type = ((   double)  tolua_tonumber(tolua_S,3,0));
  GLLbuffer* indices = ((GLLbuffer*)  tolua_tousertype(tolua_S,4,0));
  {
   glDrawElements(mode,count,type,indices);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glDrawElements'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glInterleavedArrays */
#ifndef TOLUA_DISABLE_tolua_gl_glInterleavedArrays00
static int tolua_gl_glInterleavedArrays00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,3,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double format = ((   double)  tolua_tonumber(tolua_S,1,0));
   int stride = ((  int)  tolua_tonumber(tolua_S,2,0));
  GLLbuffer* pointer = ((GLLbuffer*)  tolua_tousertype(tolua_S,3,0));
  {
   glInterleavedArrays(format,stride,pointer);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glInterleavedArrays'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glGenTextures */
#ifndef TOLUA_DISABLE_tolua_gl_glGenTextures00
static int tolua_gl_glGenTextures00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_istable(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int n = ((  int)  tolua_tonumber(tolua_S,1,0));
#ifdef __cplusplus
  unsigned int* textures = Mtolua_new_dim(unsigned int, n);
#else
  unsigned int* textures = (int*) malloc((n)*sizeof(int));
#endif
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,2,n,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<n;i++)
    textures[i] = ((int)  tolua_tofieldnumber(tolua_S,2,i+1,0));
   }
  }
  {
   glGenTextures(n,textures);
  }
  {
   int i;
   for(i=0; i<n;i++)
    tolua_pushfieldnumber(tolua_S,2,i+1,(lua_Number) textures[i]);
  }
#ifdef __cplusplus
  Mtolua_delete_dim(textures);
#else
  free(textures);
#endif
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glGenTextures'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glDeleteTextures */
#ifndef TOLUA_DISABLE_tolua_gl_glDeleteTextures00
static int tolua_gl_glDeleteTextures00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_istable(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int n = ((  int)  tolua_tonumber(tolua_S,1,0));
#ifdef __cplusplus
  unsigned int* textures = Mtolua_new_dim(unsigned int, n);
#else
  unsigned int* textures = (int*) malloc((n)*sizeof(int));
#endif
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,2,n,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<n;i++)
    textures[i] = ((int)  tolua_tofieldnumber(tolua_S,2,i+1,0));
   }
  }
  {
   glDeleteTextures(n,textures);
  }
  {
   int i;
   for(i=0; i<n;i++)
    tolua_pushfieldnumber(tolua_S,2,i+1,(lua_Number) textures[i]);
  }
#ifdef __cplusplus
  Mtolua_delete_dim(textures);
#else
  free(textures);
#endif
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glDeleteTextures'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glBindTexture */
#ifndef TOLUA_DISABLE_tolua_gl_glBindTexture00
static int tolua_gl_glBindTexture00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
  unsigned int texture = (( unsigned int)  tolua_tonumber(tolua_S,2,0));
  {
   glBindTexture(target,texture);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glBindTexture'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glPrioritizeTextures */
#ifndef TOLUA_DISABLE_tolua_gl_glPrioritizeTextures00
static int tolua_gl_glPrioritizeTextures00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_istable(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int n = ((  int)  tolua_tonumber(tolua_S,1,0));
#ifdef __cplusplus
  unsigned int* textures = Mtolua_new_dim(unsigned int, n);
#else
  unsigned int* textures = (int*) malloc((n)*sizeof(int));
#endif
#ifdef __cplusplus
   float* priorities = Mtolua_new_dim(float, n);
#else
   float* priorities = (float*) malloc((n)*sizeof(float));
#endif
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,2,n,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<n;i++)
    textures[i] = ((int)  tolua_tofieldnumber(tolua_S,2,i+1,0));
   }
  }
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,n,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<n;i++)
    priorities[i] = ((float)  tolua_tofieldnumber(tolua_S,3,i+1,0));
   }
  }
  {
   glPrioritizeTextures(n,textures,priorities);
  }
  {
   int i;
   for(i=0; i<n;i++)
    tolua_pushfieldnumber(tolua_S,2,i+1,(lua_Number) textures[i]);
  }
  {
   int i;
   for(i=0; i<n;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) priorities[i]);
  }
#ifdef __cplusplus
  Mtolua_delete_dim(textures);
#else
  free(textures);
#endif
#ifdef __cplusplus
  Mtolua_delete_dim(priorities);
#else
  free(priorities);
#endif
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glPrioritizeTextures'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glAreTexturesResident */
#ifndef TOLUA_DISABLE_tolua_gl_glAreTexturesResident00
static int tolua_gl_glAreTexturesResident00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_istable(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int n = ((  int)  tolua_tonumber(tolua_S,1,0));
#ifdef __cplusplus
  unsigned int* textures = Mtolua_new_dim(unsigned int, n);
#else
  unsigned int* textures = (int*) malloc((n)*sizeof(int));
#endif
#ifdef __cplusplus
  unsigned char* residences = Mtolua_new_dim(unsigned char, n);
#else
  unsigned char* residences = (char*) malloc((n)*sizeof(char));
#endif
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,2,n,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<n;i++)
    textures[i] = ((int)  tolua_tofieldnumber(tolua_S,2,i+1,0));
   }
  }
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,n,1,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<n;i++)
    residences[i] = ((char)  tolua_tofieldnumber(tolua_S,3,i+1,0));
   }
  }
  {
   unsigned char tolua_ret = ( unsigned char)  glAreTexturesResident(n,textures,residences);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
  {
   int i;
   for(i=0; i<n;i++)
    tolua_pushfieldnumber(tolua_S,2,i+1,(lua_Number) textures[i]);
  }
  {
   int i;
   for(i=0; i<n;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) residences[i]);
  }
#ifdef __cplusplus
  Mtolua_delete_dim(textures);
#else
  free(textures);
#endif
#ifdef __cplusplus
  Mtolua_delete_dim(residences);
#else
  free(residences);
#endif
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glAreTexturesResident'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glIsTexture */
#ifndef TOLUA_DISABLE_tolua_gl_glIsTexture00
static int tolua_gl_glIsTexture00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  unsigned int texture = (( unsigned int)  tolua_tonumber(tolua_S,1,0));
  {
   unsigned char tolua_ret = ( unsigned char)  glIsTexture(texture);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glIsTexture'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexSubImage1D */
#ifndef TOLUA_DISABLE_tolua_gl_glTexSubImage1D00
static int tolua_gl_glTexSubImage1D00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,7,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,8,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
   int level = ((  int)  tolua_tonumber(tolua_S,2,0));
   int xoffset = ((  int)  tolua_tonumber(tolua_S,3,0));
   int width = ((  int)  tolua_tonumber(tolua_S,4,0));
    double format = ((   double)  tolua_tonumber(tolua_S,5,0));
    double type = ((   double)  tolua_tonumber(tolua_S,6,0));
  GLLbuffer* pixels = ((GLLbuffer*)  tolua_tousertype(tolua_S,7,0));
  {
   glTexSubImage1D(target,level,xoffset,width,format,type,pixels);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glTexSubImage1D'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glTexSubImage2D */
#ifndef TOLUA_DISABLE_tolua_gl_glTexSubImage2D00
static int tolua_gl_glTexSubImage2D00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,7,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,8,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,9,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,10,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
   int level = ((  int)  tolua_tonumber(tolua_S,2,0));
   int xoffset = ((  int)  tolua_tonumber(tolua_S,3,0));
   int yoffset = ((  int)  tolua_tonumber(tolua_S,4,0));
   int width = ((  int)  tolua_tonumber(tolua_S,5,0));
   int height = ((  int)  tolua_tonumber(tolua_S,6,0));
    double format = ((   double)  tolua_tonumber(tolua_S,7,0));
    double type = ((   double)  tolua_tonumber(tolua_S,8,0));
  GLLbuffer* pixels = ((GLLbuffer*)  tolua_tousertype(tolua_S,9,0));
  {
   glTexSubImage2D(target,level,xoffset,yoffset,width,height,format,type,pixels);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glTexSubImage2D'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glCopyTexImage1D */
#ifndef TOLUA_DISABLE_tolua_gl_glCopyTexImage1D00
static int tolua_gl_glCopyTexImage1D00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,7,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,8,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
   int level = ((  int)  tolua_tonumber(tolua_S,2,0));
    double internalformat = ((   double)  tolua_tonumber(tolua_S,3,0));
   int x = ((  int)  tolua_tonumber(tolua_S,4,0));
   int y = ((  int)  tolua_tonumber(tolua_S,5,0));
   int width = ((  int)  tolua_tonumber(tolua_S,6,0));
   int border = ((  int)  tolua_tonumber(tolua_S,7,0));
  {
   glCopyTexImage1D(target,level,internalformat,x,y,width,border);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glCopyTexImage1D'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glCopyTexImage2D */
#ifndef TOLUA_DISABLE_tolua_gl_glCopyTexImage2D00
static int tolua_gl_glCopyTexImage2D00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,7,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,8,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,9,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
   int level = ((  int)  tolua_tonumber(tolua_S,2,0));
    double internalformat = ((   double)  tolua_tonumber(tolua_S,3,0));
   int x = ((  int)  tolua_tonumber(tolua_S,4,0));
   int y = ((  int)  tolua_tonumber(tolua_S,5,0));
   int width = ((  int)  tolua_tonumber(tolua_S,6,0));
   int height = ((  int)  tolua_tonumber(tolua_S,7,0));
   int border = ((  int)  tolua_tonumber(tolua_S,8,0));
  {
   glCopyTexImage2D(target,level,internalformat,x,y,width,height,border);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glCopyTexImage2D'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glCopyTexSubImage1D */
#ifndef TOLUA_DISABLE_tolua_gl_glCopyTexSubImage1D00
static int tolua_gl_glCopyTexSubImage1D00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,7,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
   int level = ((  int)  tolua_tonumber(tolua_S,2,0));
   int xoffset = ((  int)  tolua_tonumber(tolua_S,3,0));
   int x = ((  int)  tolua_tonumber(tolua_S,4,0));
   int y = ((  int)  tolua_tonumber(tolua_S,5,0));
   int width = ((  int)  tolua_tonumber(tolua_S,6,0));
  {
   glCopyTexSubImage1D(target,level,xoffset,x,y,width);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glCopyTexSubImage1D'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: glCopyTexSubImage2D */
#ifndef TOLUA_DISABLE_tolua_gl_glCopyTexSubImage2D00
static int tolua_gl_glCopyTexSubImage2D00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,7,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,8,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,9,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
    double target = ((   double)  tolua_tonumber(tolua_S,1,0));
   int level = ((  int)  tolua_tonumber(tolua_S,2,0));
   int xoffset = ((  int)  tolua_tonumber(tolua_S,3,0));
   int yoffset = ((  int)  tolua_tonumber(tolua_S,4,0));
   int x = ((  int)  tolua_tonumber(tolua_S,5,0));
   int y = ((  int)  tolua_tonumber(tolua_S,6,0));
   int width = ((  int)  tolua_tonumber(tolua_S,7,0));
   int height = ((  int)  tolua_tonumber(tolua_S,8,0));
  {
   glCopyTexSubImage2D(target,level,xoffset,yoffset,x,y,width,height);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'glCopyTexSubImage2D'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gllCreateBuffer2 */
#ifndef TOLUA_DISABLE_tolua_gl_gllCreateBuffer00
static int tolua_gl_gllCreateBuffer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int n = ((  int)  tolua_tonumber(tolua_S,1,0));
    double type = ((   double)  tolua_tonumber(tolua_S,2,0));
  {
   GLLbuffer* tolua_ret = (GLLbuffer*)  gllCreateBuffer2(n,type);
    tolua_pushusertype(tolua_S,(void*)tolua_ret,"GLLbuffer");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gllCreateBuffer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gllDeleteBuffer */
#ifndef TOLUA_DISABLE_tolua_gl_gllDeleteBuffer00
static int tolua_gl_gllDeleteBuffer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLLbuffer* b = ((GLLbuffer*)  tolua_tousertype(tolua_S,1,0));
  {
   gllDeleteBuffer(b);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gllDeleteBuffer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gllSetBuffer */
#ifndef TOLUA_DISABLE_tolua_gl_gllSetBuffer00
static int tolua_gl_gllSetBuffer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLLbuffer* b = ((GLLbuffer*)  tolua_tousertype(tolua_S,1,0));
  int i = ((int)  tolua_tonumber(tolua_S,2,0));
  float v = ((float)  tolua_tonumber(tolua_S,3,0));
  {
   gllSetBuffer(b,i,v);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gllSetBuffer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gllGetBuffer */
#ifndef TOLUA_DISABLE_tolua_gl_gllGetBuffer00
static int tolua_gl_gllGetBuffer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLLbuffer* b = ((GLLbuffer*)  tolua_tousertype(tolua_S,1,0));
  int i = ((int)  tolua_tonumber(tolua_S,2,0));
  {
   float tolua_ret = (float)  gllGetBuffer(b,i);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gllGetBuffer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* Open function */
TOLUA_API int tolua_gl_open (lua_State* tolua_S)
{
 tolua_open(tolua_S);
 tolua_reg_types(tolua_S);
 tolua_module(tolua_S,NULL,0);
 tolua_beginmodule(tolua_S,NULL);
#ifdef GL_VERSION_1_1
#endif
  tolua_constant(tolua_S,"GL_FALSE",GL_FALSE);
  tolua_constant(tolua_S,"GL_TRUE",GL_TRUE);
  tolua_constant(tolua_S,"GL_BYTE",GL_BYTE);
  tolua_constant(tolua_S,"GL_UNSIGNED_BYTE",GL_UNSIGNED_BYTE);
  tolua_constant(tolua_S,"GL_SHORT",GL_SHORT);
  tolua_constant(tolua_S,"GL_UNSIGNED_SHORT",GL_UNSIGNED_SHORT);
  tolua_constant(tolua_S,"GL_INT",GL_INT);
  tolua_constant(tolua_S,"GL_UNSIGNED_INT",GL_UNSIGNED_INT);
  tolua_constant(tolua_S,"GL_FLOAT",GL_FLOAT);
  tolua_constant(tolua_S,"GL_2_BYTES",GL_2_BYTES);
  tolua_constant(tolua_S,"GL_3_BYTES",GL_3_BYTES);
  tolua_constant(tolua_S,"GL_4_BYTES",GL_4_BYTES);
  tolua_constant(tolua_S,"GL_LINES",GL_LINES);
  tolua_constant(tolua_S,"GL_POINTS",GL_POINTS);
  tolua_constant(tolua_S,"GL_LINE_STRIP",GL_LINE_STRIP);
  tolua_constant(tolua_S,"GL_LINE_LOOP",GL_LINE_LOOP);
  tolua_constant(tolua_S,"GL_TRIANGLES",GL_TRIANGLES);
  tolua_constant(tolua_S,"GL_TRIANGLE_STRIP",GL_TRIANGLE_STRIP);
  tolua_constant(tolua_S,"GL_TRIANGLE_FAN",GL_TRIANGLE_FAN);
  tolua_constant(tolua_S,"GL_QUADS",GL_QUADS);
  tolua_constant(tolua_S,"GL_QUAD_STRIP",GL_QUAD_STRIP);
  tolua_constant(tolua_S,"GL_POLYGON",GL_POLYGON);
  tolua_constant(tolua_S,"GL_EDGE_FLAG",GL_EDGE_FLAG);
  tolua_constant(tolua_S,"GL_MATRIX_MODE",GL_MATRIX_MODE);
  tolua_constant(tolua_S,"GL_MODELVIEW",GL_MODELVIEW);
  tolua_constant(tolua_S,"GL_PROJECTION",GL_PROJECTION);
  tolua_constant(tolua_S,"GL_TEXTURE",GL_TEXTURE);
  tolua_constant(tolua_S,"GL_POINT_SMOOTH",GL_POINT_SMOOTH);
  tolua_constant(tolua_S,"GL_POINT_SIZE",GL_POINT_SIZE);
  tolua_constant(tolua_S,"GL_POINT_SIZE_GRANULARITY",GL_POINT_SIZE_GRANULARITY);
  tolua_constant(tolua_S,"GL_POINT_SIZE_RANGE",GL_POINT_SIZE_RANGE);
  tolua_constant(tolua_S,"GL_LINE_SMOOTH",GL_LINE_SMOOTH);
  tolua_constant(tolua_S,"GL_LINE_STIPPLE",GL_LINE_STIPPLE);
  tolua_constant(tolua_S,"GL_LINE_STIPPLE_PATTERN",GL_LINE_STIPPLE_PATTERN);
  tolua_constant(tolua_S,"GL_LINE_STIPPLE_REPEAT",GL_LINE_STIPPLE_REPEAT);
  tolua_constant(tolua_S,"GL_LINE_WIDTH",GL_LINE_WIDTH);
  tolua_constant(tolua_S,"GL_LINE_WIDTH_GRANULARITY",GL_LINE_WIDTH_GRANULARITY);
  tolua_constant(tolua_S,"GL_LINE_WIDTH_RANGE",GL_LINE_WIDTH_RANGE);
  tolua_constant(tolua_S,"GL_POINT",GL_POINT);
  tolua_constant(tolua_S,"GL_LINE",GL_LINE);
  tolua_constant(tolua_S,"GL_FILL",GL_FILL);
  tolua_constant(tolua_S,"GL_CCW",GL_CCW);
  tolua_constant(tolua_S,"GL_CW",GL_CW);
  tolua_constant(tolua_S,"GL_FRONT",GL_FRONT);
  tolua_constant(tolua_S,"GL_BACK",GL_BACK);
  tolua_constant(tolua_S,"GL_CULL_FACE",GL_CULL_FACE);
  tolua_constant(tolua_S,"GL_CULL_FACE_MODE",GL_CULL_FACE_MODE);
  tolua_constant(tolua_S,"GL_POLYGON_SMOOTH",GL_POLYGON_SMOOTH);
  tolua_constant(tolua_S,"GL_POLYGON_STIPPLE",GL_POLYGON_STIPPLE);
  tolua_constant(tolua_S,"GL_FRONT_FACE",GL_FRONT_FACE);
  tolua_constant(tolua_S,"GL_POLYGON_MODE",GL_POLYGON_MODE);
  tolua_constant(tolua_S,"GL_COMPILE",GL_COMPILE);
  tolua_constant(tolua_S,"GL_COMPILE_AND_EXECUTE",GL_COMPILE_AND_EXECUTE);
  tolua_constant(tolua_S,"GL_LIST_BASE",GL_LIST_BASE);
  tolua_constant(tolua_S,"GL_LIST_INDEX",GL_LIST_INDEX);
  tolua_constant(tolua_S,"GL_LIST_MODE",GL_LIST_MODE);
  tolua_constant(tolua_S,"GL_NEVER",GL_NEVER);
  tolua_constant(tolua_S,"GL_LESS",GL_LESS);
  tolua_constant(tolua_S,"GL_GEQUAL",GL_GEQUAL);
  tolua_constant(tolua_S,"GL_LEQUAL",GL_LEQUAL);
  tolua_constant(tolua_S,"GL_GREATER",GL_GREATER);
  tolua_constant(tolua_S,"GL_NOTEQUAL",GL_NOTEQUAL);
  tolua_constant(tolua_S,"GL_EQUAL",GL_EQUAL);
  tolua_constant(tolua_S,"GL_ALWAYS",GL_ALWAYS);
  tolua_constant(tolua_S,"GL_DEPTH_TEST",GL_DEPTH_TEST);
  tolua_constant(tolua_S,"GL_DEPTH_BITS",GL_DEPTH_BITS);
  tolua_constant(tolua_S,"GL_DEPTH_CLEAR_VALUE",GL_DEPTH_CLEAR_VALUE);
  tolua_constant(tolua_S,"GL_DEPTH_FUNC",GL_DEPTH_FUNC);
  tolua_constant(tolua_S,"GL_DEPTH_RANGE",GL_DEPTH_RANGE);
  tolua_constant(tolua_S,"GL_DEPTH_WRITEMASK",GL_DEPTH_WRITEMASK);
  tolua_constant(tolua_S,"GL_DEPTH_COMPONENT",GL_DEPTH_COMPONENT);
  tolua_constant(tolua_S,"GL_LIGHTING",GL_LIGHTING);
  tolua_constant(tolua_S,"GL_LIGHT0",GL_LIGHT0);
  tolua_constant(tolua_S,"GL_LIGHT1",GL_LIGHT1);
  tolua_constant(tolua_S,"GL_LIGHT2",GL_LIGHT2);
  tolua_constant(tolua_S,"GL_LIGHT3",GL_LIGHT3);
  tolua_constant(tolua_S,"GL_LIGHT4",GL_LIGHT4);
  tolua_constant(tolua_S,"GL_LIGHT5",GL_LIGHT5);
  tolua_constant(tolua_S,"GL_LIGHT6",GL_LIGHT6);
  tolua_constant(tolua_S,"GL_LIGHT7",GL_LIGHT7);
  tolua_constant(tolua_S,"GL_SPOT_EXPONENT",GL_SPOT_EXPONENT);
  tolua_constant(tolua_S,"GL_SPOT_CUTOFF",GL_SPOT_CUTOFF);
  tolua_constant(tolua_S,"GL_CONSTANT_ATTENUATION",GL_CONSTANT_ATTENUATION);
  tolua_constant(tolua_S,"GL_LINEAR_ATTENUATION",GL_LINEAR_ATTENUATION);
  tolua_constant(tolua_S,"GL_QUADRATIC_ATTENUATION",GL_QUADRATIC_ATTENUATION);
  tolua_constant(tolua_S,"GL_AMBIENT",GL_AMBIENT);
  tolua_constant(tolua_S,"GL_DIFFUSE",GL_DIFFUSE);
  tolua_constant(tolua_S,"GL_SPECULAR",GL_SPECULAR);
  tolua_constant(tolua_S,"GL_SHININESS",GL_SHININESS);
  tolua_constant(tolua_S,"GL_EMISSION",GL_EMISSION);
  tolua_constant(tolua_S,"GL_POSITION",GL_POSITION);
  tolua_constant(tolua_S,"GL_SPOT_DIRECTION",GL_SPOT_DIRECTION);
  tolua_constant(tolua_S,"GL_AMBIENT_AND_DIFFUSE",GL_AMBIENT_AND_DIFFUSE);
  tolua_constant(tolua_S,"GL_COLOR_INDEXES",GL_COLOR_INDEXES);
  tolua_constant(tolua_S,"GL_LIGHT_MODEL_TWO_SIDE",GL_LIGHT_MODEL_TWO_SIDE);
  tolua_constant(tolua_S,"GL_LIGHT_MODEL_LOCAL_VIEWER",GL_LIGHT_MODEL_LOCAL_VIEWER);
  tolua_constant(tolua_S,"GL_LIGHT_MODEL_AMBIENT",GL_LIGHT_MODEL_AMBIENT);
  tolua_constant(tolua_S,"GL_FRONT_AND_BACK",GL_FRONT_AND_BACK);
  tolua_constant(tolua_S,"GL_SHADE_MODEL",GL_SHADE_MODEL);
  tolua_constant(tolua_S,"GL_FLAT",GL_FLAT);
  tolua_constant(tolua_S,"GL_SMOOTH",GL_SMOOTH);
  tolua_constant(tolua_S,"GL_COLOR_MATERIAL",GL_COLOR_MATERIAL);
  tolua_constant(tolua_S,"GL_COLOR_MATERIAL_FACE",GL_COLOR_MATERIAL_FACE);
  tolua_constant(tolua_S,"GL_COLOR_MATERIAL_PARAMETER",GL_COLOR_MATERIAL_PARAMETER);
  tolua_constant(tolua_S,"GL_NORMALIZE",GL_NORMALIZE);
  tolua_constant(tolua_S,"GL_CLIP_PLANE0",GL_CLIP_PLANE0);
  tolua_constant(tolua_S,"GL_CLIP_PLANE1",GL_CLIP_PLANE1);
  tolua_constant(tolua_S,"GL_CLIP_PLANE2",GL_CLIP_PLANE2);
  tolua_constant(tolua_S,"GL_CLIP_PLANE3",GL_CLIP_PLANE3);
  tolua_constant(tolua_S,"GL_CLIP_PLANE4",GL_CLIP_PLANE4);
  tolua_constant(tolua_S,"GL_CLIP_PLANE5",GL_CLIP_PLANE5);
  tolua_constant(tolua_S,"GL_ACCUM_RED_BITS",GL_ACCUM_RED_BITS);
  tolua_constant(tolua_S,"GL_ACCUM_GREEN_BITS",GL_ACCUM_GREEN_BITS);
  tolua_constant(tolua_S,"GL_ACCUM_BLUE_BITS",GL_ACCUM_BLUE_BITS);
  tolua_constant(tolua_S,"GL_ACCUM_ALPHA_BITS",GL_ACCUM_ALPHA_BITS);
  tolua_constant(tolua_S,"GL_ACCUM_CLEAR_VALUE",GL_ACCUM_CLEAR_VALUE);
  tolua_constant(tolua_S,"GL_ACCUM",GL_ACCUM);
  tolua_constant(tolua_S,"GL_ADD",GL_ADD);
  tolua_constant(tolua_S,"GL_LOAD",GL_LOAD);
  tolua_constant(tolua_S,"GL_MULT",GL_MULT);
  tolua_constant(tolua_S,"GL_RETURN",GL_RETURN);
  tolua_constant(tolua_S,"GL_ALPHA_TEST",GL_ALPHA_TEST);
  tolua_constant(tolua_S,"GL_ALPHA_TEST_REF",GL_ALPHA_TEST_REF);
  tolua_constant(tolua_S,"GL_ALPHA_TEST_FUNC",GL_ALPHA_TEST_FUNC);
  tolua_constant(tolua_S,"GL_BLEND",GL_BLEND);
  tolua_constant(tolua_S,"GL_BLEND_SRC",GL_BLEND_SRC);
  tolua_constant(tolua_S,"GL_BLEND_DST",GL_BLEND_DST);
  tolua_constant(tolua_S,"GL_ZERO",GL_ZERO);
  tolua_constant(tolua_S,"GL_ONE",GL_ONE);
  tolua_constant(tolua_S,"GL_SRC_COLOR",GL_SRC_COLOR);
  tolua_constant(tolua_S,"GL_ONE_MINUS_SRC_COLOR",GL_ONE_MINUS_SRC_COLOR);
  tolua_constant(tolua_S,"GL_DST_COLOR",GL_DST_COLOR);
  tolua_constant(tolua_S,"GL_ONE_MINUS_DST_COLOR",GL_ONE_MINUS_DST_COLOR);
  tolua_constant(tolua_S,"GL_SRC_ALPHA",GL_SRC_ALPHA);
  tolua_constant(tolua_S,"GL_ONE_MINUS_SRC_ALPHA",GL_ONE_MINUS_SRC_ALPHA);
  tolua_constant(tolua_S,"GL_DST_ALPHA",GL_DST_ALPHA);
  tolua_constant(tolua_S,"GL_ONE_MINUS_DST_ALPHA",GL_ONE_MINUS_DST_ALPHA);
  tolua_constant(tolua_S,"GL_SRC_ALPHA_SATURATE",GL_SRC_ALPHA_SATURATE);
  tolua_constant(tolua_S,"GL_FEEDBACK",GL_FEEDBACK);
  tolua_constant(tolua_S,"GL_RENDER",GL_RENDER);
  tolua_constant(tolua_S,"GL_SELECT",GL_SELECT);
  tolua_constant(tolua_S,"GL_2D",GL_2D);
  tolua_constant(tolua_S,"GL_3D",GL_3D);
  tolua_constant(tolua_S,"GL_3D_COLOR",GL_3D_COLOR);
  tolua_constant(tolua_S,"GL_3D_COLOR_TEXTURE",GL_3D_COLOR_TEXTURE);
  tolua_constant(tolua_S,"GL_4D_COLOR_TEXTURE",GL_4D_COLOR_TEXTURE);
  tolua_constant(tolua_S,"GL_POINT_TOKEN",GL_POINT_TOKEN);
  tolua_constant(tolua_S,"GL_LINE_TOKEN",GL_LINE_TOKEN);
  tolua_constant(tolua_S,"GL_LINE_RESET_TOKEN",GL_LINE_RESET_TOKEN);
  tolua_constant(tolua_S,"GL_POLYGON_TOKEN",GL_POLYGON_TOKEN);
  tolua_constant(tolua_S,"GL_BITMAP_TOKEN",GL_BITMAP_TOKEN);
  tolua_constant(tolua_S,"GL_DRAW_PIXEL_TOKEN",GL_DRAW_PIXEL_TOKEN);
  tolua_constant(tolua_S,"GL_COPY_PIXEL_TOKEN",GL_COPY_PIXEL_TOKEN);
  tolua_constant(tolua_S,"GL_PASS_THROUGH_TOKEN",GL_PASS_THROUGH_TOKEN);
  tolua_constant(tolua_S,"GL_FOG",GL_FOG);
  tolua_constant(tolua_S,"GL_FOG_MODE",GL_FOG_MODE);
  tolua_constant(tolua_S,"GL_FOG_DENSITY",GL_FOG_DENSITY);
  tolua_constant(tolua_S,"GL_FOG_COLOR",GL_FOG_COLOR);
  tolua_constant(tolua_S,"GL_FOG_INDEX",GL_FOG_INDEX);
  tolua_constant(tolua_S,"GL_FOG_START",GL_FOG_START);
  tolua_constant(tolua_S,"GL_FOG_END",GL_FOG_END);
  tolua_constant(tolua_S,"GL_LINEAR",GL_LINEAR);
  tolua_constant(tolua_S,"GL_EXP",GL_EXP);
  tolua_constant(tolua_S,"GL_EXP2",GL_EXP2);
  tolua_constant(tolua_S,"GL_LOGIC_OP",GL_LOGIC_OP);
  tolua_constant(tolua_S,"GL_LOGIC_OP_MODE",GL_LOGIC_OP_MODE);
  tolua_constant(tolua_S,"GL_CLEAR",GL_CLEAR);
  tolua_constant(tolua_S,"GL_SET",GL_SET);
  tolua_constant(tolua_S,"GL_COPY",GL_COPY);
  tolua_constant(tolua_S,"GL_COPY_INVERTED",GL_COPY_INVERTED);
  tolua_constant(tolua_S,"GL_NOOP",GL_NOOP);
  tolua_constant(tolua_S,"GL_INVERT",GL_INVERT);
  tolua_constant(tolua_S,"GL_AND",GL_AND);
  tolua_constant(tolua_S,"GL_NAND",GL_NAND);
  tolua_constant(tolua_S,"GL_OR",GL_OR);
  tolua_constant(tolua_S,"GL_NOR",GL_NOR);
  tolua_constant(tolua_S,"GL_XOR",GL_XOR);
  tolua_constant(tolua_S,"GL_EQUIV",GL_EQUIV);
  tolua_constant(tolua_S,"GL_AND_REVERSE",GL_AND_REVERSE);
  tolua_constant(tolua_S,"GL_AND_INVERTED",GL_AND_INVERTED);
  tolua_constant(tolua_S,"GL_OR_REVERSE",GL_OR_REVERSE);
  tolua_constant(tolua_S,"GL_OR_INVERTED",GL_OR_INVERTED);
  tolua_constant(tolua_S,"GL_STENCIL_TEST",GL_STENCIL_TEST);
  tolua_constant(tolua_S,"GL_STENCIL_WRITEMASK",GL_STENCIL_WRITEMASK);
  tolua_constant(tolua_S,"GL_STENCIL_BITS",GL_STENCIL_BITS);
  tolua_constant(tolua_S,"GL_STENCIL_FUNC",GL_STENCIL_FUNC);
  tolua_constant(tolua_S,"GL_STENCIL_VALUE_MASK",GL_STENCIL_VALUE_MASK);
  tolua_constant(tolua_S,"GL_STENCIL_REF",GL_STENCIL_REF);
  tolua_constant(tolua_S,"GL_STENCIL_FAIL",GL_STENCIL_FAIL);
  tolua_constant(tolua_S,"GL_STENCIL_PASS_DEPTH_PASS",GL_STENCIL_PASS_DEPTH_PASS);
  tolua_constant(tolua_S,"GL_STENCIL_PASS_DEPTH_FAIL",GL_STENCIL_PASS_DEPTH_FAIL);
  tolua_constant(tolua_S,"GL_STENCIL_CLEAR_VALUE",GL_STENCIL_CLEAR_VALUE);
  tolua_constant(tolua_S,"GL_STENCIL_INDEX",GL_STENCIL_INDEX);
  tolua_constant(tolua_S,"GL_KEEP",GL_KEEP);
  tolua_constant(tolua_S,"GL_REPLACE",GL_REPLACE);
  tolua_constant(tolua_S,"GL_INCR",GL_INCR);
  tolua_constant(tolua_S,"GL_DECR",GL_DECR);
  tolua_constant(tolua_S,"GL_NONE",GL_NONE);
  tolua_constant(tolua_S,"GL_LEFT",GL_LEFT);
  tolua_constant(tolua_S,"GL_RIGHT",GL_RIGHT);
  tolua_constant(tolua_S,"GL_FRONT",GL_FRONT);
  tolua_constant(tolua_S,"GL_BACK",GL_BACK);
  tolua_constant(tolua_S,"GL_FRONT_AND_BACK",GL_FRONT_AND_BACK);
  tolua_constant(tolua_S,"GL_FRONT_LEFT",GL_FRONT_LEFT);
  tolua_constant(tolua_S,"GL_FRONT_RIGHT",GL_FRONT_RIGHT);
  tolua_constant(tolua_S,"GL_BACK_LEFT",GL_BACK_LEFT);
  tolua_constant(tolua_S,"GL_BACK_RIGHT",GL_BACK_RIGHT);
  tolua_constant(tolua_S,"GL_AUX0",GL_AUX0);
  tolua_constant(tolua_S,"GL_AUX1",GL_AUX1);
  tolua_constant(tolua_S,"GL_AUX2",GL_AUX2);
  tolua_constant(tolua_S,"GL_AUX3",GL_AUX3);
  tolua_constant(tolua_S,"GL_COLOR_INDEX",GL_COLOR_INDEX);
  tolua_constant(tolua_S,"GL_RED",GL_RED);
  tolua_constant(tolua_S,"GL_GREEN",GL_GREEN);
  tolua_constant(tolua_S,"GL_BLUE",GL_BLUE);
  tolua_constant(tolua_S,"GL_ALPHA",GL_ALPHA);
  tolua_constant(tolua_S,"GL_LUMINANCE",GL_LUMINANCE);
  tolua_constant(tolua_S,"GL_LUMINANCE_ALPHA",GL_LUMINANCE_ALPHA);
  tolua_constant(tolua_S,"GL_ALPHA_BITS",GL_ALPHA_BITS);
  tolua_constant(tolua_S,"GL_RED_BITS",GL_RED_BITS);
  tolua_constant(tolua_S,"GL_GREEN_BITS",GL_GREEN_BITS);
  tolua_constant(tolua_S,"GL_BLUE_BITS",GL_BLUE_BITS);
  tolua_constant(tolua_S,"GL_INDEX_BITS",GL_INDEX_BITS);
  tolua_constant(tolua_S,"GL_SUBPIXEL_BITS",GL_SUBPIXEL_BITS);
  tolua_constant(tolua_S,"GL_AUX_BUFFERS",GL_AUX_BUFFERS);
  tolua_constant(tolua_S,"GL_READ_BUFFER",GL_READ_BUFFER);
  tolua_constant(tolua_S,"GL_DRAW_BUFFER",GL_DRAW_BUFFER);
  tolua_constant(tolua_S,"GL_DOUBLEBUFFER",GL_DOUBLEBUFFER);
  tolua_constant(tolua_S,"GL_STEREO",GL_STEREO);
  tolua_constant(tolua_S,"GL_BITMAP",GL_BITMAP);
  tolua_constant(tolua_S,"GL_COLOR",GL_COLOR);
  tolua_constant(tolua_S,"GL_DEPTH",GL_DEPTH);
  tolua_constant(tolua_S,"GL_STENCIL",GL_STENCIL);
  tolua_constant(tolua_S,"GL_DITHER",GL_DITHER);
  tolua_constant(tolua_S,"GL_RGB",GL_RGB);
  tolua_constant(tolua_S,"GL_RGBA",GL_RGBA);
  tolua_constant(tolua_S,"GL_MAX_LIST_NESTING",GL_MAX_LIST_NESTING);
  tolua_constant(tolua_S,"GL_MAX_ATTRIB_STACK_DEPTH",GL_MAX_ATTRIB_STACK_DEPTH);
  tolua_constant(tolua_S,"GL_MAX_MODELVIEW_STACK_DEPTH",GL_MAX_MODELVIEW_STACK_DEPTH);
  tolua_constant(tolua_S,"GL_MAX_NAME_STACK_DEPTH",GL_MAX_NAME_STACK_DEPTH);
  tolua_constant(tolua_S,"GL_MAX_PROJECTION_STACK_DEPTH",GL_MAX_PROJECTION_STACK_DEPTH);
  tolua_constant(tolua_S,"GL_MAX_TEXTURE_STACK_DEPTH",GL_MAX_TEXTURE_STACK_DEPTH);
  tolua_constant(tolua_S,"GL_MAX_EVAL_ORDER",GL_MAX_EVAL_ORDER);
  tolua_constant(tolua_S,"GL_MAX_LIGHTS",GL_MAX_LIGHTS);
  tolua_constant(tolua_S,"GL_MAX_CLIP_PLANES",GL_MAX_CLIP_PLANES);
  tolua_constant(tolua_S,"GL_MAX_TEXTURE_SIZE",GL_MAX_TEXTURE_SIZE);
  tolua_constant(tolua_S,"GL_MAX_PIXEL_MAP_TABLE",GL_MAX_PIXEL_MAP_TABLE);
  tolua_constant(tolua_S,"GL_MAX_VIEWPORT_DIMS",GL_MAX_VIEWPORT_DIMS);
  tolua_constant(tolua_S,"GL_ATTRIB_STACK_DEPTH",GL_ATTRIB_STACK_DEPTH);
  tolua_constant(tolua_S,"GL_COLOR_CLEAR_VALUE",GL_COLOR_CLEAR_VALUE);
  tolua_constant(tolua_S,"GL_COLOR_WRITEMASK",GL_COLOR_WRITEMASK);
  tolua_constant(tolua_S,"GL_CURRENT_INDEX",GL_CURRENT_INDEX);
  tolua_constant(tolua_S,"GL_CURRENT_COLOR",GL_CURRENT_COLOR);
  tolua_constant(tolua_S,"GL_CURRENT_NORMAL",GL_CURRENT_NORMAL);
  tolua_constant(tolua_S,"GL_CURRENT_RASTER_COLOR",GL_CURRENT_RASTER_COLOR);
  tolua_constant(tolua_S,"GL_CURRENT_RASTER_DISTANCE",GL_CURRENT_RASTER_DISTANCE);
  tolua_constant(tolua_S,"GL_CURRENT_RASTER_INDEX",GL_CURRENT_RASTER_INDEX);
  tolua_constant(tolua_S,"GL_CURRENT_RASTER_POSITION",GL_CURRENT_RASTER_POSITION);
  tolua_constant(tolua_S,"GL_CURRENT_RASTER_TEXTURE_COORDS",GL_CURRENT_RASTER_TEXTURE_COORDS);
  tolua_constant(tolua_S,"GL_CURRENT_RASTER_POSITION_VALID",GL_CURRENT_RASTER_POSITION_VALID);
  tolua_constant(tolua_S,"GL_CURRENT_TEXTURE_COORDS",GL_CURRENT_TEXTURE_COORDS);
  tolua_constant(tolua_S,"GL_INDEX_CLEAR_VALUE",GL_INDEX_CLEAR_VALUE);
  tolua_constant(tolua_S,"GL_INDEX_MODE",GL_INDEX_MODE);
  tolua_constant(tolua_S,"GL_INDEX_WRITEMASK",GL_INDEX_WRITEMASK);
  tolua_constant(tolua_S,"GL_MODELVIEW_MATRIX",GL_MODELVIEW_MATRIX);
  tolua_constant(tolua_S,"GL_MODELVIEW_STACK_DEPTH",GL_MODELVIEW_STACK_DEPTH);
  tolua_constant(tolua_S,"GL_NAME_STACK_DEPTH",GL_NAME_STACK_DEPTH);
  tolua_constant(tolua_S,"GL_PROJECTION_MATRIX",GL_PROJECTION_MATRIX);
  tolua_constant(tolua_S,"GL_PROJECTION_STACK_DEPTH",GL_PROJECTION_STACK_DEPTH);
  tolua_constant(tolua_S,"GL_RENDER_MODE",GL_RENDER_MODE);
  tolua_constant(tolua_S,"GL_RGBA_MODE",GL_RGBA_MODE);
  tolua_constant(tolua_S,"GL_TEXTURE_MATRIX",GL_TEXTURE_MATRIX);
  tolua_constant(tolua_S,"GL_TEXTURE_STACK_DEPTH",GL_TEXTURE_STACK_DEPTH);
  tolua_constant(tolua_S,"GL_VIEWPORT",GL_VIEWPORT);
  tolua_constant(tolua_S,"GL_AUTO_NORMAL",GL_AUTO_NORMAL);
  tolua_constant(tolua_S,"GL_MAP1_COLOR_4",GL_MAP1_COLOR_4);
  tolua_constant(tolua_S,"GL_MAP1_GRID_DOMAIN",GL_MAP1_GRID_DOMAIN);
  tolua_constant(tolua_S,"GL_MAP1_GRID_SEGMENTS",GL_MAP1_GRID_SEGMENTS);
  tolua_constant(tolua_S,"GL_MAP1_INDEX",GL_MAP1_INDEX);
  tolua_constant(tolua_S,"GL_MAP1_NORMAL",GL_MAP1_NORMAL);
  tolua_constant(tolua_S,"GL_MAP1_TEXTURE_COORD_1",GL_MAP1_TEXTURE_COORD_1);
  tolua_constant(tolua_S,"GL_MAP1_TEXTURE_COORD_2",GL_MAP1_TEXTURE_COORD_2);
  tolua_constant(tolua_S,"GL_MAP1_TEXTURE_COORD_3",GL_MAP1_TEXTURE_COORD_3);
  tolua_constant(tolua_S,"GL_MAP1_TEXTURE_COORD_4",GL_MAP1_TEXTURE_COORD_4);
  tolua_constant(tolua_S,"GL_MAP1_VERTEX_3",GL_MAP1_VERTEX_3);
  tolua_constant(tolua_S,"GL_MAP1_VERTEX_4",GL_MAP1_VERTEX_4);
  tolua_constant(tolua_S,"GL_MAP2_COLOR_4",GL_MAP2_COLOR_4);
  tolua_constant(tolua_S,"GL_MAP2_GRID_DOMAIN",GL_MAP2_GRID_DOMAIN);
  tolua_constant(tolua_S,"GL_MAP2_GRID_SEGMENTS",GL_MAP2_GRID_SEGMENTS);
  tolua_constant(tolua_S,"GL_MAP2_INDEX",GL_MAP2_INDEX);
  tolua_constant(tolua_S,"GL_MAP2_NORMAL",GL_MAP2_NORMAL);
  tolua_constant(tolua_S,"GL_MAP2_TEXTURE_COORD_1",GL_MAP2_TEXTURE_COORD_1);
  tolua_constant(tolua_S,"GL_MAP2_TEXTURE_COORD_2",GL_MAP2_TEXTURE_COORD_2);
  tolua_constant(tolua_S,"GL_MAP2_TEXTURE_COORD_3",GL_MAP2_TEXTURE_COORD_3);
  tolua_constant(tolua_S,"GL_MAP2_TEXTURE_COORD_4",GL_MAP2_TEXTURE_COORD_4);
  tolua_constant(tolua_S,"GL_MAP2_VERTEX_3",GL_MAP2_VERTEX_3);
  tolua_constant(tolua_S,"GL_MAP2_VERTEX_4",GL_MAP2_VERTEX_4);
  tolua_constant(tolua_S,"GL_COEFF",GL_COEFF);
  tolua_constant(tolua_S,"GL_DOMAIN",GL_DOMAIN);
  tolua_constant(tolua_S,"GL_ORDER",GL_ORDER);
  tolua_constant(tolua_S,"GL_FOG_HINT",GL_FOG_HINT);
  tolua_constant(tolua_S,"GL_LINE_SMOOTH_HINT",GL_LINE_SMOOTH_HINT);
  tolua_constant(tolua_S,"GL_PERSPECTIVE_CORRECTION_HINT",GL_PERSPECTIVE_CORRECTION_HINT);
  tolua_constant(tolua_S,"GL_POINT_SMOOTH_HINT",GL_POINT_SMOOTH_HINT);
  tolua_constant(tolua_S,"GL_POLYGON_SMOOTH_HINT",GL_POLYGON_SMOOTH_HINT);
  tolua_constant(tolua_S,"GL_DONT_CARE",GL_DONT_CARE);
  tolua_constant(tolua_S,"GL_FASTEST",GL_FASTEST);
  tolua_constant(tolua_S,"GL_NICEST",GL_NICEST);
  tolua_constant(tolua_S,"GL_SCISSOR_TEST",GL_SCISSOR_TEST);
  tolua_constant(tolua_S,"GL_SCISSOR_BOX",GL_SCISSOR_BOX);
  tolua_constant(tolua_S,"GL_MAP_COLOR",GL_MAP_COLOR);
  tolua_constant(tolua_S,"GL_MAP_STENCIL",GL_MAP_STENCIL);
  tolua_constant(tolua_S,"GL_INDEX_SHIFT",GL_INDEX_SHIFT);
  tolua_constant(tolua_S,"GL_INDEX_OFFSET",GL_INDEX_OFFSET);
  tolua_constant(tolua_S,"GL_RED_SCALE",GL_RED_SCALE);
  tolua_constant(tolua_S,"GL_RED_BIAS",GL_RED_BIAS);
  tolua_constant(tolua_S,"GL_GREEN_SCALE",GL_GREEN_SCALE);
  tolua_constant(tolua_S,"GL_GREEN_BIAS",GL_GREEN_BIAS);
  tolua_constant(tolua_S,"GL_BLUE_SCALE",GL_BLUE_SCALE);
  tolua_constant(tolua_S,"GL_BLUE_BIAS",GL_BLUE_BIAS);
  tolua_constant(tolua_S,"GL_ALPHA_SCALE",GL_ALPHA_SCALE);
  tolua_constant(tolua_S,"GL_ALPHA_BIAS",GL_ALPHA_BIAS);
  tolua_constant(tolua_S,"GL_DEPTH_SCALE",GL_DEPTH_SCALE);
  tolua_constant(tolua_S,"GL_DEPTH_BIAS",GL_DEPTH_BIAS);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_S_TO_S_SIZE",GL_PIXEL_MAP_S_TO_S_SIZE);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_I_TO_I_SIZE",GL_PIXEL_MAP_I_TO_I_SIZE);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_I_TO_R_SIZE",GL_PIXEL_MAP_I_TO_R_SIZE);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_I_TO_G_SIZE",GL_PIXEL_MAP_I_TO_G_SIZE);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_I_TO_B_SIZE",GL_PIXEL_MAP_I_TO_B_SIZE);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_I_TO_A_SIZE",GL_PIXEL_MAP_I_TO_A_SIZE);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_R_TO_R_SIZE",GL_PIXEL_MAP_R_TO_R_SIZE);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_G_TO_G_SIZE",GL_PIXEL_MAP_G_TO_G_SIZE);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_B_TO_B_SIZE",GL_PIXEL_MAP_B_TO_B_SIZE);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_A_TO_A_SIZE",GL_PIXEL_MAP_A_TO_A_SIZE);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_S_TO_S",GL_PIXEL_MAP_S_TO_S);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_I_TO_I",GL_PIXEL_MAP_I_TO_I);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_I_TO_R",GL_PIXEL_MAP_I_TO_R);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_I_TO_G",GL_PIXEL_MAP_I_TO_G);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_I_TO_B",GL_PIXEL_MAP_I_TO_B);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_I_TO_A",GL_PIXEL_MAP_I_TO_A);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_R_TO_R",GL_PIXEL_MAP_R_TO_R);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_G_TO_G",GL_PIXEL_MAP_G_TO_G);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_B_TO_B",GL_PIXEL_MAP_B_TO_B);
  tolua_constant(tolua_S,"GL_PIXEL_MAP_A_TO_A",GL_PIXEL_MAP_A_TO_A);
  tolua_constant(tolua_S,"GL_PACK_ALIGNMENT",GL_PACK_ALIGNMENT);
  tolua_constant(tolua_S,"GL_PACK_LSB_FIRST",GL_PACK_LSB_FIRST);
  tolua_constant(tolua_S,"GL_PACK_ROW_LENGTH",GL_PACK_ROW_LENGTH);
  tolua_constant(tolua_S,"GL_PACK_SKIP_PIXELS",GL_PACK_SKIP_PIXELS);
  tolua_constant(tolua_S,"GL_PACK_SKIP_ROWS",GL_PACK_SKIP_ROWS);
  tolua_constant(tolua_S,"GL_PACK_SWAP_BYTES",GL_PACK_SWAP_BYTES);
  tolua_constant(tolua_S,"GL_UNPACK_ALIGNMENT",GL_UNPACK_ALIGNMENT);
  tolua_constant(tolua_S,"GL_UNPACK_LSB_FIRST",GL_UNPACK_LSB_FIRST);
  tolua_constant(tolua_S,"GL_UNPACK_ROW_LENGTH",GL_UNPACK_ROW_LENGTH);
  tolua_constant(tolua_S,"GL_UNPACK_SKIP_PIXELS",GL_UNPACK_SKIP_PIXELS);
  tolua_constant(tolua_S,"GL_UNPACK_SKIP_ROWS",GL_UNPACK_SKIP_ROWS);
  tolua_constant(tolua_S,"GL_UNPACK_SWAP_BYTES",GL_UNPACK_SWAP_BYTES);
  tolua_constant(tolua_S,"GL_ZOOM_X",GL_ZOOM_X);
  tolua_constant(tolua_S,"GL_ZOOM_Y",GL_ZOOM_Y);
  tolua_constant(tolua_S,"GL_TEXTURE_ENV",GL_TEXTURE_ENV);
  tolua_constant(tolua_S,"GL_TEXTURE_ENV_MODE",GL_TEXTURE_ENV_MODE);
  tolua_constant(tolua_S,"GL_TEXTURE_1D",GL_TEXTURE_1D);
  tolua_constant(tolua_S,"GL_TEXTURE_2D",GL_TEXTURE_2D);
  tolua_constant(tolua_S,"GL_TEXTURE_WRAP_S",GL_TEXTURE_WRAP_S);
  tolua_constant(tolua_S,"GL_TEXTURE_WRAP_T",GL_TEXTURE_WRAP_T);
  tolua_constant(tolua_S,"GL_TEXTURE_MAG_FILTER",GL_TEXTURE_MAG_FILTER);
  tolua_constant(tolua_S,"GL_TEXTURE_MIN_FILTER",GL_TEXTURE_MIN_FILTER);
  tolua_constant(tolua_S,"GL_TEXTURE_ENV_COLOR",GL_TEXTURE_ENV_COLOR);
  tolua_constant(tolua_S,"GL_TEXTURE_GEN_S",GL_TEXTURE_GEN_S);
  tolua_constant(tolua_S,"GL_TEXTURE_GEN_T",GL_TEXTURE_GEN_T);
  tolua_constant(tolua_S,"GL_TEXTURE_GEN_MODE",GL_TEXTURE_GEN_MODE);
  tolua_constant(tolua_S,"GL_TEXTURE_BORDER_COLOR",GL_TEXTURE_BORDER_COLOR);
  tolua_constant(tolua_S,"GL_TEXTURE_WIDTH",GL_TEXTURE_WIDTH);
  tolua_constant(tolua_S,"GL_TEXTURE_HEIGHT",GL_TEXTURE_HEIGHT);
  tolua_constant(tolua_S,"GL_TEXTURE_BORDER",GL_TEXTURE_BORDER);
  tolua_constant(tolua_S,"GL_TEXTURE_COMPONENTS",GL_TEXTURE_COMPONENTS);
  tolua_constant(tolua_S,"GL_NEAREST_MIPMAP_NEAREST",GL_NEAREST_MIPMAP_NEAREST);
  tolua_constant(tolua_S,"GL_NEAREST_MIPMAP_LINEAR",GL_NEAREST_MIPMAP_LINEAR);
  tolua_constant(tolua_S,"GL_LINEAR_MIPMAP_NEAREST",GL_LINEAR_MIPMAP_NEAREST);
  tolua_constant(tolua_S,"GL_LINEAR_MIPMAP_LINEAR",GL_LINEAR_MIPMAP_LINEAR);
  tolua_constant(tolua_S,"GL_OBJECT_LINEAR",GL_OBJECT_LINEAR);
  tolua_constant(tolua_S,"GL_OBJECT_PLANE",GL_OBJECT_PLANE);
  tolua_constant(tolua_S,"GL_EYE_LINEAR",GL_EYE_LINEAR);
  tolua_constant(tolua_S,"GL_EYE_PLANE",GL_EYE_PLANE);
  tolua_constant(tolua_S,"GL_SPHERE_MAP",GL_SPHERE_MAP);
  tolua_constant(tolua_S,"GL_DECAL",GL_DECAL);
  tolua_constant(tolua_S,"GL_MODULATE",GL_MODULATE);
  tolua_constant(tolua_S,"GL_NEAREST",GL_NEAREST);
  tolua_constant(tolua_S,"GL_REPEAT",GL_REPEAT);
  tolua_constant(tolua_S,"GL_CLAMP",GL_CLAMP);
  tolua_constant(tolua_S,"GL_S",GL_S);
  tolua_constant(tolua_S,"GL_T",GL_T);
  tolua_constant(tolua_S,"GL_R",GL_R);
  tolua_constant(tolua_S,"GL_Q",GL_Q);
  tolua_constant(tolua_S,"GL_TEXTURE_GEN_R",GL_TEXTURE_GEN_R);
  tolua_constant(tolua_S,"GL_TEXTURE_GEN_Q",GL_TEXTURE_GEN_Q);
  tolua_constant(tolua_S,"GL_VENDOR",GL_VENDOR);
  tolua_constant(tolua_S,"GL_RENDERER",GL_RENDERER);
  tolua_constant(tolua_S,"GL_VERSION",GL_VERSION);
  tolua_constant(tolua_S,"GL_EXTENSIONS",GL_EXTENSIONS);
  tolua_constant(tolua_S,"GL_INVALID_VALUE",GL_INVALID_VALUE);
  tolua_constant(tolua_S,"GL_INVALID_ENUM",GL_INVALID_ENUM);
  tolua_constant(tolua_S,"GL_INVALID_OPERATION",GL_INVALID_OPERATION);
  tolua_constant(tolua_S,"GL_STACK_OVERFLOW",GL_STACK_OVERFLOW);
  tolua_constant(tolua_S,"GL_STACK_UNDERFLOW",GL_STACK_UNDERFLOW);
  tolua_constant(tolua_S,"GL_OUT_OF_MEMORY",GL_OUT_OF_MEMORY);
  tolua_constant(tolua_S,"GL_NO_ERROR",GL_NO_ERROR);
  tolua_constant(tolua_S,"GL_CURRENT_BIT",GL_CURRENT_BIT);
  tolua_constant(tolua_S,"GL_POINT_BIT",GL_POINT_BIT);
  tolua_constant(tolua_S,"GL_LINE_BIT",GL_LINE_BIT);
  tolua_constant(tolua_S,"GL_POLYGON_BIT",GL_POLYGON_BIT);
  tolua_constant(tolua_S,"GL_POLYGON_STIPPLE_BIT",GL_POLYGON_STIPPLE_BIT);
  tolua_constant(tolua_S,"GL_PIXEL_MODE_BIT",GL_PIXEL_MODE_BIT);
  tolua_constant(tolua_S,"GL_LIGHTING_BIT",GL_LIGHTING_BIT);
  tolua_constant(tolua_S,"GL_FOG_BIT",GL_FOG_BIT);
  tolua_constant(tolua_S,"GL_DEPTH_BUFFER_BIT",GL_DEPTH_BUFFER_BIT);
  tolua_constant(tolua_S,"GL_ACCUM_BUFFER_BIT",GL_ACCUM_BUFFER_BIT);
  tolua_constant(tolua_S,"GL_STENCIL_BUFFER_BIT",GL_STENCIL_BUFFER_BIT);
  tolua_constant(tolua_S,"GL_VIEWPORT_BIT",GL_VIEWPORT_BIT);
  tolua_constant(tolua_S,"GL_TRANSFORM_BIT",GL_TRANSFORM_BIT);
  tolua_constant(tolua_S,"GL_ENABLE_BIT",GL_ENABLE_BIT);
  tolua_constant(tolua_S,"GL_COLOR_BUFFER_BIT",GL_COLOR_BUFFER_BIT);
  tolua_constant(tolua_S,"GL_HINT_BIT",GL_HINT_BIT);
  tolua_constant(tolua_S,"GL_EVAL_BIT",GL_EVAL_BIT);
  tolua_constant(tolua_S,"GL_LIST_BIT",GL_LIST_BIT);
  tolua_constant(tolua_S,"GL_TEXTURE_BIT",GL_TEXTURE_BIT);
  tolua_constant(tolua_S,"GL_SCISSOR_BIT",GL_SCISSOR_BIT);
  tolua_constant(tolua_S,"GL_ALL_ATTRIB_BITS",GL_ALL_ATTRIB_BITS);
  tolua_function(tolua_S,"glClearIndex",tolua_gl_glClearIndex00);
  tolua_function(tolua_S,"glClearColor",tolua_gl_glClearColor00);
  tolua_function(tolua_S,"glClear",tolua_gl_glClear00);
  tolua_function(tolua_S,"glIndexMask",tolua_gl_glIndexMask00);
  tolua_function(tolua_S,"glColorMask",tolua_gl_glColorMask00);
  tolua_function(tolua_S,"glAlphaFunc",tolua_gl_glAlphaFunc00);
  tolua_function(tolua_S,"glBlendFunc",tolua_gl_glBlendFunc00);
  tolua_function(tolua_S,"glLogicOp",tolua_gl_glLogicOp00);
  tolua_function(tolua_S,"glCullFace",tolua_gl_glCullFace00);
  tolua_function(tolua_S,"glFrontFace",tolua_gl_glFrontFace00);
  tolua_function(tolua_S,"glPointSize",tolua_gl_glPointSize00);
  tolua_function(tolua_S,"glLineWidth",tolua_gl_glLineWidth00);
  tolua_function(tolua_S,"glLineStipple",tolua_gl_glLineStipple00);
  tolua_function(tolua_S,"glPolygonMode",tolua_gl_glPolygonMode00);
  tolua_function(tolua_S,"glPolygonStipple",tolua_gl_glPolygonStipple00);
  tolua_function(tolua_S,"glGetPolygonStipple",tolua_gl_glGetPolygonStipple00);
  tolua_function(tolua_S,"glEdgeFlag",tolua_gl_glEdgeFlag00);
  tolua_function(tolua_S,"glScissor",tolua_gl_glScissor00);
  tolua_function(tolua_S,"glClipPlane",tolua_gl_glClipPlane00);
  tolua_function(tolua_S,"glGetClipPlane",tolua_gl_glGetClipPlane00);
  tolua_function(tolua_S,"glDrawBuffer",tolua_gl_glDrawBuffer00);
  tolua_function(tolua_S,"glReadBuffer",tolua_gl_glReadBuffer00);
  tolua_function(tolua_S,"glEnable",tolua_gl_glEnable00);
  tolua_function(tolua_S,"glDisable",tolua_gl_glDisable00);
  tolua_function(tolua_S,"glIsEnabled",tolua_gl_glIsEnabled00);
  tolua_function(tolua_S,"glGet",tolua_gl_glGet00);
  tolua_function(tolua_S,"glGet",tolua_gl_glGet01);
  tolua_function(tolua_S,"glPushAttrib",tolua_gl_glPushAttrib00);
  tolua_function(tolua_S,"glPopAttrib",tolua_gl_glPopAttrib00);
  tolua_function(tolua_S,"glRenderMode",tolua_gl_glRenderMode00);
  tolua_function(tolua_S,"glGetError",tolua_gl_glGetError00);
  tolua_function(tolua_S,"glGetString",tolua_gl_glGetString00);
  tolua_function(tolua_S,"glFinish",tolua_gl_glFinish00);
  tolua_function(tolua_S,"glFlush",tolua_gl_glFlush00);
  tolua_function(tolua_S,"glHint",tolua_gl_glHint00);
  tolua_function(tolua_S,"glClearDepth",tolua_gl_glClearDepth00);
  tolua_function(tolua_S,"glDepthFunc",tolua_gl_glDepthFunc00);
  tolua_function(tolua_S,"glDepthMask",tolua_gl_glDepthMask00);
  tolua_function(tolua_S,"glDepthRange",tolua_gl_glDepthRange00);
  tolua_function(tolua_S,"glClearAccum",tolua_gl_glClearAccum00);
  tolua_function(tolua_S,"glAccum",tolua_gl_glAccum00);
  tolua_function(tolua_S,"glMatrixMode",tolua_gl_glMatrixMode00);
  tolua_function(tolua_S,"glOrtho",tolua_gl_glOrtho00);
  tolua_function(tolua_S,"glFrustum",tolua_gl_glFrustum00);
  tolua_function(tolua_S,"glViewport",tolua_gl_glViewport00);
  tolua_function(tolua_S,"glPushMatrix",tolua_gl_glPushMatrix00);
  tolua_function(tolua_S,"glPopMatrix",tolua_gl_glPopMatrix00);
  tolua_function(tolua_S,"glLoadIdentity",tolua_gl_glLoadIdentity00);
  tolua_function(tolua_S,"glLoadMatrix",tolua_gl_glLoadMatrix00);
  tolua_function(tolua_S,"glMultMatrix",tolua_gl_glMultMatrix00);
  tolua_function(tolua_S,"glRotate",tolua_gl_glRotate00);
  tolua_function(tolua_S,"glScale",tolua_gl_glScale00);
  tolua_function(tolua_S,"glTranslate",tolua_gl_glTranslate00);
  tolua_function(tolua_S,"glIsList",tolua_gl_glIsList00);
  tolua_function(tolua_S,"glDeleteLists",tolua_gl_glDeleteLists00);
  tolua_function(tolua_S,"glGenLists",tolua_gl_glGenLists00);
  tolua_function(tolua_S,"glNewList",tolua_gl_glNewList00);
  tolua_function(tolua_S,"glEndList",tolua_gl_glEndList00);
  tolua_function(tolua_S,"glCallList",tolua_gl_glCallList00);
  tolua_function(tolua_S,"glListBase",tolua_gl_glListBase00);
  tolua_function(tolua_S,"glBegin",tolua_gl_glBegin00);
  tolua_function(tolua_S,"glEnd",tolua_gl_glEnd00);
  tolua_function(tolua_S,"glVertex",tolua_gl_glVertex00);
  tolua_function(tolua_S,"glVertex",tolua_gl_glVertex01);
  tolua_function(tolua_S,"glVertex",tolua_gl_glVertex02);
  tolua_function(tolua_S,"glNormal",tolua_gl_glNormal00);
  tolua_function(tolua_S,"glNormal",tolua_gl_glNormal01);
  tolua_function(tolua_S,"glIndex",tolua_gl_glIndex00);
  tolua_function(tolua_S,"glIndex",tolua_gl_glIndex01);
  tolua_function(tolua_S,"glColor",tolua_gl_glColor00);
  tolua_function(tolua_S,"glColor",tolua_gl_glColor01);
  tolua_function(tolua_S,"glTexCoord",tolua_gl_glTexCoord00);
  tolua_function(tolua_S,"glTexCoord",tolua_gl_glTexCoord01);
  tolua_function(tolua_S,"glTexCoord",tolua_gl_glTexCoord02);
  tolua_function(tolua_S,"glRasterPos",tolua_gl_glRasterPos00);
  tolua_function(tolua_S,"glRasterPos",tolua_gl_glRasterPos01);
  tolua_function(tolua_S,"glRasterPos",tolua_gl_glRasterPos02);
  tolua_function(tolua_S,"glRect",tolua_gl_glRect00);
  tolua_function(tolua_S,"glRect",tolua_gl_glRect01);
  tolua_function(tolua_S,"glShadeModel",tolua_gl_glShadeModel00);
  tolua_function(tolua_S,"glLight",tolua_gl_glLight00);
  tolua_function(tolua_S,"glLight",tolua_gl_glLight01);
  tolua_function(tolua_S,"glGetLight",tolua_gl_glGetLight00);
  tolua_function(tolua_S,"glGetLight",tolua_gl_glGetLight01);
  tolua_function(tolua_S,"glLightModel",tolua_gl_glLightModel00);
  tolua_function(tolua_S,"glLightModel",tolua_gl_glLightModel01);
  tolua_function(tolua_S,"glMaterial",tolua_gl_glMaterial00);
  tolua_function(tolua_S,"glMaterial",tolua_gl_glMaterial01);
  tolua_function(tolua_S,"glGetMaterial",tolua_gl_glGetMaterial00);
  tolua_function(tolua_S,"glGetMaterial",tolua_gl_glGetMaterial01);
  tolua_function(tolua_S,"glColorMaterial",tolua_gl_glColorMaterial00);
  tolua_function(tolua_S,"glPixelZoom",tolua_gl_glPixelZoom00);
  tolua_function(tolua_S,"glPixelStore",tolua_gl_glPixelStore00);
  tolua_function(tolua_S,"glPixelTransfer",tolua_gl_glPixelTransfer00);
  tolua_function(tolua_S,"glPixelMap",tolua_gl_glPixelMap00);
  tolua_function(tolua_S,"glBitmap",tolua_gl_glBitmap00);
  tolua_function(tolua_S,"glReadPixels",tolua_gl_glReadPixels00);
  tolua_function(tolua_S,"glDrawPixels",tolua_gl_glDrawPixels00);
  tolua_function(tolua_S,"glCopyPixels",tolua_gl_glCopyPixels00);
  tolua_function(tolua_S,"glStencilFunc",tolua_gl_glStencilFunc00);
  tolua_function(tolua_S,"glStencilMask",tolua_gl_glStencilMask00);
  tolua_function(tolua_S,"glStencilOp",tolua_gl_glStencilOp00);
  tolua_function(tolua_S,"glClearStencil",tolua_gl_glClearStencil00);
  tolua_function(tolua_S,"glTexGen",tolua_gl_glTexGen00);
  tolua_function(tolua_S,"glTexGen",tolua_gl_glTexGen01);
  tolua_function(tolua_S,"glGetTexGen",tolua_gl_glGetTexGen00);
  tolua_function(tolua_S,"glTexEnv",tolua_gl_glTexEnv00);
  tolua_function(tolua_S,"glTexEnvfv",tolua_gl_glTexEnvfv00);
  tolua_function(tolua_S,"glGetTexEnv",tolua_gl_glGetTexEnv00);
  tolua_function(tolua_S,"glTexParameter",tolua_gl_glTexParameter00);
  tolua_function(tolua_S,"glTexParameter",tolua_gl_glTexParameter01);
  tolua_function(tolua_S,"glGetTexParameter",tolua_gl_glGetTexParameter00);
  tolua_function(tolua_S,"glGetTexLevelParameter",tolua_gl_glGetTexLevelParameter00);
  tolua_function(tolua_S,"glTexImage1D",tolua_gl_glTexImage1D00);
  tolua_function(tolua_S,"glTexImage2D",tolua_gl_glTexImage2D00);
  tolua_function(tolua_S,"glGetTexImage",tolua_gl_glGetTexImage00);
  tolua_function(tolua_S,"glMap1",tolua_gl_glMap100);
  tolua_function(tolua_S,"glMap2",tolua_gl_glMap200);
  tolua_function(tolua_S,"glEvalCoord",tolua_gl_glEvalCoord00);
  tolua_function(tolua_S,"glEvalCoord",tolua_gl_glEvalCoord01);
  tolua_function(tolua_S,"glEvalCoord",tolua_gl_glEvalCoord02);
  tolua_function(tolua_S,"glEvalCoord",tolua_gl_glEvalCoord03);
  tolua_function(tolua_S,"glMapGrid",tolua_gl_glMapGrid00);
  tolua_function(tolua_S,"glMapGrid",tolua_gl_glMapGrid01);
  tolua_function(tolua_S,"glEvalPoint",tolua_gl_glEvalPoint00);
  tolua_function(tolua_S,"glEvalPoint",tolua_gl_glEvalPoint01);
  tolua_function(tolua_S,"glEvalMesh",tolua_gl_glEvalMesh00);
  tolua_function(tolua_S,"glEvalMesh",tolua_gl_glEvalMesh01);
  tolua_function(tolua_S,"glFog",tolua_gl_glFog00);
  tolua_function(tolua_S,"glFog",tolua_gl_glFog01);
  tolua_function(tolua_S,"glPassThrough",tolua_gl_glPassThrough00);
  tolua_function(tolua_S,"glInitNames",tolua_gl_glInitNames00);
  tolua_function(tolua_S,"glLoadName",tolua_gl_glLoadName00);
  tolua_function(tolua_S,"glPushName",tolua_gl_glPushName00);
  tolua_function(tolua_S,"glPopName",tolua_gl_glPopName00);
  tolua_constant(tolua_S,"GL_VERSION_1_1",GL_VERSION_1_1);
  tolua_constant(tolua_S,"GL_DOUBLE",GL_DOUBLE);
  tolua_constant(tolua_S,"GL_VERTEX_ARRAY",GL_VERTEX_ARRAY);
  tolua_constant(tolua_S,"GL_NORMAL_ARRAY",GL_NORMAL_ARRAY);
  tolua_constant(tolua_S,"GL_COLOR_ARRAY",GL_COLOR_ARRAY);
  tolua_constant(tolua_S,"GL_INDEX_ARRAY",GL_INDEX_ARRAY);
  tolua_constant(tolua_S,"GL_TEXTURE_COORD_ARRAY",GL_TEXTURE_COORD_ARRAY);
  tolua_constant(tolua_S,"GL_EDGE_FLAG_ARRAY",GL_EDGE_FLAG_ARRAY);
  tolua_constant(tolua_S,"GL_VERTEX_ARRAY_SIZE",GL_VERTEX_ARRAY_SIZE);
  tolua_constant(tolua_S,"GL_VERTEX_ARRAY_TYPE",GL_VERTEX_ARRAY_TYPE);
  tolua_constant(tolua_S,"GL_VERTEX_ARRAY_STRIDE",GL_VERTEX_ARRAY_STRIDE);
  tolua_constant(tolua_S,"GL_NORMAL_ARRAY_TYPE",GL_NORMAL_ARRAY_TYPE);
  tolua_constant(tolua_S,"GL_NORMAL_ARRAY_STRIDE",GL_NORMAL_ARRAY_STRIDE);
  tolua_constant(tolua_S,"GL_COLOR_ARRAY_SIZE",GL_COLOR_ARRAY_SIZE);
  tolua_constant(tolua_S,"GL_COLOR_ARRAY_TYPE",GL_COLOR_ARRAY_TYPE);
  tolua_constant(tolua_S,"GL_COLOR_ARRAY_STRIDE",GL_COLOR_ARRAY_STRIDE);
  tolua_constant(tolua_S,"GL_INDEX_ARRAY_TYPE",GL_INDEX_ARRAY_TYPE);
  tolua_constant(tolua_S,"GL_INDEX_ARRAY_STRIDE",GL_INDEX_ARRAY_STRIDE);
  tolua_constant(tolua_S,"GL_TEXTURE_COORD_ARRAY_SIZE",GL_TEXTURE_COORD_ARRAY_SIZE);
  tolua_constant(tolua_S,"GL_TEXTURE_COORD_ARRAY_TYPE",GL_TEXTURE_COORD_ARRAY_TYPE);
  tolua_constant(tolua_S,"GL_TEXTURE_COORD_ARRAY_STRIDE",GL_TEXTURE_COORD_ARRAY_STRIDE);
  tolua_constant(tolua_S,"GL_EDGE_FLAG_ARRAY_STRIDE",GL_EDGE_FLAG_ARRAY_STRIDE);
  tolua_constant(tolua_S,"GL_VERTEX_ARRAY_POINTER",GL_VERTEX_ARRAY_POINTER);
  tolua_constant(tolua_S,"GL_NORMAL_ARRAY_POINTER",GL_NORMAL_ARRAY_POINTER);
  tolua_constant(tolua_S,"GL_COLOR_ARRAY_POINTER",GL_COLOR_ARRAY_POINTER);
  tolua_constant(tolua_S,"GL_INDEX_ARRAY_POINTER",GL_INDEX_ARRAY_POINTER);
  tolua_constant(tolua_S,"GL_TEXTURE_COORD_ARRAY_POINTER",GL_TEXTURE_COORD_ARRAY_POINTER);
  tolua_constant(tolua_S,"GL_EDGE_FLAG_ARRAY_POINTER",GL_EDGE_FLAG_ARRAY_POINTER);
  tolua_constant(tolua_S,"GL_V2F",GL_V2F);
  tolua_constant(tolua_S,"GL_V3F",GL_V3F);
  tolua_constant(tolua_S,"GL_C4UB_V2F",GL_C4UB_V2F);
  tolua_constant(tolua_S,"GL_C4UB_V3F",GL_C4UB_V3F);
  tolua_constant(tolua_S,"GL_C3F_V3F",GL_C3F_V3F);
  tolua_constant(tolua_S,"GL_N3F_V3F",GL_N3F_V3F);
  tolua_constant(tolua_S,"GL_C4F_N3F_V3F",GL_C4F_N3F_V3F);
  tolua_constant(tolua_S,"GL_T2F_V3F",GL_T2F_V3F);
  tolua_constant(tolua_S,"GL_T4F_V4F",GL_T4F_V4F);
  tolua_constant(tolua_S,"GL_T2F_C4UB_V3F",GL_T2F_C4UB_V3F);
  tolua_constant(tolua_S,"GL_T2F_C3F_V3F",GL_T2F_C3F_V3F);
  tolua_constant(tolua_S,"GL_T2F_N3F_V3F",GL_T2F_N3F_V3F);
  tolua_constant(tolua_S,"GL_T2F_C4F_N3F_V3F",GL_T2F_C4F_N3F_V3F);
  tolua_constant(tolua_S,"GL_T4F_C4F_N3F_V4F",GL_T4F_C4F_N3F_V4F);
  tolua_constant(tolua_S,"GL_POLYGON_OFFSET_FACTOR",GL_POLYGON_OFFSET_FACTOR);
  tolua_constant(tolua_S,"GL_POLYGON_OFFSET_UNITS",GL_POLYGON_OFFSET_UNITS);
  tolua_constant(tolua_S,"GL_POLYGON_OFFSET_POINT",GL_POLYGON_OFFSET_POINT);
  tolua_constant(tolua_S,"GL_POLYGON_OFFSET_LINE",GL_POLYGON_OFFSET_LINE);
  tolua_constant(tolua_S,"GL_POLYGON_OFFSET_FILL",GL_POLYGON_OFFSET_FILL);
  tolua_constant(tolua_S,"GL_FEEDBACK_BUFFER_POINTER",GL_FEEDBACK_BUFFER_POINTER);
  tolua_constant(tolua_S,"GL_FEEDBACK_BUFFER_SIZE",GL_FEEDBACK_BUFFER_SIZE);
  tolua_constant(tolua_S,"GL_FEEDBACK_BUFFER_TYPE",GL_FEEDBACK_BUFFER_TYPE);
  tolua_constant(tolua_S,"GL_INDEX_LOGIC_OP",GL_INDEX_LOGIC_OP);
  tolua_constant(tolua_S,"GL_COLOR_LOGIC_OP",GL_COLOR_LOGIC_OP);
  tolua_constant(tolua_S,"GL_MAX_CLIENT_ATTRIB_STACK_DEPTH",GL_MAX_CLIENT_ATTRIB_STACK_DEPTH);
  tolua_constant(tolua_S,"GL_CLIENT_ATTRIB_STACK_DEPTH",GL_CLIENT_ATTRIB_STACK_DEPTH);
  tolua_constant(tolua_S,"GL_TEXTURE_RED_SIZE",GL_TEXTURE_RED_SIZE);
  tolua_constant(tolua_S,"GL_TEXTURE_GREEN_SIZE",GL_TEXTURE_GREEN_SIZE);
  tolua_constant(tolua_S,"GL_TEXTURE_BLUE_SIZE",GL_TEXTURE_BLUE_SIZE);
  tolua_constant(tolua_S,"GL_TEXTURE_ALPHA_SIZE",GL_TEXTURE_ALPHA_SIZE);
  tolua_constant(tolua_S,"GL_TEXTURE_LUMINANCE_SIZE",GL_TEXTURE_LUMINANCE_SIZE);
  tolua_constant(tolua_S,"GL_TEXTURE_INTENSITY_SIZE",GL_TEXTURE_INTENSITY_SIZE);
  tolua_constant(tolua_S,"GL_PROXY_TEXTURE_1D",GL_PROXY_TEXTURE_1D);
  tolua_constant(tolua_S,"GL_PROXY_TEXTURE_2D",GL_PROXY_TEXTURE_2D);
  tolua_constant(tolua_S,"GL_TEXTURE_PRIORITY",GL_TEXTURE_PRIORITY);
  tolua_constant(tolua_S,"GL_TEXTURE_RESIDENT",GL_TEXTURE_RESIDENT);
  tolua_constant(tolua_S,"GL_TEXTURE_BINDING_1D",GL_TEXTURE_BINDING_1D);
  tolua_constant(tolua_S,"GL_TEXTURE_BINDING_2D",GL_TEXTURE_BINDING_2D);
  tolua_constant(tolua_S,"GL_CLIENT_PIXEL_STORE_BIT",GL_CLIENT_PIXEL_STORE_BIT);
  tolua_constant(tolua_S,"GL_CLIENT_VERTEX_ARRAY_BIT",GL_CLIENT_VERTEX_ARRAY_BIT);
  tolua_constant(tolua_S,"GL_CLIENT_ALL_ATTRIB_BITS",GL_CLIENT_ALL_ATTRIB_BITS);
  tolua_function(tolua_S,"glPolygonOffset",tolua_gl_glPolygonOffset00);
  tolua_function(tolua_S,"glEnableClientState",tolua_gl_glEnableClientState00);
  tolua_function(tolua_S,"glDisableClientState",tolua_gl_glDisableClientState00);
  tolua_function(tolua_S,"glPushClientAttrib",tolua_gl_glPushClientAttrib00);
  tolua_function(tolua_S,"glPopClientAttrib",tolua_gl_glPopClientAttrib00);
  tolua_function(tolua_S,"glVertexPointer",tolua_gl_glVertexPointer00);
  tolua_function(tolua_S,"glNormalPointer",tolua_gl_glNormalPointer00);
  tolua_function(tolua_S,"glColorPointer",tolua_gl_glColorPointer00);
  tolua_function(tolua_S,"glIndexPointer",tolua_gl_glIndexPointer00);
  tolua_function(tolua_S,"glTexCoordPointer",tolua_gl_glTexCoordPointer00);
  tolua_function(tolua_S,"glEdgeFlagPointer",tolua_gl_glEdgeFlagPointer00);
  tolua_function(tolua_S,"glGetPointerv",tolua_gl_glGetPointerv00);
  tolua_function(tolua_S,"glArrayElement",tolua_gl_glArrayElement00);
  tolua_function(tolua_S,"glDrawArrays",tolua_gl_glDrawArrays00);
  tolua_function(tolua_S,"glDrawElements",tolua_gl_glDrawElements00);
  tolua_function(tolua_S,"glInterleavedArrays",tolua_gl_glInterleavedArrays00);
  tolua_function(tolua_S,"glGenTextures",tolua_gl_glGenTextures00);
  tolua_function(tolua_S,"glDeleteTextures",tolua_gl_glDeleteTextures00);
  tolua_function(tolua_S,"glBindTexture",tolua_gl_glBindTexture00);
  tolua_function(tolua_S,"glPrioritizeTextures",tolua_gl_glPrioritizeTextures00);
  tolua_function(tolua_S,"glAreTexturesResident",tolua_gl_glAreTexturesResident00);
  tolua_function(tolua_S,"glIsTexture",tolua_gl_glIsTexture00);
  tolua_function(tolua_S,"glTexSubImage1D",tolua_gl_glTexSubImage1D00);
  tolua_function(tolua_S,"glTexSubImage2D",tolua_gl_glTexSubImage2D00);
  tolua_function(tolua_S,"glCopyTexImage1D",tolua_gl_glCopyTexImage1D00);
  tolua_function(tolua_S,"glCopyTexImage2D",tolua_gl_glCopyTexImage2D00);
  tolua_function(tolua_S,"glCopyTexSubImage1D",tolua_gl_glCopyTexSubImage1D00);
  tolua_function(tolua_S,"glCopyTexSubImage2D",tolua_gl_glCopyTexSubImage2D00);
  tolua_function(tolua_S,"gllCreateBuffer",tolua_gl_gllCreateBuffer00);
  tolua_function(tolua_S,"gllDeleteBuffer",tolua_gl_gllDeleteBuffer00);
  tolua_function(tolua_S,"gllSetBuffer",tolua_gl_gllSetBuffer00);
  tolua_function(tolua_S,"gllGetBuffer",tolua_gl_gllGetBuffer00);
 tolua_endmodule(tolua_S);
 return 1;
}


#if defined(LUA_VERSION_NUM) && LUA_VERSION_NUM >= 501
 TOLUA_API int luaopen_gl (lua_State* tolua_S) {
 return tolua_gl_open(tolua_S);
};
#endif

