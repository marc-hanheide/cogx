/*
** Lua binding: glu
** Generated automatically by tolua++-1.0.93 on Wed Jul 20 12:46:09 2011.
*/

#ifndef __cplusplus
#include "stdlib.h"
#endif
#include "string.h"

#include "tolua++.h"

/* Exported function */
TOLUA_API int  tolua_glu_open (lua_State* tolua_S);

#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>
#include "gluaux.h"
#include "gllbuffer.h"

/* function to register type */
static void tolua_reg_types (lua_State* tolua_S)
{
 tolua_usertype(tolua_S,"GLUquadricObj");
 tolua_usertype(tolua_S,"GLLbuffer");
 tolua_usertype(tolua_S,"GLUnurbsObj");
}

/* function: gluLookAt */
#ifndef TOLUA_DISABLE_tolua_glu_gluLookAt00
static int tolua_glu_gluLookAt00(lua_State* tolua_S)
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
     !tolua_isnoobj(tolua_S,10,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double eyex = ((  double)  tolua_tonumber(tolua_S,1,0));
   double eyey = ((  double)  tolua_tonumber(tolua_S,2,0));
   double eyez = ((  double)  tolua_tonumber(tolua_S,3,0));
   double centerx = ((  double)  tolua_tonumber(tolua_S,4,0));
   double centery = ((  double)  tolua_tonumber(tolua_S,5,0));
   double centerz = ((  double)  tolua_tonumber(tolua_S,6,0));
   double upx = ((  double)  tolua_tonumber(tolua_S,7,0));
   double upy = ((  double)  tolua_tonumber(tolua_S,8,0));
   double upz = ((  double)  tolua_tonumber(tolua_S,9,0));
  {
   gluLookAt(eyex,eyey,eyez,centerx,centery,centerz,upx,upy,upz);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluLookAt'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluOrtho2D */
#ifndef TOLUA_DISABLE_tolua_glu_gluOrtho2D00
static int tolua_glu_gluOrtho2D00(lua_State* tolua_S)
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
   double left = ((  double)  tolua_tonumber(tolua_S,1,0));
   double right = ((  double)  tolua_tonumber(tolua_S,2,0));
   double bottom = ((  double)  tolua_tonumber(tolua_S,3,0));
   double top = ((  double)  tolua_tonumber(tolua_S,4,0));
  {
   gluOrtho2D(left,right,bottom,top);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluOrtho2D'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluPerspective */
#ifndef TOLUA_DISABLE_tolua_glu_gluPerspective00
static int tolua_glu_gluPerspective00(lua_State* tolua_S)
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
   double fovy = ((  double)  tolua_tonumber(tolua_S,1,0));
   double aspect = ((  double)  tolua_tonumber(tolua_S,2,0));
   double zNear = ((  double)  tolua_tonumber(tolua_S,3,0));
   double zFar = ((  double)  tolua_tonumber(tolua_S,4,0));
  {
   gluPerspective(fovy,aspect,zNear,zFar);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluPerspective'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluPickMatrix */
#ifndef TOLUA_DISABLE_tolua_glu_gluPickMatrix00
static int tolua_glu_gluPickMatrix00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_istable(tolua_S,5,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,6,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double x = ((  double)  tolua_tonumber(tolua_S,1,0));
   double y = ((  double)  tolua_tonumber(tolua_S,2,0));
   double width = ((  double)  tolua_tonumber(tolua_S,3,0));
   double height = ((  double)  tolua_tonumber(tolua_S,4,0));
   int viewport[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,5,4,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    viewport[i] = ((int)  tolua_tofieldnumber(tolua_S,5,i+1,0));
   }
  }
  {
   gluPickMatrix(x,y,width,height,viewport);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,5,i+1,(lua_Number) viewport[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluPickMatrix'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluProject */
#ifndef TOLUA_DISABLE_tolua_glu_gluProject00
static int tolua_glu_gluProject00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_istable(tolua_S,4,0,&tolua_err) ||
     !tolua_istable(tolua_S,5,0,&tolua_err) ||
     !tolua_istable(tolua_S,6,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,7,1,&tolua_err) ||
     !tolua_isnumber(tolua_S,8,1,&tolua_err) ||
     !tolua_isnumber(tolua_S,9,1,&tolua_err) ||
     !tolua_isnoobj(tolua_S,10,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double objx = ((  double)  tolua_tonumber(tolua_S,1,0));
   double objy = ((  double)  tolua_tonumber(tolua_S,2,0));
   double objz = ((  double)  tolua_tonumber(tolua_S,3,0));
   double modelMatrix[16];
   double projMatrix[16];
   int viewport[4];
   double winx = ((  double)  tolua_tonumber(tolua_S,7,0));
   double winy = ((  double)  tolua_tonumber(tolua_S,8,0));
   double winz = ((  double)  tolua_tonumber(tolua_S,9,0));
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,4,16,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<16;i++)
    modelMatrix[i] = ((double)  tolua_tofieldnumber(tolua_S,4,i+1,0));
   }
  }
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,5,16,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<16;i++)
    projMatrix[i] = ((double)  tolua_tofieldnumber(tolua_S,5,i+1,0));
   }
  }
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,6,4,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    viewport[i] = ((int)  tolua_tofieldnumber(tolua_S,6,i+1,0));
   }
  }
  {
    int tolua_ret = (  int)  gluProject(objx,objy,objz,modelMatrix,projMatrix,viewport,&winx,&winy,&winz);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
   tolua_pushnumber(tolua_S,(lua_Number)winx);
   tolua_pushnumber(tolua_S,(lua_Number)winy);
   tolua_pushnumber(tolua_S,(lua_Number)winz);
  }
  {
   int i;
   for(i=0; i<16;i++)
    tolua_pushfieldnumber(tolua_S,4,i+1,(lua_Number) modelMatrix[i]);
  }
  {
   int i;
   for(i=0; i<16;i++)
    tolua_pushfieldnumber(tolua_S,5,i+1,(lua_Number) projMatrix[i]);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,6,i+1,(lua_Number) viewport[i]);
  }
 }
 return 4;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluProject'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluUnProject */
#ifndef TOLUA_DISABLE_tolua_glu_gluUnProject00
static int tolua_glu_gluUnProject00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_istable(tolua_S,4,0,&tolua_err) ||
     !tolua_istable(tolua_S,5,0,&tolua_err) ||
     !tolua_istable(tolua_S,6,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,7,1,&tolua_err) ||
     !tolua_isnumber(tolua_S,8,1,&tolua_err) ||
     !tolua_isnumber(tolua_S,9,1,&tolua_err) ||
     !tolua_isnoobj(tolua_S,10,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   double winx = ((  double)  tolua_tonumber(tolua_S,1,0));
   double winy = ((  double)  tolua_tonumber(tolua_S,2,0));
   double winz = ((  double)  tolua_tonumber(tolua_S,3,0));
   double modelMatrix[16];
   double projMatrix[16];
   int viewport[4];
   double objx = ((  double)  tolua_tonumber(tolua_S,7,0));
   double objy = ((  double)  tolua_tonumber(tolua_S,8,0));
   double objz = ((  double)  tolua_tonumber(tolua_S,9,0));
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,4,16,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<16;i++)
    modelMatrix[i] = ((double)  tolua_tofieldnumber(tolua_S,4,i+1,0));
   }
  }
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,5,16,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<16;i++)
    projMatrix[i] = ((double)  tolua_tofieldnumber(tolua_S,5,i+1,0));
   }
  }
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,6,4,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    viewport[i] = ((int)  tolua_tofieldnumber(tolua_S,6,i+1,0));
   }
  }
  {
    int tolua_ret = (  int)  gluUnProject(winx,winy,winz,modelMatrix,projMatrix,viewport,&objx,&objy,&objz);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
   tolua_pushnumber(tolua_S,(lua_Number)objx);
   tolua_pushnumber(tolua_S,(lua_Number)objy);
   tolua_pushnumber(tolua_S,(lua_Number)objz);
  }
  {
   int i;
   for(i=0; i<16;i++)
    tolua_pushfieldnumber(tolua_S,4,i+1,(lua_Number) modelMatrix[i]);
  }
  {
   int i;
   for(i=0; i<16;i++)
    tolua_pushfieldnumber(tolua_S,5,i+1,(lua_Number) projMatrix[i]);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,6,i+1,(lua_Number) viewport[i]);
  }
 }
 return 4;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluUnProject'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluErrorString */
#ifndef TOLUA_DISABLE_tolua_glu_gluErrorString00
static int tolua_glu_gluErrorString00(lua_State* tolua_S)
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
   int errorCode = ((  int)  tolua_tonumber(tolua_S,1,0));
  {
   unsigned const char* tolua_ret = ( unsigned const char*)  gluErrorString(errorCode);
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluErrorString'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluScaleImage */
#ifndef TOLUA_DISABLE_tolua_glu_gluScaleImage00
static int tolua_glu_gluScaleImage00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,5,"GLLbuffer",0,&tolua_err) ||
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
   int format = ((  int)  tolua_tonumber(tolua_S,1,0));
   int widthin = ((  int)  tolua_tonumber(tolua_S,2,0));
   int heightin = ((  int)  tolua_tonumber(tolua_S,3,0));
   int typein = ((  int)  tolua_tonumber(tolua_S,4,0));
  GLLbuffer* datain = ((GLLbuffer*)  tolua_tousertype(tolua_S,5,0));
   int widthout = ((  int)  tolua_tonumber(tolua_S,6,0));
   int heightout = ((  int)  tolua_tonumber(tolua_S,7,0));
   int typeout = ((  int)  tolua_tonumber(tolua_S,8,0));
  GLLbuffer* dataout = ((GLLbuffer*)  tolua_tousertype(tolua_S,9,0));
  {
    int tolua_ret = (  int)  gluScaleImage(format,widthin,heightin,typein,datain,widthout,heightout,typeout,dataout);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluScaleImage'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluBuild1DMipmaps */
#ifndef TOLUA_DISABLE_tolua_glu_gluBuild1DMipmaps00
static int tolua_glu_gluBuild1DMipmaps00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isnumber(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isusertype(tolua_S,6,"GLLbuffer",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,7,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
   int target = ((  int)  tolua_tonumber(tolua_S,1,0));
   int components = ((  int)  tolua_tonumber(tolua_S,2,0));
   int width = ((  int)  tolua_tonumber(tolua_S,3,0));
   int format = ((  int)  tolua_tonumber(tolua_S,4,0));
   int type = ((  int)  tolua_tonumber(tolua_S,5,0));
  GLLbuffer* data = ((GLLbuffer*)  tolua_tousertype(tolua_S,6,0));
  {
    int tolua_ret = (  int)  gluBuild1DMipmaps(target,components,width,format,type,data);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluBuild1DMipmaps'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluBuild2DMipmaps */
#ifndef TOLUA_DISABLE_tolua_glu_gluBuild2DMipmaps00
static int tolua_glu_gluBuild2DMipmaps00(lua_State* tolua_S)
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
   int target = ((  int)  tolua_tonumber(tolua_S,1,0));
   int components = ((  int)  tolua_tonumber(tolua_S,2,0));
   int width = ((  int)  tolua_tonumber(tolua_S,3,0));
   int height = ((  int)  tolua_tonumber(tolua_S,4,0));
   int format = ((  int)  tolua_tonumber(tolua_S,5,0));
   int type = ((  int)  tolua_tonumber(tolua_S,6,0));
  GLLbuffer* data = ((GLLbuffer*)  tolua_tousertype(tolua_S,7,0));
  {
    int tolua_ret = (  int)  gluBuild2DMipmaps(target,components,width,height,format,type,data);
   tolua_pushnumber(tolua_S,(lua_Number)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluBuild2DMipmaps'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluNewQuadric */
#ifndef TOLUA_DISABLE_tolua_glu_gluNewQuadric00
static int tolua_glu_gluNewQuadric00(lua_State* tolua_S)
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
   GLUquadricObj* tolua_ret = (GLUquadricObj*)  gluNewQuadric();
    tolua_pushusertype(tolua_S,(void*)tolua_ret,"GLUquadricObj");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluNewQuadric'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluDeleteQuadric */
#ifndef TOLUA_DISABLE_tolua_glu_gluDeleteQuadric00
static int tolua_glu_gluDeleteQuadric00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUquadricObj",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUquadricObj* state = ((GLUquadricObj*)  tolua_tousertype(tolua_S,1,0));
  {
   gluDeleteQuadric(state);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluDeleteQuadric'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluQuadricDrawStyle */
#ifndef TOLUA_DISABLE_tolua_glu_gluQuadricDrawStyle00
static int tolua_glu_gluQuadricDrawStyle00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUquadricObj",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUquadricObj* quadObject = ((GLUquadricObj*)  tolua_tousertype(tolua_S,1,0));
   int drawStyle = ((  int)  tolua_tonumber(tolua_S,2,0));
  {
   gluQuadricDrawStyle(quadObject,drawStyle);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluQuadricDrawStyle'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluQuadricOrientation */
#ifndef TOLUA_DISABLE_tolua_glu_gluQuadricOrientation00
static int tolua_glu_gluQuadricOrientation00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUquadricObj",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUquadricObj* quadObject = ((GLUquadricObj*)  tolua_tousertype(tolua_S,1,0));
   int orientation = ((  int)  tolua_tonumber(tolua_S,2,0));
  {
   gluQuadricOrientation(quadObject,orientation);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluQuadricOrientation'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluQuadricNormals */
#ifndef TOLUA_DISABLE_tolua_glu_gluQuadricNormals00
static int tolua_glu_gluQuadricNormals00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUquadricObj",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUquadricObj* quadObject = ((GLUquadricObj*)  tolua_tousertype(tolua_S,1,0));
   int normals = ((  int)  tolua_tonumber(tolua_S,2,0));
  {
   gluQuadricNormals(quadObject,normals);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluQuadricNormals'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluQuadricTexture */
#ifndef TOLUA_DISABLE_tolua_glu_gluQuadricTexture00
static int tolua_glu_gluQuadricTexture00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUquadricObj",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,3,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUquadricObj* quadObject = ((GLUquadricObj*)  tolua_tousertype(tolua_S,1,0));
  unsigned char textureCoords = (( unsigned char)  tolua_tonumber(tolua_S,2,0));
  {
   gluQuadricTexture(quadObject,textureCoords);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluQuadricTexture'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluCylinder */
#ifndef TOLUA_DISABLE_tolua_glu_gluCylinder00
static int tolua_glu_gluCylinder00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUquadricObj",0,&tolua_err) ||
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
  GLUquadricObj* qobj = ((GLUquadricObj*)  tolua_tousertype(tolua_S,1,0));
   double baseRadius = ((  double)  tolua_tonumber(tolua_S,2,0));
   double topRadius = ((  double)  tolua_tonumber(tolua_S,3,0));
   double height = ((  double)  tolua_tonumber(tolua_S,4,0));
   int slices = ((  int)  tolua_tonumber(tolua_S,5,0));
   int stacks = ((  int)  tolua_tonumber(tolua_S,6,0));
  {
   gluCylinder(qobj,baseRadius,topRadius,height,slices,stacks);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluCylinder'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluSphere */
#ifndef TOLUA_DISABLE_tolua_glu_gluSphere00
static int tolua_glu_gluSphere00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUquadricObj",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUquadricObj* qobj = ((GLUquadricObj*)  tolua_tousertype(tolua_S,1,0));
   double radius = ((  double)  tolua_tonumber(tolua_S,2,0));
   int slices = ((  int)  tolua_tonumber(tolua_S,3,0));
   int stacks = ((  int)  tolua_tonumber(tolua_S,4,0));
  {
   gluSphere(qobj,radius,slices,stacks);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluSphere'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluDisk */
#ifndef TOLUA_DISABLE_tolua_glu_gluDisk00
static int tolua_glu_gluDisk00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUquadricObj",0,&tolua_err) ||
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
  GLUquadricObj* qobj = ((GLUquadricObj*)  tolua_tousertype(tolua_S,1,0));
   double innerRadius = ((  double)  tolua_tonumber(tolua_S,2,0));
   double outerRadius = ((  double)  tolua_tonumber(tolua_S,3,0));
   int slices = ((  int)  tolua_tonumber(tolua_S,4,0));
   int loops = ((  int)  tolua_tonumber(tolua_S,5,0));
  {
   gluDisk(qobj,innerRadius,outerRadius,slices,loops);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluDisk'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluPartialDisk */
#ifndef TOLUA_DISABLE_tolua_glu_gluPartialDisk00
static int tolua_glu_gluPartialDisk00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUquadricObj",0,&tolua_err) ||
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
  GLUquadricObj* qobj = ((GLUquadricObj*)  tolua_tousertype(tolua_S,1,0));
   double innerRadius = ((  double)  tolua_tonumber(tolua_S,2,0));
   double outerRadius = ((  double)  tolua_tonumber(tolua_S,3,0));
   int slices = ((  int)  tolua_tonumber(tolua_S,4,0));
   int loops = ((  int)  tolua_tonumber(tolua_S,5,0));
   double startAngle = ((  double)  tolua_tonumber(tolua_S,6,0));
   double sweepAngle = ((  double)  tolua_tonumber(tolua_S,7,0));
  {
   gluPartialDisk(qobj,innerRadius,outerRadius,slices,loops,startAngle,sweepAngle);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluPartialDisk'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluNewNurbsRenderer */
#ifndef TOLUA_DISABLE_tolua_glu_gluNewNurbsRenderer00
static int tolua_glu_gluNewNurbsRenderer00(lua_State* tolua_S)
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
   GLUnurbsObj* tolua_ret = (GLUnurbsObj*)  gluNewNurbsRenderer();
    tolua_pushusertype(tolua_S,(void*)tolua_ret,"GLUnurbsObj");
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluNewNurbsRenderer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluDeleteNurbsRenderer */
#ifndef TOLUA_DISABLE_tolua_glu_gluDeleteNurbsRenderer00
static int tolua_glu_gluDeleteNurbsRenderer00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUnurbsObj",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUnurbsObj* nobj = ((GLUnurbsObj*)  tolua_tousertype(tolua_S,1,0));
  {
   gluDeleteNurbsRenderer(nobj);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluDeleteNurbsRenderer'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluLoadSamplingMatrices */
#ifndef TOLUA_DISABLE_tolua_glu_gluLoadSamplingMatrices00
static int tolua_glu_gluLoadSamplingMatrices00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUnurbsObj",0,&tolua_err) ||
     !tolua_istable(tolua_S,2,0,&tolua_err) ||
     !tolua_istable(tolua_S,3,0,&tolua_err) ||
     !tolua_istable(tolua_S,4,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,5,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUnurbsObj* nobj = ((GLUnurbsObj*)  tolua_tousertype(tolua_S,1,0));
   float modelMatrix[16];
   float projMatrix[16];
   int viewport[4];
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,2,16,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<16;i++)
    modelMatrix[i] = ((float)  tolua_tofieldnumber(tolua_S,2,i+1,0));
   }
  }
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,3,16,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<16;i++)
    projMatrix[i] = ((float)  tolua_tofieldnumber(tolua_S,3,i+1,0));
   }
  }
  {
#ifndef TOLUA_RELEASE
   if (!tolua_isnumberarray(tolua_S,4,4,0,&tolua_err))
    goto tolua_lerror;
   else
#endif
   {
    int i;
    for(i=0; i<4;i++)
    viewport[i] = ((int)  tolua_tofieldnumber(tolua_S,4,i+1,0));
   }
  }
  {
   gluLoadSamplingMatrices(nobj,modelMatrix,projMatrix,viewport);
  }
  {
   int i;
   for(i=0; i<16;i++)
    tolua_pushfieldnumber(tolua_S,2,i+1,(lua_Number) modelMatrix[i]);
  }
  {
   int i;
   for(i=0; i<16;i++)
    tolua_pushfieldnumber(tolua_S,3,i+1,(lua_Number) projMatrix[i]);
  }
  {
   int i;
   for(i=0; i<4;i++)
    tolua_pushfieldnumber(tolua_S,4,i+1,(lua_Number) viewport[i]);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluLoadSamplingMatrices'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluNurbsProperty */
#ifndef TOLUA_DISABLE_tolua_glu_gluNurbsProperty00
static int tolua_glu_gluNurbsProperty00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUnurbsObj",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUnurbsObj* nobj = ((GLUnurbsObj*)  tolua_tousertype(tolua_S,1,0));
   int property = ((  int)  tolua_tonumber(tolua_S,2,0));
   float value = ((  float)  tolua_tonumber(tolua_S,3,0));
  {
   gluNurbsProperty(nobj,property,value);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluNurbsProperty'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluGetNurbsProperty */
#ifndef TOLUA_DISABLE_tolua_glu_gluGetNurbsProperty00
static int tolua_glu_gluGetNurbsProperty00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUnurbsObj",0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,4,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUnurbsObj* nobj = ((GLUnurbsObj*)  tolua_tousertype(tolua_S,1,0));
   int property = ((  int)  tolua_tonumber(tolua_S,2,0));
   float value = ((  float)  tolua_tonumber(tolua_S,3,0));
  {
   gluGetNurbsProperty(nobj,property,&value);
   tolua_pushnumber(tolua_S,(lua_Number)value);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluGetNurbsProperty'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluBeginCurve */
#ifndef TOLUA_DISABLE_tolua_glu_gluBeginCurve00
static int tolua_glu_gluBeginCurve00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUnurbsObj",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUnurbsObj* nobj = ((GLUnurbsObj*)  tolua_tousertype(tolua_S,1,0));
  {
   gluBeginCurve(nobj);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluBeginCurve'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluEndCurve */
#ifndef TOLUA_DISABLE_tolua_glu_gluEndCurve00
static int tolua_glu_gluEndCurve00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUnurbsObj",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUnurbsObj* nobj = ((GLUnurbsObj*)  tolua_tousertype(tolua_S,1,0));
  {
   gluEndCurve(nobj);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluEndCurve'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluBeginSurface */
#ifndef TOLUA_DISABLE_tolua_glu_gluBeginSurface00
static int tolua_glu_gluBeginSurface00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUnurbsObj",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUnurbsObj* nobj = ((GLUnurbsObj*)  tolua_tousertype(tolua_S,1,0));
  {
   gluBeginSurface(nobj);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluBeginSurface'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluEndSurface */
#ifndef TOLUA_DISABLE_tolua_glu_gluEndSurface00
static int tolua_glu_gluEndSurface00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUnurbsObj",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUnurbsObj* nobj = ((GLUnurbsObj*)  tolua_tousertype(tolua_S,1,0));
  {
   gluEndSurface(nobj);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluEndSurface'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluBeginTrim */
#ifndef TOLUA_DISABLE_tolua_glu_gluBeginTrim00
static int tolua_glu_gluBeginTrim00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUnurbsObj",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUnurbsObj* nobj = ((GLUnurbsObj*)  tolua_tousertype(tolua_S,1,0));
  {
   gluBeginTrim(nobj);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluBeginTrim'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluEndTrim */
#ifndef TOLUA_DISABLE_tolua_glu_gluEndTrim00
static int tolua_glu_gluEndTrim00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isusertype(tolua_S,1,"GLUnurbsObj",0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,2,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  GLUnurbsObj* nobj = ((GLUnurbsObj*)  tolua_tousertype(tolua_S,1,0));
  {
   gluEndTrim(nobj);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluEndTrim'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: gluGetString */
#ifndef TOLUA_DISABLE_tolua_glu_gluGetString00
static int tolua_glu_gluGetString00(lua_State* tolua_S)
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
   int name = ((  int)  tolua_tonumber(tolua_S,1,0));
  {
   unsigned const char* tolua_ret = ( unsigned const char*)  gluGetString(name);
   tolua_pushstring(tolua_S,(const char*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'gluGetString'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* Open function */
TOLUA_API int tolua_glu_open (lua_State* tolua_S)
{
 tolua_open(tolua_S);
 tolua_reg_types(tolua_S);
 tolua_module(tolua_S,NULL,0);
 tolua_beginmodule(tolua_S,NULL);
  tolua_constant(tolua_S,"GLU_VERSION_1_1",GLU_VERSION_1_1);
  tolua_constant(tolua_S,"GLU_TRUE",GLU_TRUE);
  tolua_constant(tolua_S,"GLU_FALSE",GLU_FALSE);
  tolua_constant(tolua_S,"GLU_SMOOTH",GLU_SMOOTH);
  tolua_constant(tolua_S,"GLU_FLAT",GLU_FLAT);
  tolua_constant(tolua_S,"GLU_NONE",GLU_NONE);
  tolua_constant(tolua_S,"GLU_POINT",GLU_POINT);
  tolua_constant(tolua_S,"GLU_LINE",GLU_LINE);
  tolua_constant(tolua_S,"GLU_FILL",GLU_FILL);
  tolua_constant(tolua_S,"GLU_SILHOUETTE",GLU_SILHOUETTE);
  tolua_constant(tolua_S,"GLU_OUTSIDE",GLU_OUTSIDE);
  tolua_constant(tolua_S,"GLU_INSIDE",GLU_INSIDE);
  tolua_constant(tolua_S,"GLU_TESS_BEGIN",GLU_TESS_BEGIN);
  tolua_constant(tolua_S,"GLU_TESS_VERTEX",GLU_TESS_VERTEX);
  tolua_constant(tolua_S,"GLU_TESS_END",GLU_TESS_END);
  tolua_constant(tolua_S,"GLU_TESS_ERROR",GLU_TESS_ERROR);
  tolua_constant(tolua_S,"GLU_TESS_EDGE_FLAG",GLU_TESS_EDGE_FLAG);
  tolua_constant(tolua_S,"GLU_TESS_COMBINE",GLU_TESS_COMBINE);
  tolua_constant(tolua_S,"GLU_TESS_ERROR1",GLU_TESS_ERROR1);
  tolua_constant(tolua_S,"GLU_TESS_ERROR2",GLU_TESS_ERROR2);
  tolua_constant(tolua_S,"GLU_TESS_ERROR3",GLU_TESS_ERROR3);
  tolua_constant(tolua_S,"GLU_TESS_ERROR4",GLU_TESS_ERROR4);
  tolua_constant(tolua_S,"GLU_TESS_ERROR5",GLU_TESS_ERROR5);
  tolua_constant(tolua_S,"GLU_TESS_ERROR6",GLU_TESS_ERROR6);
  tolua_constant(tolua_S,"GLU_TESS_ERROR7",GLU_TESS_ERROR7);
  tolua_constant(tolua_S,"GLU_TESS_ERROR8",GLU_TESS_ERROR8);
  tolua_constant(tolua_S,"GLU_AUTO_LOAD_MATRIX",GLU_AUTO_LOAD_MATRIX);
  tolua_constant(tolua_S,"GLU_CULLING",GLU_CULLING);
  tolua_constant(tolua_S,"GLU_PARAMETRIC_TOLERANCE",GLU_PARAMETRIC_TOLERANCE);
  tolua_constant(tolua_S,"GLU_SAMPLING_TOLERANCE",GLU_SAMPLING_TOLERANCE);
  tolua_constant(tolua_S,"GLU_DISPLAY_MODE",GLU_DISPLAY_MODE);
  tolua_constant(tolua_S,"GLU_SAMPLING_METHOD",GLU_SAMPLING_METHOD);
  tolua_constant(tolua_S,"GLU_U_STEP",GLU_U_STEP);
  tolua_constant(tolua_S,"GLU_V_STEP",GLU_V_STEP);
  tolua_constant(tolua_S,"GLU_PATH_LENGTH",GLU_PATH_LENGTH);
  tolua_constant(tolua_S,"GLU_PARAMETRIC_ERROR",GLU_PARAMETRIC_ERROR);
  tolua_constant(tolua_S,"GLU_DOMAIN_DISTANCE",GLU_DOMAIN_DISTANCE);
  tolua_constant(tolua_S,"GLU_MAP1_TRIM_2",GLU_MAP1_TRIM_2);
  tolua_constant(tolua_S,"GLU_MAP1_TRIM_3",GLU_MAP1_TRIM_3);
  tolua_constant(tolua_S,"GLU_OUTLINE_POLYGON",GLU_OUTLINE_POLYGON);
  tolua_constant(tolua_S,"GLU_OUTLINE_PATCH",GLU_OUTLINE_PATCH);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR1",GLU_NURBS_ERROR1);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR2",GLU_NURBS_ERROR2);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR3",GLU_NURBS_ERROR3);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR4",GLU_NURBS_ERROR4);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR5",GLU_NURBS_ERROR5);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR6",GLU_NURBS_ERROR6);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR7",GLU_NURBS_ERROR7);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR8",GLU_NURBS_ERROR8);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR9",GLU_NURBS_ERROR9);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR10",GLU_NURBS_ERROR10);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR11",GLU_NURBS_ERROR11);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR12",GLU_NURBS_ERROR12);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR13",GLU_NURBS_ERROR13);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR14",GLU_NURBS_ERROR14);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR15",GLU_NURBS_ERROR15);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR16",GLU_NURBS_ERROR16);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR17",GLU_NURBS_ERROR17);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR18",GLU_NURBS_ERROR18);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR19",GLU_NURBS_ERROR19);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR20",GLU_NURBS_ERROR20);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR21",GLU_NURBS_ERROR21);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR22",GLU_NURBS_ERROR22);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR23",GLU_NURBS_ERROR23);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR24",GLU_NURBS_ERROR24);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR25",GLU_NURBS_ERROR25);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR26",GLU_NURBS_ERROR26);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR27",GLU_NURBS_ERROR27);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR28",GLU_NURBS_ERROR28);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR29",GLU_NURBS_ERROR29);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR30",GLU_NURBS_ERROR30);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR31",GLU_NURBS_ERROR31);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR32",GLU_NURBS_ERROR32);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR33",GLU_NURBS_ERROR33);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR34",GLU_NURBS_ERROR34);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR35",GLU_NURBS_ERROR35);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR36",GLU_NURBS_ERROR36);
  tolua_constant(tolua_S,"GLU_NURBS_ERROR37",GLU_NURBS_ERROR37);
  tolua_constant(tolua_S,"GLU_INVALID_ENUM",GLU_INVALID_ENUM);
  tolua_constant(tolua_S,"GLU_INVALID_VALUE",GLU_INVALID_VALUE);
  tolua_constant(tolua_S,"GLU_OUT_OF_MEMORY",GLU_OUT_OF_MEMORY);
  tolua_constant(tolua_S,"GLU_INCOMPATIBLE_GL_VERSION",GLU_INCOMPATIBLE_GL_VERSION);
  tolua_constant(tolua_S,"GLU_VERSION",GLU_VERSION);
  tolua_constant(tolua_S,"GLU_EXTENSIONS",GLU_EXTENSIONS);
  tolua_function(tolua_S,"gluLookAt",tolua_glu_gluLookAt00);
  tolua_function(tolua_S,"gluOrtho2D",tolua_glu_gluOrtho2D00);
  tolua_function(tolua_S,"gluPerspective",tolua_glu_gluPerspective00);
  tolua_function(tolua_S,"gluPickMatrix",tolua_glu_gluPickMatrix00);
  tolua_function(tolua_S,"gluProject",tolua_glu_gluProject00);
  tolua_function(tolua_S,"gluUnProject",tolua_glu_gluUnProject00);
  tolua_function(tolua_S,"gluErrorString",tolua_glu_gluErrorString00);
  tolua_function(tolua_S,"gluScaleImage",tolua_glu_gluScaleImage00);
  tolua_function(tolua_S,"gluBuild1DMipmaps",tolua_glu_gluBuild1DMipmaps00);
  tolua_function(tolua_S,"gluBuild2DMipmaps",tolua_glu_gluBuild2DMipmaps00);
  tolua_function(tolua_S,"gluNewQuadric",tolua_glu_gluNewQuadric00);
  tolua_function(tolua_S,"gluDeleteQuadric",tolua_glu_gluDeleteQuadric00);
  tolua_function(tolua_S,"gluQuadricDrawStyle",tolua_glu_gluQuadricDrawStyle00);
  tolua_function(tolua_S,"gluQuadricOrientation",tolua_glu_gluQuadricOrientation00);
  tolua_function(tolua_S,"gluQuadricNormals",tolua_glu_gluQuadricNormals00);
  tolua_function(tolua_S,"gluQuadricTexture",tolua_glu_gluQuadricTexture00);
  tolua_function(tolua_S,"gluCylinder",tolua_glu_gluCylinder00);
  tolua_function(tolua_S,"gluSphere",tolua_glu_gluSphere00);
  tolua_function(tolua_S,"gluDisk",tolua_glu_gluDisk00);
  tolua_function(tolua_S,"gluPartialDisk",tolua_glu_gluPartialDisk00);
  tolua_function(tolua_S,"gluNewNurbsRenderer",tolua_glu_gluNewNurbsRenderer00);
  tolua_function(tolua_S,"gluDeleteNurbsRenderer",tolua_glu_gluDeleteNurbsRenderer00);
  tolua_function(tolua_S,"gluLoadSamplingMatrices",tolua_glu_gluLoadSamplingMatrices00);
  tolua_function(tolua_S,"gluNurbsProperty",tolua_glu_gluNurbsProperty00);
  tolua_function(tolua_S,"gluGetNurbsProperty",tolua_glu_gluGetNurbsProperty00);
  tolua_function(tolua_S,"gluBeginCurve",tolua_glu_gluBeginCurve00);
  tolua_function(tolua_S,"gluEndCurve",tolua_glu_gluEndCurve00);
  tolua_function(tolua_S,"gluBeginSurface",tolua_glu_gluBeginSurface00);
  tolua_function(tolua_S,"gluEndSurface",tolua_glu_gluEndSurface00);
  tolua_function(tolua_S,"gluBeginTrim",tolua_glu_gluBeginTrim00);
  tolua_function(tolua_S,"gluEndTrim",tolua_glu_gluEndTrim00);
  tolua_function(tolua_S,"gluGetString",tolua_glu_gluGetString00);
 tolua_endmodule(tolua_S);
 return 1;
}


#if defined(LUA_VERSION_NUM) && LUA_VERSION_NUM >= 501
 TOLUA_API int luaopen_glu (lua_State* tolua_S) {
 return tolua_glu_open(tolua_S);
};
#endif

