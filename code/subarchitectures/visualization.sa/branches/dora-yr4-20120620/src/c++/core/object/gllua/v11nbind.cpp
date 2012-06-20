/*
** Lua binding: v11n
** Generated automatically by tolua++-1.0.93 on Wed Jul 20 12:46:09 2011.
*/

#ifndef __cplusplus
#include "stdlib.h"
#endif
#include "string.h"

#include "tolua++.h"

/* Exported function */
TOLUA_API int  tolua_v11n_open (lua_State* tolua_S);

#include "v11n.h"

/* function to register type */
static void tolua_reg_types (lua_State* tolua_S)
{
}

/* function: v11nGetOpenGlContext */
#ifndef TOLUA_DISABLE_tolua_v11n_v11nGetOpenGlContext00
static int tolua_v11n_v11nGetOpenGlContext00(lua_State* tolua_S)
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
   void* tolua_ret = (void*)  v11nGetOpenGlContext();
   tolua_pushuserdata(tolua_S,(void*)tolua_ret);
  }
 }
 return 1;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'v11nGetOpenGlContext'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: v11nCamera_SetPosition */
#ifndef TOLUA_DISABLE_tolua_v11n_v11nCamera_SetPosition00
static int tolua_v11n_v11nCamera_SetPosition00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isuserdata(tolua_S,1,0,&tolua_err) ||
     !tolua_isstring(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,7,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,8,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,9,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,10,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,11,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,12,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  void* scriptObj = ((void*)  tolua_touserdata(tolua_S,1,0));
  char* name = ((char*)  tolua_tostring(tolua_S,2,0));
  double x0 = ((double)  tolua_tonumber(tolua_S,3,0));
  double y0 = ((double)  tolua_tonumber(tolua_S,4,0));
  double z0 = ((double)  tolua_tonumber(tolua_S,5,0));
  double x1 = ((double)  tolua_tonumber(tolua_S,6,0));
  double y1 = ((double)  tolua_tonumber(tolua_S,7,0));
  double z1 = ((double)  tolua_tonumber(tolua_S,8,0));
  double xUp = ((double)  tolua_tonumber(tolua_S,9,0));
  double yUp = ((double)  tolua_tonumber(tolua_S,10,0));
  double zUp = ((double)  tolua_tonumber(tolua_S,11,0));
  {
   v11nCamera_SetPosition(scriptObj,name,x0,y0,z0,x1,y1,z1,xUp,yUp,zUp);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'v11nCamera_SetPosition'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* function: v11nGlw_RenderText */
#ifndef TOLUA_DISABLE_tolua_v11n_v11nGlw_RenderText00
static int tolua_v11n_v11nGlw_RenderText00(lua_State* tolua_S)
{
#ifndef TOLUA_RELEASE
 tolua_Error tolua_err;
 if (
     !tolua_isuserdata(tolua_S,1,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,2,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,3,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,4,0,&tolua_err) ||
     !tolua_isstring(tolua_S,5,0,&tolua_err) ||
     !tolua_isnumber(tolua_S,6,0,&tolua_err) ||
     !tolua_isnoobj(tolua_S,7,&tolua_err)
 )
  goto tolua_lerror;
 else
#endif
 {
  void* writerObject = ((void*)  tolua_touserdata(tolua_S,1,0));
  double x = ((double)  tolua_tonumber(tolua_S,2,0));
  double y = ((double)  tolua_tonumber(tolua_S,3,0));
  double z = ((double)  tolua_tonumber(tolua_S,4,0));
  char* text = ((char*)  tolua_tostring(tolua_S,5,0));
  double size = ((double)  tolua_tonumber(tolua_S,6,0));
  {
   v11nGlw_RenderText(writerObject,x,y,z,text,size);
  }
 }
 return 0;
#ifndef TOLUA_RELEASE
 tolua_lerror:
 tolua_error(tolua_S,"#ferror in function 'v11nGlw_RenderText'.",&tolua_err);
 return 0;
#endif
}
#endif //#ifndef TOLUA_DISABLE

/* Open function */
TOLUA_API int tolua_v11n_open (lua_State* tolua_S)
{
 tolua_open(tolua_S);
 tolua_reg_types(tolua_S);
 tolua_module(tolua_S,NULL,0);
 tolua_beginmodule(tolua_S,NULL);
  tolua_function(tolua_S,"v11nGetOpenGlContext",tolua_v11n_v11nGetOpenGlContext00);
  tolua_function(tolua_S,"v11nCamera_SetPosition",tolua_v11n_v11nCamera_SetPosition00);
  tolua_function(tolua_S,"v11nGlw_RenderText",tolua_v11n_v11nGlw_RenderText00);
 tolua_endmodule(tolua_S);
 return 1;
}


#if defined(LUA_VERSION_NUM) && LUA_VERSION_NUM >= 501
 TOLUA_API int luaopen_v11n (lua_State* tolua_S) {
 return tolua_v11n_open(tolua_S);
};
#endif

