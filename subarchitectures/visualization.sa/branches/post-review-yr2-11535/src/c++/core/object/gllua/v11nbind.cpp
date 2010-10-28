/*
** Lua binding: v11n
** Generated automatically by tolua++-1.0.93 on Tue Jun  8 18:56:05 2010.
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

/* Open function */
TOLUA_API int tolua_v11n_open (lua_State* tolua_S)
{
 tolua_open(tolua_S);
 tolua_reg_types(tolua_S);
 tolua_module(tolua_S,NULL,0);
 tolua_beginmodule(tolua_S,NULL);
  tolua_function(tolua_S,"v11nGetOpenGlContext",tolua_v11n_v11nGetOpenGlContext00);
 tolua_endmodule(tolua_S);
 return 1;
}


#if defined(LUA_VERSION_NUM) && LUA_VERSION_NUM >= 501
 TOLUA_API int luaopen_v11n (lua_State* tolua_S) {
 return tolua_v11n_open(tolua_S);
};
#endif

