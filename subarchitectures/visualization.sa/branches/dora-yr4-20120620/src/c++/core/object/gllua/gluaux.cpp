/* gluaux.c
** Auxiliary function to bind GLU to Lua.
** Waldemar Celes
** Sep 1998
*/

#include <stdlib.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
extern "C" {
#include "lua.h"
}
#include "gluaux.h"

#ifndef WIN32
#define CALLBACK
#endif

// extern lua_State* luaS; // naughty but nice - NT
typedef void (*FN) ();

/* Type to store tesselator callback */
typedef struct Tesselator
{
 int begin;
 int edgeflag;
 int vertex;
 int end;
 int error;
 GLUtriangulatorObj* obj;
 struct Tesselator* next;
} Tesselator;


static Tesselator* head = nullptr;
static Tesselator* curr = nullptr;

static void create (GLUtriangulatorObj* tessobj)
{
 Tesselator* tess = (Tesselator*)malloc(sizeof(Tesselator));
 tess->obj = tessobj;
 tess->begin = 0;
 tess->edgeflag = 0;
 tess->vertex = 0;
 tess->end = 0;
 tess->error = 0;
 tess->next = head;
 head = curr = tess;
}

static void release (GLUtriangulatorObj* tessobj)
{
 Tesselator* prev = nullptr;
 Tesselator* tess = head;
 while (tess->obj != tessobj)
 {
  prev = tess;
  tess = tess->next;
 }

 if (tess)
 {
  if (prev)
   prev->next = tess->next;
  else
   head = tess->next;
  if (curr == tess)
   curr = head;
  free(tess);
 }
}

static void current (GLUtriangulatorObj* tessobj)
{
 if (curr->obj != tessobj)
 {
  Tesselator* tess = head;
  while (tess->obj != tessobj)
   tess = tess->next;
  curr = tess;
 }
}

//static void CALLBACK cb_begin (GLenum type)
//{
// if (curr && curr->begin)
// {
////  lua_pushnumber(luaS,(lua_Number)type);
//  lua_getref(luaS,curr->begin);
//  lua_call(luaS,1,0);
// }
//}

//static void CALLBACK cb_edgeflag (GLboolean flag)
//{
// if (curr && curr->edgeflag)
// {
//  lua_pushnumber(luaS,flag);
//  lua_getref(luaS,curr->edgeflag);
//  lua_call(luaS,1,0);
// }
//}

//static void CALLBACK cb_vertex (void* data)
//{
// if (curr && curr->vertex)
// {
//  if (data)
//  {
//   //lua_pushobject(lua_getref((int)data));
//   lua_getref(luaS,(uintptr_t)data);
//  }
//  else
//   lua_pushnil(luaS);

//  lua_getref(luaS,curr->vertex);
//  lua_call(luaS,1,0);
// }
//}

//static void CALLBACK cb_end (void)
//{
// if (curr && curr->end)
// {
//  lua_getref(luaS,curr->end);
//  lua_call(luaS,1,0);
// }
//}

//static void CALLBACK cb_error (GLenum errno)
//{
// if (curr && curr->error)
// {
//  lua_pushnumber(luaS,(lua_Number)errno);
//  lua_getref(luaS,curr->error);
//  lua_call(luaS,1,0);
// }
//}


static void callback (GLUtriangulatorObj* tessobj, GLenum type, lua_Number fn)
{
	/*
 if (curr)
 {
  int ref;
  if (fn!=-1) lua_error(luaS,"oops callback not on stack top");
  ref = lua_ref(luaS, 0);
  switch (type)
  {
   case GLU_BEGIN: 
    curr->begin = ref;
    gluTessCallback(tessobj,type,(FN)cb_begin);
   break;
   case GLU_EDGE_FLAG: 
    curr->edgeflag = ref; 
    gluTessCallback(tessobj,type,(FN)cb_edgeflag); 
   break;
   case GLU_VERTEX: 
    curr->vertex = ref; 
    gluTessCallback(tessobj,type,(FN)cb_vertex); 
   break;
   case GLU_END: 
    curr->end = ref; 
    gluTessCallback(tessobj,type,(FN)cb_end); 
   break;
   case GLU_ERROR: 
    curr->error = ref; 
    gluTessCallback(tessobj,type,(FN)cb_error); 
   break;
   default: 
   break;
  }
 }*/
}

/*******************************************************************/

GLUtriangulatorObj* gluauxNewTess (void)
{
 GLUtriangulatorObj* tessobj = gluNewTess();
 create(tessobj);
 return tessobj; 
}

void gluauxDeleteTess (GLUtriangulatorObj* tessobj)
{
 release(tessobj);
}

void gluauxTessCallback (GLUtriangulatorObj* tessobj, GLenum type, lua_Number fn)
{
 current(tessobj);
 callback(tessobj,type,fn);
}

void gluauxBeginPolygon (GLUtriangulatorObj* tessobj)
{
 current(tessobj);
 gluBeginPolygon(tessobj); 
}

//void gluauxTessVertex (GLUtriangulatorObj* tessobj, GLdouble v[3], lua_Number data)
//{
// void* vdata;
// current(tessobj);
// // lua_pushobject(data);
// lua_pushvalue(luaS, (int) data);
// // vdata = (void*)lua_ref(0);
// vdata = (void*)lua_ref(luaS,0);
// gluTessVertex(tessobj,v,vdata); 
//}

void gluauxNextContour (GLUtriangulatorObj* tessobj, GLenum type)
{
 current(tessobj);
 gluNextContour(tessobj,type);
}

void gluauxEndPolygon (GLUtriangulatorObj* tessobj)
{
 current(tessobj);
 gluEndPolygon(tessobj);
}
