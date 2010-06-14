/*
 * Author: Marko Mahnič
 * Created: 2010-06-02
 *
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "CLuaGlScript.hpp"

extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}
#include "tolua++.h"
#include "glbind.h"
#include "glubind.h"
#include "v11nbind.h"
#include <gl.h>

#ifdef DEBUG_TRACE
#undef DEBUG_TRACE
#endif
#include "../convenience.hpp"


namespace cogx { namespace display {

#include "v11n_luacode.inc"

std::auto_ptr<CRenderer> CLuaGlScript::renderGL(new CLuaGlScript_RenderGL());

CLuaGlScript::CScript::CScript()
{
   luaS = NULL;
}

CLuaGlScript::CScript::~CScript()
{
   if (luaS) lua_close(luaS);
   luaS = NULL;
}

// http://www.sharpoblunto.com/News/lua
std::list<std::string> _stack;
void FunctionHook(lua_State *l, lua_Debug *ar)
{
   //fill up the debug structure with information from the lua stack
   lua_getinfo(l, "Sln", ar);
   //push function calls to the top of the callstack
   if (ar->event == LUA_HOOKCALL) {

      std::stringstream ss;
      ss << ar->short_src << ":"

	 << ar->linedefined << ": "
	 << (ar->name == NULL ? "[UNKNOWN]" : ar->name)
	 << " (" << ar->namewhat << ")";

      _stack.push_front(ss.str());
      //printf("   %s\n", ss.str().c_str());
   }
   //pop the returned function from the callstack
   else if (ar->event ==LUA_HOOKRET) {

      if (_stack.size()>0)
      {
	 const std::string& ss = _stack.front();
	 //printf("   END %s\n", ss.c_str());
      }
   }
}

static int reportError(lua_State *luaS)
{
   const char *errmsg = lua_tostring(luaS, -1);
   printf("\nError: %s\n", errmsg ? errmsg : "<no message>");

   //lua_getglobal(luaS, "debug");
   //lua_getfield(luaS, -1, "traceback");
   //lua_remove(luaS, -2);
   //// lua_getglobal(luaS, "_TRACEBACK");
   //lua_pcall(luaS, 0, 0, 0);

   printf("Stack size: %d\n", _stack.size());
   typeof(_stack.begin()) it;
   for(it = _stack.begin(); it != _stack.end(); it++) {
      printf("   %s\n", it->c_str());
   }

   return 0;
}

 
void CLuaGlScript::CScript::initLuaState()
{
   if (! luaS) {
      luaS = lua_open();

      // luaL_openlibs(L); we don't want io, so we open libs one by one:
      // luaopen_io(L);
      luaopen_base(luaS);
      luaopen_table(luaS);
      luaopen_string(luaS);
      luaopen_math(luaS);
#if 0
      luaopen_debug(luaS);
      lua_sethook(luaS, &FunctionHook, LUA_MASKCALL | LUA_MASKRET, 0);
#endif

      // application specific bindings
      tolua_gl_open(luaS);
      tolua_glu_open(luaS);
      tolua_v11n_open(luaS);

      lua_register(luaS, "_ALERT", reportError);
      lua_atpanic(luaS, reportError);

      // load some utility scripts
      loadScript(luacode_displist_lua);

   }
}

int CLuaGlScript::CScript::loadScript(const char* pscript)
{
   if (!luaS) initLuaState();
   if (luaS) {
      //long long t0 = gethrtime();
      int rv = luaL_loadstring(luaS, (char*) pscript);
      //long long t1 = gethrtime();
      //double dt = (t1 - t0) * 1e-6;
      //printf(" ******** Time to loadstring: %lf\n", dt);
      if (rv != 0)
	 printf(" ***** luaL_loadstring FAILED\n");
      else {
         //t0 = gethrtime();
         rv = lua_pcall(luaS, 0, 0, 0);
         //t1 = gethrtime();
         //dt = (t1 - t0) * 1e-6;
         //printf(" ******** Time to pcall: %lf\n", dt);
         if (rv != 0) {
            // TODO CScript needs an ID(object, part) so it can be printed
            printf(" ***** Problem executing script (error %d):\n   %s\n", rv, lua_tostring(luaS, -1));
         }
      }
      return rv;
   }
   return -1;
}

int CLuaGlScript::CScript::exec()
{
   _stack.clear();
   if (!luaS) return -1;
   // long long t0 = gethrtime();
   lua_getfield(luaS, LUA_GLOBALSINDEX, "render");
   int rv = lua_pcall(luaS, 0, 0, 0);
   //long long t1 = gethrtime();
   //double dt = (t1 - t0) * 1e-6;
   //printf(" ******** Time to exec.pcall: %lf\n", dt);
   if (rv != 0) {
      // TODO CScript needs an ID(object, part) so it can be printed
      printf(" ***** Problem executing render() (error %d):\n   %s\n", rv, lua_tostring(luaS, -1));

      printf("Stack size: %d\n", _stack.size());
      typeof(_stack.begin()) it;
      for(it = _stack.begin(); it != _stack.end(); it++) {
	 printf("   %s\n", it->c_str());
      }
   }
   return rv;
}

CLuaGlScript::CLuaGlScript()
{
}

CLuaGlScript::~CLuaGlScript()
{
   CScript* pModel;
   IceUtil::RWRecMutex::WLock lock(_objectMutex);
   FOR_EACH_V(pModel, m_Models) {
      if (pModel) delete pModel;
   }
   m_Models.erase(m_Models.begin(), m_Models.end());
}

void CLuaGlScript::loadScript(const std::string& partId, const std::string& script)
{
   CScript* pModel = NULL;
   IceUtil::RWRecMutex::WLock lock(_objectMutex);
   if (m_Models.find(partId)->second != NULL) {
      pModel = m_Models[partId];
   }

   if (pModel == NULL) {
      pModel = new CScript();
      m_Models[partId] = pModel;
   }

   try {
      int rv = pModel->loadScript(script.c_str());
   }
   catch (...) {
   }
}

void CLuaGlScript::removePart(const std::string& partId)
{
   typeof(m_Models.begin()) it = m_Models.find(partId);
   if (it->second != NULL) {
      IceUtil::RWRecMutex::WLock lock(_objectMutex);
      CScript* pModel = m_Models[partId];
      m_Models.erase(it);
      if (pModel) delete pModel;
   }
}

ERenderContext CLuaGlScript::getPreferredContext()
{
   return ContextGL;
}

CRenderer* CLuaGlScript::getRenderer(ERenderContext context)
{
   switch(context) {
      case ContextGL: return renderGL.get();
      default: break;
   }
   return NULL;
}

void CLuaGlScript::setPose3D(const std::string& partId, const std::vector<double>& position,
      const std::vector<double>& rotation)
{
   // From TomGine
   //assert(position.size() == 3);
   //assert(rotation.size() == 4);

   //TomGine::tgRenderModel* pModel = NULL;
   //if (m_Models.find(partId)->second != NULL) {
   //   pModel = m_Models[partId];
   //}
   //if (! pModel) return;

   //pModel->m_pose.pos.x = position[0];
   //pModel->m_pose.pos.y = position[1];
   //pModel->m_pose.pos.z = position[2];
   //pModel->m_pose.q.x = rotation[0];
   //pModel->m_pose.q.y = rotation[1];
   //pModel->m_pose.q.z = rotation[2];
   //pModel->m_pose.q.w = rotation[3];
}

void CLuaGlScript_RenderGL::draw(CDisplayObject *pObject, void *pContext)
{
   DTRACE("CLuaGlScript_RenderGL::draw");
   if (pObject == NULL) return;
   CLuaGlScript *pModel = dynamic_cast<CLuaGlScript*>(pObject);
   if (pModel == NULL) return;
   if (pModel->m_Models.size() < 1) return;
   DMESSAGE("Models present.");

   CLuaGlScript::CScript* pPart;
   // Prevent script modification while executing
   IceUtil::RWRecMutex::RLock lock(pObject->_objectMutex);
   FOR_EACH_V(pPart, pModel->m_Models) {
      if (!pPart) continue;
      glPushMatrix();
      pPart->exec();
      glPopMatrix();
   }
}

}} // namespace

// A test of rendering speed.
// Client and server are running on the same machine (Intel Core 2 6600 2.4GHz)
// A client produces 2500 points, and generates a string with glVertex for each point:
//    function render()
//      glBegin(GL_POINTS)
//      glVertex(...)
//      ...
//      glEnd()
//    end
// The string is passed to the server (loadScript) and executed when needed (exec).
//
// Times:
//   Client:
//     4.8ms    : create string
//    10.2ms    : transport string (this includes luaL_loadstring and pcall)
//   Server:
//     8.5ms    : luaL_loadstring
//     0.03ms   : pcall (the string is parsed)
//     0.85ms   : exec.pcall (call to render())
