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
}
#include "tolua++.h"
#include "glbind.h"
#include "glubind.h"

#ifdef DEBUG_TRACE
#undef DEBUG_TRACE
#endif
#include "../convenience.hpp"

namespace cogx { namespace display {

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

void CLuaGlScript::CScript::initLuaState()
{
   if (! luaS) {
      luaS = lua_open();
      tolua_gl_open(luaS);
      tolua_glu_open(luaS);
   }
}

int CLuaGlScript::CScript::loadScript(const char* pscript)
{
   if (!luaS) initLuaState();
   if (luaS) {
      int rv = luaL_loadstring(luaS, (char*) pscript);
      if (rv != 0) printf("luaL_loadstring FAILED\n");
      else {
         rv = lua_pcall(luaS, 0, 0, 0);
         if (rv != 0) {
            // TODO CScript needs an ID(object, part) so it can be printed
            printf("Problem executing script (error %d):\n   %s\n", rv, lua_tostring(luaS, -1));
         }
      }
      return rv;
   }
   return -1;
}

int CLuaGlScript::CScript::exec()
{
   if (!luaS) return -1;
   lua_getfield(luaS, LUA_GLOBALSINDEX, "render");
   int rv = lua_pcall(luaS, 0, 0, 0);
   if (rv != 0) {
      // TODO CScript needs an ID(object, part) so it can be printed
      printf("Problem executing render() (error %d):\n   %s\n", rv, lua_tostring(luaS, -1));
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

bool CLuaGlScript::is3D()
{
   return true;
}

CRenderer* CLuaGlScript::getRenderer(ERenderContext context)
{
   switch(context) {
      case ContextGL: return renderGL.get();
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
   CLuaGlScript *pModel = (CLuaGlScript*) pObject;
   if (pModel->m_Models.size() < 1) return;
   DMESSAGE("Models present.");

   CLuaGlScript::CScript* pPart;
   // Prevent script modification while executing
   IceUtil::RWRecMutex::RLock lock(pObject->_objectMutex);
   FOR_EACH_V(pPart, pModel->m_Models) {
      if (!pPart) continue;
      pPart->exec();
   }
}

}} // namespace
