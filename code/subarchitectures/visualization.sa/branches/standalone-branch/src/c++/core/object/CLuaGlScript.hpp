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
#ifndef CLUAGLSCRIPT_4K4O60J9
#define CLUAGLSCRIPT_4K4O60J9
#include "../Model.hpp"

extern "C" {
struct lua_State;
};

namespace cogx { namespace display {

class CLuaGlScript: public CDisplayObject
{
   friend class CLuaGlScript_RenderGL;
   static std::auto_ptr<CRenderer> renderGL;

   class CScript
   {
   private:
      lua_State* luaS;
   private:
      void initLuaState();
   public:
      CScript();
      ~CScript();
      int loadScript(const char* pscript); 
      int exec();
   };

public:
   std::map<std::string, CScript*> m_Models;

public:
   CLuaGlScript();
   ~CLuaGlScript();
   void loadScript(const std::string& partId, const std::string& script);
   void removePart(const std::string& partId);
   virtual ERenderContext getPreferredContext(); /*override*/
   virtual CRenderer* getRenderer(ERenderContext context); /*override*/
   virtual void setPose3D(const std::string& partId, const std::vector<double>& position,
         const std::vector<double>& rotation); /*override*/
};

class CLuaGlScript_RenderGL: public CRenderer
{
public:
   virtual void draw(CDisplayObject *pObject, void *pContext); /*override*/
};


}} // namespace
#endif /* end of include guard: CLUAGLSCRIPT_4K4O60J9 */