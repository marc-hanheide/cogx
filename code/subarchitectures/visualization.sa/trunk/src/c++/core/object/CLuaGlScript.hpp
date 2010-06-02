/*
 * Author: Marko Mahnič
 * Created: 2010-06-02
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
   virtual bool is3D(); /*override*/
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
