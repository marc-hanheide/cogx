
#include "v11n.h"
#include "../CLuaGlScript.hpp"
#include <QGLContext>

void* v11nGetOpenGlContext()
{
   return (void*) QGLContext::currentContext();
}

void v11nCameraLookAt(void* scriptObj, char* name,
      double x0, double y0, double z0,      // camera positon
      double x1, double y1, double z1,      // camera direction
      double xUp, double yUp, double zUp)   // camera orientation
{
   // TODO: check if scriptObj is valid
   cogx::display::CLuaGlScript* pScript = (cogx::display::CLuaGlScript*)scriptObj;
   if (! pScript) return;
   pScript->setCamera(name, x0, y0, z0, x1, y1, z1, xUp, yUp, zUp);
}
