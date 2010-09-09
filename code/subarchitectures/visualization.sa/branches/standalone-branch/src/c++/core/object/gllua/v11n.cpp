
#include "v11n.h"
#include <QGLContext>

void* v11nGetOpenGlContext()
{
   return (void*) QGLContext::currentContext();
}
