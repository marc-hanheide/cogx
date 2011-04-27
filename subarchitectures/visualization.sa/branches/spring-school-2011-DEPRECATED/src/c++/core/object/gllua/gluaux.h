/* gluaux.c
** Auxiliary function to bind GLU to Lua.
** Waldemar Celes
** Sep 1998
*/

#ifndef gluaux_h
#define gluaux_h

#include "tolua++.h"

GLUtriangulatorObj* gluauxNewTess (void);
void gluauxDeleteTess (GLUtriangulatorObj* tessobj);
void gluauxTessCallback(GLUtriangulatorObj* tessobj, GLenum type, lua_Number fn);
void gluauxBeginPolygon (GLUtriangulatorObj* tessobj);
// void gluauxTessVertex (GLUtriangulatorObj* tessobj, GLdouble v[3], lua_Number data);
void gluauxNextContour (GLUtriangulatorObj* tessobj, GLenum type);
void gluauxEndPolygon (GLUtriangulatorObj* tessobj);

#endif
