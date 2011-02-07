/*
 * @author:  Marko Mahnič
 * @created: jun 2010 
 */
   
#ifndef V11N_WWMWW49Q
#define V11N_WWMWW49Q

void* v11nGetOpenGlContext();

void v11nCameraLookAt(void* scriptObj, char* name,
      double x0, double y0, double z0,      // camera positon
      double x1, double y1, double z1,      // camera direction
      double xUp, double yUp, double zUp);  // camera orientation

#endif /* end of include guard: V11N_WWMWW49Q */
