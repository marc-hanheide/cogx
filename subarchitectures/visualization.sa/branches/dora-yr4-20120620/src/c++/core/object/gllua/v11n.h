/*
 * @author:  Marko Mahnič
 * @created: jun 2010 
 */
   
#ifndef V11N_WWMWW49Q
#define V11N_WWMWW49Q

void* v11nGetOpenGlContext();

void v11nCamera_SetPosition(void* scriptObj, char* name,
      double xEye, double yEye, double zEye,    // camera positon
      double xView, double yView, double zView, // camera direction
      double xUp, double yUp, double zUp);      // camera orientation

void v11nGlw_RenderText(void* writerObject, double x, double y, double z, char* text, double size);

#endif /* end of include guard: V11N_WWMWW49Q */
