// Frustum Culling
// http://robertmarkmorley.com/2008/11/16/frustum-culling-in-opengl/
// © 2009 - Robert M. Morley

#ifndef TG_FRUSTUM
#define TG_FRUSTUM

#include<GL/gl.h>
#include<math.h>

class tgFrustum{
    
private:
    float frustum[6][4];


public:
    tgFrustum();
    ~tgFrustum();
    
    void ExtractFrustum();
    bool PointInFrustum( float x, float y, float z );
    bool SphereInFrustum( float x, float y, float z, float radius );

};

#endif
