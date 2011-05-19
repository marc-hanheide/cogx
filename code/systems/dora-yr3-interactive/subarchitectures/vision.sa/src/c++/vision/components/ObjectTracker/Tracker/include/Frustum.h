// Frustum Culling
// http://robertmarkmorley.com/2008/11/16/frustum-culling-in-opengl/
// © 2009 - Robert M. Morley

#ifndef FRUSTUM_H
#define FRUSTUM_H

#include "headers.h"

class Frustum{
    
private:
    float frustum[6][4];


public:
    Frustum();
    ~Frustum();
    
    void ExtractFrustum();
    bool PointInFrustum( float x, float y, float z );
    bool SphereInFrustum( float x, float y, float z, float radius );

};

#endif
