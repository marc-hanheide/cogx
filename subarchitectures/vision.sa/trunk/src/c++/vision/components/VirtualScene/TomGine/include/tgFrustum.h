 /**
 * @file tgFrustum.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief View frustum of a camera.
 */

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
