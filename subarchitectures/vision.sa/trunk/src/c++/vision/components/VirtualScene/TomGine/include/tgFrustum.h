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

namespace TomGine{

/**
* @brief Class tgFrustum
*/
class tgFrustum{
    
private:
	float frustum[6][4];
	float m_proj[16];
	float m_modl[16];


public:
	void ExtractFrustum();
	bool PointInFrustum( float x, float y, float z );
	bool SphereInFrustum( float x, float y, float z, float radius );
	void DrawFrustum();

};

} // namespace TomGine

#endif
