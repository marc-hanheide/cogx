 /**
 * @file tgPose.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Defining the position and orientation of an object in 3D.
 */
 
#ifndef TG_POSE
#define TG_POSE

#include <GL/gl.h>

#include "tgMathlib.h"
#include "tgQuaternion.h"

namespace TomGine{

/**
* @brief Class tgPose
*/
class tgPose{
public:
	vec3 pos;
	tgQuaternion q;
	
public:
	tgPose();
	tgPose(float val);

	void Activate();	
	void Deactivate();
	
	void SetPose(mat3 r, vec3 p);	
	void GetPose(mat3 &r, vec3 &p);
	
	void Rotate(float x, float y, float z);	
	void Rotate(vec3 r);	
	void Translate(float x, float y, float z);	
	void Translate(vec3 t);	
};

} // namespace TomGine

#endif