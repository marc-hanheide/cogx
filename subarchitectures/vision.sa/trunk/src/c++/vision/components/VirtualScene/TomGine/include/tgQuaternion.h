 /**
 * @file tgQuaternion.h
 * @author Thomas Mörwald
 * @date September 2009
 * @version 0.1
 * @brief tgQuaternion representing rotations (avoiding singularity locks)
 */
 
#ifndef TG_QUATERNION
#define TG_QUATERNION

#include <math.h>

#include "tgMathlib.h"

#ifndef FTOL
#define FTOL 0.0001
#endif
#ifndef PIOVER180
#define PIOVER180 PI/180.0
#endif

namespace TomGine{

/**
* @brief Class tgQuaternion
*/
class tgQuaternion
{
private:
	
	
public:
	float x,y,z,w;
	tgQuaternion();
	tgQuaternion(float x, float y, float z, float w);
	
	void normalise();
	tgQuaternion getConjugate();
	tgQuaternion operator+ (const tgQuaternion &q2);
	tgQuaternion operator* (const tgQuaternion &rq);
	tgQuaternion operator* (const float f);
	
	vec3 operator* (const vec3 &vec);
	
	
	void fromAxis(const vec3 &v, float angle);
	void fromEuler(float pitch, float yaw, float roll);
	void fromMatrix(mat3 m);
	void fromMatrix(mat4 m);
	mat4 getMatrix4();
	mat3 getMatrix3();
	void getAxisAngle(vec3 *axis, double *angle);

};

} // namespace TomGine

#endif