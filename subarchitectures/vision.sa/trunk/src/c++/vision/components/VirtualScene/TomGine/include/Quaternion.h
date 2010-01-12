 /**
 * @file Quaternion.h
 * @author Thomas Mörwald
 * @date September 2009
 * @version 0.1
 * @brief Quaternion representing rotations (avoiding singularity locks)
 */
 
#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>
#include "mathlib.h"

#ifndef FTOL
#define FTOL 0.0001
#endif
#ifndef PIOVER180
#define PIOVER180 PI/180.0
#endif

class Quaternion
{
private:
	
	
public:
	float x,y,z,w;
	Quaternion();
	Quaternion(float x, float y, float z, float w);
	
	void normalise();
	Quaternion getConjugate();
	Quaternion operator+ (const Quaternion &q2);
	Quaternion operator* (const Quaternion &rq);
	Quaternion operator* (const float f);
	
	vec3 operator* (const vec3 &vec);
	
	
	void fromAxis(const vec3 &v, float angle);
	void fromEuler(float pitch, float yaw, float roll);
	void fromMatrix(mat3 m);
	void fromMatrix(mat4 m);
	mat4 getMatrix4();
	mat3 getMatrix3();
	void getAxisAngle(vec3 *axis, double *angle);

};

#endif