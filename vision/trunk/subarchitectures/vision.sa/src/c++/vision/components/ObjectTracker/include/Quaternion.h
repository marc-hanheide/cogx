//HEADER:
//			Title:			class Quaternion
//			File:				Quaternion.h
//
//			Function:		Header file for Quaternions for rotations in OpenGL
//			From:			http://gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
//
//			Author:			Thomas Mörwald
//			Date:				17.06.2009
// ----------------------------------------------------------------------------

#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

#include "mathlib.h"

#define FTOL 0.0001
#define PIOVER180 PI/180.0

class Quaternion
{
private:
	
	
public:
	float x,y,z,w;
	Quaternion();
	Quaternion(float x, float y, float z, float w);
	
	void normalise();
	Quaternion getConjugate();
	Quaternion operator* (const Quaternion &rq);
	vec3 operator* (const vec3 &vec);
	
	void FromAxis(const vec3 &v, float angle);
	void FromEuler(float pitch, float yaw, float roll);
	mat4 getMatrix();
	void getAxisAngle(vec3 *axis, float *angle);

};

#endif