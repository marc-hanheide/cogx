
#ifndef __POSE_H__
#define __POSE_H__

class Pose;
#include "Quaternion.h"
#include "mathlib.h"

#include <stdio.h>
//#include <stdlib.h>
//#include <algorithm>
#include <GL/gl.h>

class Pose
{
public:
	float rX, rY, rZ;	// rotation
	float tX, tY, tZ;	// translation
	float w;			// Likelihood
	//float v, d;
	Quaternion q;		// representing rotation with quaternions

	Pose();
	Pose(float val);
	Pose(const Pose& p2);
	
	Pose& operator=(const Pose& p2);
	Pose& operator+(const Pose& p2);
	
	inline bool operator<(const Pose& p2) const { return w < p2.w; }
	inline bool operator>(const Pose& p2) const { return w > p2.w; }
	
	void activateGL();
	void deactivateGL();
	
	void print();
	//void getModelView(float* matrix4x4);
	void setPose(mat3 rot, vec3 pos);
	void getPose(mat3 &rot, vec3 &pos);
	
	void rotate(float x, float y, float z);
	void translate(float x, float y, float z);

};

#endif