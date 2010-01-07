//HEADER:
//			Title:			class tgPose
//			File:				tgPose.h
//
//			Function:		Header file for pose of an object
//
//			Author:			Thomas Mörwald
//			Date:				17.12.2009
// ----------------------------------------------------------------------------

#ifndef TG_POSE
#define TG_POSE

#include <GL/gl.h>
#include "mathlib.h"
#include "Quaternion.h"

class tgPose{
public:
	vec3 pos;
	Quaternion q;
	
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

#endif