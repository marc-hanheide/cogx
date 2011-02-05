//SRC:
//			Title:			class tgPose
//			File:				tgPose.cpp
//
//			Function:		Representing pose of an object and applying it to GL
//
//			Author:			Thomas Mörwald
//			Date:				17.12.2009
// ----------------------------------------------------------------------------

#include "tgPose.h"

using namespace TomGine;

tgPose::tgPose(){
	tgPose(0.0);
}

tgPose::tgPose(float val){
	pos = vec3(0.0,0.0,0.0);
}

void tgPose::Activate(){
	glPushMatrix();
		glTranslatef(pos.x, pos.y, pos.z);
		glMultMatrixf(q.getMatrix4());
}

void tgPose::Deactivate(){
	glPopMatrix();
}

void tgPose::SetPose(mat3 r, vec3 p){
	// Set Pose doest not work correctly now
	q.fromMatrix(r);
		
	pos.x = p.x;
	pos.y = p.y;
	pos.z = p.z;
}

void tgPose::GetPose(mat3 &r, vec3 &p){
	
	r = q.getMatrix3();
	
	p[0] = pos.x;
	p[1] = pos.y;
	p[2] = pos.z;
}

void tgPose::Rotate(float x, float y, float z){
	tgQuaternion q2;
	
	q2.fromEuler(x,y,z);
	q = q2 * q;
	//q.normalise();
}

void tgPose::Rotate(vec3 r){
	tgQuaternion q2;
	
	q2.fromEuler(r.x,r.y,r.z);
	q = q2 * q;
	//q.normalise();
}

void tgPose::Translate(float x, float y, float z){
	pos.x += + x;
	pos.y += + y;
	pos.z += + z;
}

void tgPose::Translate(vec3 t){
	pos.x += t.x;
	pos.y += t.y;
	pos.z += t.z;
}
