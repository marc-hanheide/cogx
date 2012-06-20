
#include "Pose.h"

using namespace Tracking;

void Pose::activate(){
	glPushMatrix();
		glTranslatef(t.x, t.y, t.z);
		glMultMatrixf(q.getMatrix4());
}

void Pose::deactivate(){
	glPopMatrix();
}

void Pose::print(){
	printf("t: %f %f %f, q: %f %f %f %f\n", t.x, t.y, t.z, q.x, q.y, q.z, q.w);
}

void Pose::setPose(mat3 rot, vec3 pos){
	q.fromMatrix(rot);
	q.normalise();
		
	t.x = pos.x;
	t.y = pos.y;
	t.z = pos.z;
}

void Pose::getPose(mat3 &rot, vec3 &pos){
	
	rot = q.getMatrix3();
	
	pos.x = t.x;
	pos.y = t.y;
	pos.z = t.z;
}

void Pose::rotate(float x, float y, float z){
	if(x==0.0 && y==0.0 && z ==0.0)
		return;
	
	Quaternion q2;
	
	q2.fromEuler(x,y,z);
	q = q2 * q;
	q.normalise();
}

void Pose::rotateAxis(vec3 rot){
// 	Quaternion q2;
// 	q2.fromEuler(rot.x,rot.y,rot.z);
// 	q = q2 * q;
// 	q.normalise();
	Quaternion q2;
	float a = rot.length();
	rot.normalize();
	q2.fromAxis(rot,a);
	q = q2 * q;
	q.normalise(); 
}

void Pose::rotateEuler(vec3 rot){
	Quaternion q2;
	q2.fromEuler(rot.x,rot.y,rot.z);
	q = q2 * q;
	q.normalise();
}

void Pose::translate(float x, float y, float z){
	t.x = t.x + x;
	t.y = t.y + y;
	t.z = t.z + z;
}

void Pose::translate(vec3 trans){
	t.x = t.x + trans.x;
	t.y = t.y + trans.y;
	t.z = t.z + trans.z;
}
