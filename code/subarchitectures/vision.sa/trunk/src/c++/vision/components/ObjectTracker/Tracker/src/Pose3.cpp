
#include "Pose3.h"

void Pose3::activate(){
	glPushMatrix();
		glTranslatef(t.x, t.y, t.z);
		glMultMatrixf(q.getMatrix4());
}

void Pose3::deactivate(){
	glPopMatrix();
}

void Pose3::print(){
	printf("t: %f %f %f, q: %f %f %f %f\n", t.x, t.y, t.z, q.x, q.y, q.z, q.w);
}

void Pose3::setPose(mat3 rot, vec3 pos){
	q.fromMatrix(rot);
	q.normalise();
		
	t.x = pos.x;
	t.y = pos.y;
	t.z = pos.z;
}

void Pose3::getPose(mat3 &rot, vec3 &pos){
	
	rot = q.getMatrix3();
	
	pos.x = t.x;
	pos.y = t.y;
	pos.z = t.z;
}

void Pose3::rotate(float x, float y, float z){
	Quaternion q2;
	
	q2.fromEuler(x,y,z);
	q = q2 * q;
	q.normalise();
}

void Pose3::rotate(vec3 rot){
	Quaternion q2;
	
	q2.fromEuler(rot.x,rot.y,rot.z);
	q = q2 * q;
	q.normalise();
}

void Pose3::translate(float x, float y, float z){
	t.x = t.x + x;
	t.y = t.y + y;
	t.z = t.z + z;
}

void Pose3::translate(vec3 trans){
	t.x = t.x + trans.x;
	t.y = t.y + trans.y;
	t.z = t.z + trans.z;
}
