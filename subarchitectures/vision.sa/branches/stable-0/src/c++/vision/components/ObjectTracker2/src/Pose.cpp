
#include "Pose.h"

Pose::Pose(){
	Pose(0.0);
}

Pose::Pose(float val){
	rX = val;
	rY = val;
	rZ = val;
	
	tX = val;
	tY = val;
	tZ = val;
	
	q = Quaternion();
	
	w = val;
}

Pose::Pose(const Pose& p2){
	rX = p2.rX;
	rY = p2.rY;
	rZ = p2.rZ;
	
	tX = p2.tX;
	tY = p2.tY;
	tZ = p2.tZ;
	
	q = p2.q;
	
	w = p2.w;
}

Pose& Pose::operator=(const Pose& p2){
	rX = p2.rX;
	rY = p2.rY;
	rZ = p2.rZ;
	
	tX = p2.tX;
	tY = p2.tY;
	tZ = p2.tZ;
	
	q = p2.q;
	
	w = p2.w;
}

Pose& Pose::operator+(const Pose& p2){

}

void Pose::activateGL(){
	glPushMatrix();
		glTranslatef(tX, tY, tZ);
		glMultMatrixf(q.getMatrix4());
}

void Pose::deactivateGL(){
	glPopMatrix();
}

void Pose::print(){
	printf("r: %f %f %f\n", rX, rY, rZ);
	printf("t: %f %f %f\n", tX, tY, tZ);
	printf("w: %f\n", w);
}

void Pose::setPose(mat3 rot, vec3 pos){
	// Set Pose doest not work correctly now
	q.fromMatrix(rot);
		
	tX = pos.x;
	tY = pos.y;
	tZ = pos.z;
	
	w = 0.0;
	
	//printf("[Pose::setPose] Warning conversion from rotation matrix to quaternion not correct!\n");
}

void Pose::getPose(mat3 &rot, vec3 &pos){
	
	rot = q.getMatrix3();
	
	pos[0] = tX;
	pos[1] = tY;
	pos[2] = tZ;
}

void Pose::rotate(float x, float y, float z){
	Quaternion q2;
	
	q2.fromEuler(x,y,z);
	q = q * q2;
	q.normalise();
}

void Pose::translate(float x, float y, float z){
	tX = tX + x;
	tY = tY + y;
	tZ = tZ + z;
}
	
