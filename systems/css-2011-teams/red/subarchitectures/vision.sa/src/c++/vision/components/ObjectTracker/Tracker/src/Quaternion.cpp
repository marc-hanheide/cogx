
#include "Quaternion.h"

using namespace Tracking;

Quaternion::Quaternion(){
	x=0.0;
	y=0.0;
	z=0.0;
	w=1.0;
}

Quaternion::Quaternion(float x, float y, float z, float w){
	this->x = x;
	this->y = y;
	this->z = z;
	this->w = w;
}

// normalising a quaternion works similar to a vector. This method will not do anything
// if the quaternion is close enough to being unit-length. define TOLERANCE as something
// small like 0.00001f to get accurate results
void Quaternion::normalise(){
	// Don't normalize if we don't have to
	float mag2 = w * w + x * x + y * y + z * z;
	if (fabs(mag2 - 1.0f) > FTOL) {
		float mag = sqrt(mag2);
		w /= mag;
		x /= mag;
		y /= mag;
		z /= mag;
	}
}

// We need to get the inverse of a quaternion to properly apply a quaternion-rotation to a vector
// The conjugate of a quaternion is the same as the inverse, as long as the quaternion is unit-length
Quaternion Quaternion::getConjugate(){
	return Quaternion(-x, -y, -z, w);
}

// Adding
Quaternion Quaternion::operator+ (const Quaternion &q2){
	Quaternion rq;
	rq.x = x+q2.x;
	rq.y = y+q2.y;
	rq.z = z+q2.z;
	rq.w = w+q2.w;
	
	return rq;
}


// Multiplying q1 with q2 applies the rotation q2 to q1
Quaternion Quaternion::operator* (const Quaternion &rq){
	// the constructor takes its arguments as (x, y, z, w)
	return Quaternion(w * rq.x + x * rq.w + y * rq.z - z * rq.y,
	                  w * rq.y + y * rq.w + z * rq.x - x * rq.z,
	                  w * rq.z + z * rq.w + x * rq.y - y * rq.x,
	                  w * rq.w - x * rq.x - y * rq.y - z * rq.z);
}

Quaternion Quaternion::operator* (const float f){
	return Quaternion(x*f, y*f, z*f, w*f);
}


// Multiplying a quaternion q with a vector v applies the q-rotation to v
vec3 Quaternion::operator* (const vec3 &vec){
	vec3 vn(vec);
	vn.normalize();
 
	Quaternion vecQuat, resQuat;
	vecQuat.x = vn.x;
	vecQuat.y = vn.y;
	vecQuat.z = vn.z;
	vecQuat.w = 0.0f;
 
	resQuat = vecQuat * getConjugate();
	resQuat = *this * resQuat;
 
	return (vec3(resQuat.x, resQuat.y, resQuat.z));
}

// Convert from Axis Angle
void Quaternion::fromAxis(const vec3 &v, float angle){
	float sinAngle;
	angle *= 0.5f;
	vec3 vn(v);
	vn.normalize();
 
	sinAngle = sin(angle);
 
	x = (vn.x * sinAngle);
	y = (vn.y * sinAngle);
	z = (vn.z * sinAngle);
	w = cos(angle);
}

// Convert from Euler Angles
void Quaternion::fromEuler(float roll, float pitch, float yaw){
	// Basically we create 3 Quaternions, one for pitch, one for yaw, one for roll
	// and multiply those together.
	// the calculation below does the same, just shorter
 
	float p = pitch / 2.0;
	float y = yaw / 2.0;
	float r = roll / 2.0;
 
	float sinp = sin(p);
	float siny = sin(y);
	float sinr = sin(r);
	float cosp = cos(p);
	float cosy = cos(y);
	float cosr = cos(r);
 
	this->x = sinr * cosp * cosy - cosr * sinp * siny;
	this->y = cosr * sinp * cosy + sinr * cosp * siny;
	this->z = cosr * cosp * siny - sinr * sinp * cosy;
	this->w = cosr * cosp * cosy + sinr * sinp * siny;
 
	normalise();
}

// Convert from Matrix 4x4
void Quaternion::fromMatrix(mat4 m){
	w = sqrt(1.0 + m[0] + m[5] + m[10]) / 2.0;
	float w4 = (4.0 * w);
	x = (m[9] - m[6]) / w4 ;
	y = (m[2] - m[8]) / w4 ;
	z = (m[4] - m[1]) / w4 ;
	
	normalise();
}

// Convert from Matrix 3x3
void Quaternion::fromMatrix(mat3 m){
	w = sqrt(1.0 + m[0] + m[4] + m[8]) / 2.0;
	float w4 = (4.0 * w);
	x = (m[7] - m[5]) / w4 ;
	y = (m[2] - m[6]) / w4 ;
	z = (m[3] - m[1]) / w4 ;
	
	normalise();
}

// Convert to Matrix 4x4
mat4 Quaternion::getMatrix4(){
	float x2 = x * x;
	float y2 = y * y;
	float z2 = z * z;
	float xy = x * y;
	float xz = x * z;
	float yz = y * z;
	float wx = w * x;
	float wy = w * y;
	float wz = w * z;
 
	mat4 rot;
	rot[0]=1.0f - 2.0f * (y2 + z2);	rot[1]=2.0f * (xy - wz);		rot[2]=2.0f * (xz + wy);			rot[3]=0.0f;
	rot[4]=2.0f * (xy + wz); 		rot[5]=1.0f - 2.0f * (x2 + z2);	rot[6]=2.0f * (yz - wx);			rot[7]=0.0f;
	rot[8]=2.0f * (xz - wy);		rot[9]=2.0f * (yz + wx);		rot[10]=1.0f - 2.0f * (x2 + y2);	rot[11]=0.0f;
	rot[12]=0.0f;					rot[13]=0.0f;					rot[14]=0.0f;						rot[15]=1.0f;
	
	return rot;
}

// Convert to Matrix 3x3
mat3 Quaternion::getMatrix3(){
	float x2 = x * x;
	float y2 = y * y;
	float z2 = z * z;
	float xy = x * y;
	float xz = x * z;
	float yz = y * z;
	float wx = w * x;
	float wy = w * y;
	float wz = w * z;
	
	mat3 rot;
	rot[0]=1.0f - 2.0f * (y2 + z2);	rot[1]=2.0f * (xy - wz);		rot[2]=2.0f * (xz + wy);
	rot[3]=2.0f * (xy + wz); 		rot[4]=1.0f - 2.0f * (x2 + z2);	rot[5]=2.0f * (yz - wx);
	rot[6]=2.0f * (xz - wy);		rot[7]=2.0f * (yz + wx);		rot[8]=1.0f - 2.0f * (x2 + y2);
	
	return rot;
}


// Convert to Axis/Angles
void Quaternion::getAxisAngle(vec3 &axis, double &angle){
	float scale = sqrt(x * x + y * y + z * z);
	axis.x = x / scale;
	axis.y = y / scale;
	axis.z = z / scale;
	angle = acos(w) * 2.0f;
	if(angle < 0.0 || angle > (2.0*PI))
		angle = 0.0;
}

