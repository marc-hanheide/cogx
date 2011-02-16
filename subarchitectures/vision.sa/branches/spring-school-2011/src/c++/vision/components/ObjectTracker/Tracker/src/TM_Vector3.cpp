//CPP:
//			Title:			class TM_Vector3
//			File:				TM_Vector3.cpp
//
//			Function:		3D Vector with all necessary operations 
//
//			Author:			Thomas M�rwald
//			Date:				10.01.2007
// ----------------------------------------------------------------------------
#include "TM_Vector3.h"

using namespace Tracking;

TM_Vector3::TM_Vector3()
{
	TM_Vector3(0.0f);
	
}
//TM_Vector3::TM_Vector3(const TM_Vector3 &v){
//	TM_Vector3(v.x,v.y,v.z);
//}
TM_Vector3::TM_Vector3(float all)
{
	TM_Vector3(all,all,all);
	
}
TM_Vector3::TM_Vector3(float xn, float yn, float zn)
{
	x = xn; y = yn; z = zn;
}


TM_Vector3 TM_Vector3::operator+(TM_Vector3 v)
{
	return TM_Vector3(x+v.x, y+v.y, z+v.z);
}

TM_Vector3 TM_Vector3::operator-(TM_Vector3 v)
{
	return TM_Vector3(x-v.x, y-v.y, z-v.z);
}

TM_Vector3 TM_Vector3::operator*(TM_Vector3 v)
{
	return TM_Vector3(x*v.x, y*v.y, z*v.z);
}

TM_Vector3 TM_Vector3::operator*(float m)
{
	return TM_Vector3(x*m, y*m, z*m);
}

TM_Vector3 TM_Vector3::operator/(float m)
{
	return TM_Vector3(x/m, y/m, z/m);
}

TM_Vector3 TM_Vector3::cross(TM_Vector3 v)
{
	return TM_Vector3(	y*v.z - z*v.y, 
					z*v.x - x*v.z,
					x*v.y - y*v.x);
}

TM_Vector3 TM_Vector3::cross(TM_Vector3 v1, TM_Vector3 v2)
{
	return TM_Vector3(	v1.y*v2.z - v1.z*v2.y, 
					v1.z*v2.x - v1.x*v2.z,
					v1.x*v2.y - v1.y*v2.x);
}

float TM_Vector3::dot(TM_Vector3 v)
{
	return (x*v.x + y*v.y + z*v.z);
}

void TM_Vector3::normalize()
{
	float s = sqrt(x*x + y*y + z*z);
	if(s != 0.0f)
	{
		x /= s;
		y /= s;
		z /= s;
	}
}

float TM_Vector3::length()
{
	return sqrt(x*x + y*y + z*z);
}

void TM_Vector3::setLength(float l)
{
	normalize();
	x *= l;
	y *= l;
	z *= l;	
}

void TM_Vector3::rotateX(float fAngle)
{
	float tz = z;
	float ty = y;
	y = ty * cos(fAngle) - tz * sin(fAngle);
	z = ty * sin(fAngle) + tz * cos(fAngle);
}

void TM_Vector3::rotateY(float fAngle)
{	
	float tz = z;
	float tx = x;
	z = tz * cos(fAngle) - tx * sin(fAngle);
	x = tz * sin(fAngle) + tx * cos(fAngle);
}

void TM_Vector3::rotateZ(float fAngle)
{
	float tx = x;
	float ty = y;
	x = tx * cos(fAngle) - ty * sin(fAngle);
	y = tx * sin(fAngle) + ty * cos(fAngle);
}
/*
void TM_Vector3::rotate(float fAngle, TM_Vector3 vAxis) {
	float h,r;
	TM_Vector3 b1, b2, b3;
	TM_Vector3 vResult;
	TM_Vector3 vThis(x,y,z);

	//
	b1 = vAxis;
	vAxis.normalize();
	h = b1.dot(vThis);

	b2 = b1.cross(vThis); //vektor
	r = b2.length();
	b2.normalize();

	b3 = b2.cross(vAxis);
	b3.normalize();

	vResult = b1*h + b3*r*cos(fAngle) + b2*r*sin(fAngle);

	vResult = vResult * vThis.length() / vResult.length();
	x = vResult.x;
	y = vResult.y;
	z = vResult.z;
}
*/

void TM_Vector3::rotate(float alpha, TM_Vector3 r) {
	
	TM_Vector3 v,s,t,n;
	TM_Matrix3 M,Mt,Rx,X;
	
	if(alpha != 0){

		r.normalize();
		s = r.cross(TM_Vector3(1,0,0));
		
		if(s.length() < FLOAT_NULL)
			s = r.cross(TM_Vector3(0,1,0));
		
		s.normalize();
		t = r.cross(s);
		
		Mt = TM_Matrix3(r,s,t);
		M = TM_Matrix3(Mt);
		M.transpose();	
		
		Rx = TM_Matrix3(	1.0f,	0.0f,	0.0f,
						0.0f,	cos(alpha),	-sin(alpha),
						0.0f,	sin(alpha),	cos(alpha));
						
		X=Mt*Rx*M;

		v = TM_Vector3(x,y,z);
		n = X*v;
		
		x = n.x;
		y = n.y;
		z = n.z;
	}
}

/*
void TM_Vector3::rotate(float a, TM_Vector3 r){
	TM_Vector3 v,n;
	
	TM_Matrix3 R = TM_Matrix3(	
		cos(a)+(1-cos(a))*r.x*r.x,	(1-cos(a))*r.x*r.y-r.z*sin(a),	(1-cos(a))*r.x*r.z+r.y*sin(a),
		(1-cos(a))*r.x*r.y+r.z*sin(a),	cos(a)+(1-cos(a))*r.y*r.y,	(1-cos(a))*r.y*r.z-r.x*sin(a),	
		(1-cos(a))*r.x*r.z-r.y*sin(a),	(1-cos(a))*r.y*r.z+r.x*sin(a),	cos(a)+(1-cos(a))*r.z*r.z);
	
	v = TM_Vector3(x,y,z);
	n = R*v;
	
	x = n.x;
	y = n.y; 
	z = n.z;
}
*/

/*
void TM_Vector3::GetRotYZ(float &fAngleY, float &fAngleZ)
{
	TM_Vector3 v = *this;
	v.normalize();
	fAngleY = 0.0f; //abs(asin(v.z));
	fAngleZ = 0.0f; //asin(v.y/(cos(fAngleY)));

	if(x>0.0f)
		if(z>0.0f)	// 4
			fAngleY = (float)PI*2 - fAngleY ;
		else				// 1
			fAngleY = fAngleY;
	else
		if(z>0.0f)	// 3
			fAngleY = (float)PI + fAngleY;
		else				// 2
			fAngleY = (float)PI - fAngleY;
}
*/

float Angle(TM_Vector3 a, TM_Vector3 b)
{
	float d = a.dot(b)/(a.length() * b.length());
	return acos(d);
}



