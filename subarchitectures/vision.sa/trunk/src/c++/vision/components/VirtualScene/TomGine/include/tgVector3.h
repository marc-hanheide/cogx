//HEADER:
//			Title:			class tgVector3
//			File:				tgVector3.h
//
//			Function:		3D Vector with all necessary operations 
//
//			Author:			Thomas Mörwald
//			Date:				10.01.2007
// ----------------------------------------------------------------------------
#ifndef TG_VECTOR3
#define TG_VECTOR3

#ifndef PI
#define PI 3.14159265358979323846f
#endif

class tgVector3;

#include <math.h>
#include "tgMatrix3.h"

class tgVector3
{
public:
	float x, y, z;
	
	tgVector3();
	//tgVector3(const tgVector3 &v);
	tgVector3(float all);
	tgVector3(float x, float y, float z);

	tgVector3 operator+(tgVector3 v);
	tgVector3 operator-(tgVector3 v);
	tgVector3 operator*(tgVector3 v);
	tgVector3 operator*(float m);
	tgVector3 operator/(float m);
	
	tgVector3 cross(tgVector3 v);
	static tgVector3 cross(tgVector3 v1, tgVector3 v2);
	float dot(tgVector3 v);
	void normalize();
	float length();
	void setLength(float l);

	void rotateX(float fAngle);
	void rotateY(float fAngle);
	void rotateZ(float fAngle);

	void rotate(float fAngle, tgVector3 vAxis);

	//void GetRotYZ(float &fAngleY, float &fAngleZ);

};

// tgVector3 funktions
float Angle(tgVector3 a, tgVector3 b);

#endif
