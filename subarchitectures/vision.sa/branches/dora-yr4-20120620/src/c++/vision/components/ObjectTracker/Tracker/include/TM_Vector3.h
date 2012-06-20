//HEADER:
//			Title:			class TM_Vector3
//			File:				TM_Vector3.h
//
//			Function:		3D Vector with all necessary operations 
//
//			Author:			Thomas Mörwald
//			Date:				10.01.2007
// ----------------------------------------------------------------------------
#ifndef TM_VECTOR3_H
#define TM_VECTOR3_H

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define FLOAT_NULL 0.01

namespace Tracking{
	class TM_Vector3;
}

#include <math.h>
#include "TM_Matrix3.h"

namespace Tracking{

class TM_Vector3
{
public:
	float x, y, z;
	
	TM_Vector3();
	//TM_Vector3(const TM_Vector3 &v);
	TM_Vector3(float all);
	TM_Vector3(float x, float y, float z);

	TM_Vector3 operator+(TM_Vector3 v);
	TM_Vector3 operator-(TM_Vector3 v);
	TM_Vector3 operator*(TM_Vector3 v);
	TM_Vector3 operator*(float m);
	TM_Vector3 operator/(float m);
	
	TM_Vector3 cross(TM_Vector3 v);
	static TM_Vector3 cross(TM_Vector3 v1, TM_Vector3 v2);
	float dot(TM_Vector3 v);
	void normalize();
	float length();
	void setLength(float l);

	void rotateX(float fAngle);
	void rotateY(float fAngle);
	void rotateZ(float fAngle);

	void rotate(float fAngle, TM_Vector3 vAxis);

	//void GetRotYZ(float &fAngleY, float &fAngleZ);

};

// TM_Vector3 funktions
float Angle(TM_Vector3 a, TM_Vector3 b);

} // namespace Tracking

#endif
