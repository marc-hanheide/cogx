//HEADER:
//			Title:			class TM_Matrix3
//			File:				TM_Matrix3.cpp
//
//			Function:		TM_Matrix3 with all necessary operations 
//
//			Author:			Thomas Mörwald
//			Date:				20.09.2007
// ---------------------------------------------------------------------------
#ifndef MATRIX3_H
#define MATRIX3_H

#ifndef PI
#define PI 3.141592
#endif

namespace Tracking{
	class TM_Matrix3;
}

#include <math.h>
#include "TM_Vector3.h"

namespace Tracking{

class TM_Matrix3
{
private:
	
public:
	float m[9];
		
	TM_Matrix3();
	TM_Matrix3(const TM_Matrix3 &m);
	TM_Matrix3(TM_Vector3 x, TM_Vector3 y, TM_Vector3 z);
	TM_Matrix3(float m0, float m1, float m2,
			float m3, float m4, float m5,
			float m6, float m7, float m8);

	TM_Matrix3 operator+(TM_Matrix3 v);
	TM_Matrix3 operator-(TM_Matrix3 v);
	TM_Matrix3 operator*(TM_Matrix3 v);
	TM_Matrix3 operator*(float m);
	TM_Vector3 operator*(TM_Vector3 v);
	
	void transpose();
};

} // namespace Tracking

#endif
