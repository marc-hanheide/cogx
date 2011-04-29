//CPP:
//			Title:			class TM_Matrix3
//			File:				TM_Matrix3.cpp
//
//			Function:		TM_Matrix3 with all necessary operations 
//
//			Author:			Thomas M�rwald
//			Date:				20.09.2007
// ----------------------------------------------------------------------------
#include "TM_Matrix3.h"

using namespace Tracking;

TM_Matrix3::TM_Matrix3(){
	for(int i=0; i<9; i++)
		m[i] = 0.0f;
}

TM_Matrix3::TM_Matrix3(TM_Vector3 x, TM_Vector3 y, TM_Vector3 z){
	m[0] = x.x;	m[1] = y.x;	m[2] = z.x;
	m[3] = x.y; m[4] = y.y; m[5] = z.y;
	m[6] = x.z; m[7] = y.z; m[8] = z.z;
}

TM_Matrix3::TM_Matrix3(const TM_Matrix3 &m1){
	m[0]=m1.m[0];	m[1]=m1.m[1];	m[2]=m1.m[2];
	m[3]=m1.m[3];	m[4]=m1.m[4];	m[5]=m1.m[5];
	m[6]=m1.m[6];	m[7]=m1.m[7];	m[8]=m1.m[8];
}

TM_Matrix3::TM_Matrix3(	float m0, float m1, float m2,
					float m3, float m4, float m5,
					float m6, float m7, float m8){
	m[0]=m0; m[1]=m1; m[2]=m2;
	m[3]=m3; m[4]=m4; m[5]=m5;
	m[6]=m6; m[7]=m7; m[8]=m8;

}

TM_Matrix3 TM_Matrix3::operator+(TM_Matrix3 m1){
	TM_Matrix3 m2;
	return m2;
}

TM_Matrix3 TM_Matrix3::operator-(TM_Matrix3 m1){
	TM_Matrix3 m2;
	return m2;
}

TM_Matrix3 TM_Matrix3::operator*(TM_Matrix3 m1){
	TM_Matrix3 m2;
	
	m2.m[0] = m[0]*m1.m[0]	+	m[1]*m1.m[3]	+	m[2]*m1.m[6];
	m2.m[1] = m[0]*m1.m[1]	+	m[1]*m1.m[4]	+	m[2]*m1.m[7];
	m2.m[2] = m[0]*m1.m[2]	+	m[1]*m1.m[5]	+	m[2]*m1.m[8];
	
	m2.m[3] = m[3]*m1.m[0]	+	m[4]*m1.m[3]	+	m[5]*m1.m[6];
	m2.m[4] = m[3]*m1.m[1]	+	m[4]*m1.m[4]	+	m[5]*m1.m[7];
	m2.m[5] = m[3]*m1.m[2]	+	m[4]*m1.m[5]	+	m[5]*m1.m[8];
	
	m2.m[6] = m[6]*m1.m[0]	+	m[7]*m1.m[3]	+	m[8]*m1.m[6];
	m2.m[7] = m[6]*m1.m[1]	+	m[7]*m1.m[4]	+	m[8]*m1.m[7];
	m2.m[8] = m[6]*m1.m[2]	+	m[7]*m1.m[5]	+	m[8]*m1.m[8];
	
	return m2;	
}

TM_Matrix3 TM_Matrix3::operator*(float f){
	TM_Matrix3 m2;
	return m2;
}

TM_Vector3 TM_Matrix3::operator*(TM_Vector3 v1){
	TM_Vector3 v2;
	
	v2.x = m[0]*v1.x + m[1]*v1.y + m[2]*v1.z;
	v2.y = m[3]*v1.x + m[4]*v1.y + m[5]*v1.z;
	v2.z = m[6]*v1.x + m[7]*v1.y + m[8]*v1.z;
	
	return v2;
}

void TM_Matrix3::transpose(){
	TM_Matrix3 m1 = TM_Matrix3(*this);
	
	m[0]=m1.m[0];	m[1]=m1.m[3];	m[2]=m1.m[6];
	m[3]=m1.m[1];	m[4]=m1.m[4];	m[5]=m1.m[7];
	m[6]=m1.m[2];	m[7]=m1.m[5];	m[8]=m1.m[8];
}



