//HEADER:
//			Title:			class tgCamera
//			File:				tgCamera.h
//
//			Function:		Header file for Viewing tgCamera
//
//			Author:			Thomas Mörwald
//			Date:				17.06.2009
// ----------------------------------------------------------------------------
#ifndef TG_CAMERA
#define TG_CAMERA

class tgCamera;
#include <stdio.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "tgVector3.h"
#include "tgFrustum.h"
#include "mathlib.h"

#define GL_ORTHO 0
#define GL_PERSPECTIVE 1


class tgCamera
{
private:
	// tgCamera definition
	tgVector3 m_vPos;			// Position of tgCamera (absolute)
	tgVector3 m_vView;		// Viewpoint of tgCamera (absolute)
	tgVector3 m_vUp;			// The tgCameras upside (relative)
	
	tgVector3 f;		// Vector of camera pointing forward 
	tgVector3 s;		// Vector of camera pointing sidewards (right)
	tgVector3 u;		// Vector of camera pointing up
	
	float m_fovy, m_width, m_height;
	float m_zNear, m_zFar;
	unsigned short m_projection;	
	mat4 m_extrinsic;
	mat4 m_intrinsic;
	
	tgFrustum m_frustum;
	
public:
	tgCamera();

	// Define tgCamera
	void Set(	float posx,  float posy,  float posz,
						float viewx, float viewy, float viewz,
						float upx,   float upy,   float upz,
						float fovy=45, float width=800, float height=600,
						float zNear=0.1, float zFar=100,
						unsigned short projection=GL_PERSPECTIVE );
	void SetExtrinsic(float* M);
	void SetIntrinsic(float* M);
	void SetIntrinsic(float fovy, float width, float height, float zNear, float zFar, unsigned short projection);
	void SetViewport(float w, float h);
	void SetZRange(float near, float far);
	void SetPerspective(){m_projection=GL_PERSPECTIVE;}
	void SetOrtho(){m_projection=GL_ORTHO;}
	void SetPos(float x, float y, float z){ m_vPos.x=x; m_vPos.y=y; m_vPos.z=z; }
	
	void Activate();
	void Print();
	
	void pvu2fsu();
	void fsu2pvu();
	void fsu2extrinsic();
	void extrinsic2fsu();
	void fwh2intrinsic();
	
	// Gets
	tgVector3 GetF(){return f;}
	tgVector3 GetS(){return s;}
	tgVector3 GetU(){return u;}
	
	tgVector3 GetPos(){return m_vPos;}
	tgVector3 GetView(){return m_vView;}
	tgVector3 GetUp(){return m_vUp;}
	
	float GetZNear(){ return m_zNear; }
	float GetZFar(){ return m_zFar; }
	
	float GetFOVY(){ return m_fovy; }
	unsigned short GetProjection(){ return m_projection; }

	// Translations
	void Translate(tgVector3 v);
	void Translate(float x, float y, float z, float fWay);
	void TranslateF(float fWay);
	void TranslateS(float fWay);
	void TranslateU(float fWay);
	
	// Rotations
	void Rotate(float x, float y, float z, float fAngle);
	void RotateF(float fAngle);
	void RotateS(float fAngle);
	void RotateU(float fAngle);
	
	void RotateX(float fAngle);
	void RotateY(float fAngle);
	void RotateZ(float fAngle);
	
	void Orbit(tgVector3 vPoint, tgVector3 vAxis, float fAngle);
	
	// Movement
	void ApplyTransform();

};

#endif
