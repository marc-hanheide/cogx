//HEADER:
//			Title:			class Camera
//			File:				Camera.h
//
//			Function:		Header file for Viewing Camera
//
//			Author:			Thomas Mörwald
//			Date:				17.06.2009
// ----------------------------------------------------------------------------
#ifndef CAMERA_H
#define CAMERA_H

#include "headers.h"
#include "TM_Vector3.h"
#include "Frustum.h"
#include "mathlib.h"

#define GL_ORTHO 0
#define GL_PERSPECTIVE 1

namespace Tracking{

struct CameraParameter{
  // image dimension
  int width;
  int height;
  // Instrinsic parameters:
  // entries of the camera matrix
  double fx;
  double fy;
  double cx;
  double cy;
  // radial distortion parameters
  double k1;
  double k2;
  double k3;
  // tangential distortion parameters
  double p1;
  double p2;
  // extrinsic parameters: 3D pose of camera w.r.t. world
  mat3 rot;
  vec3 pos;
	
	float zNear;
	float zFar;
};

class Camera
{
private:
	// Camera definition
	TM_Vector3 m_vPos;			// Position of Camera (absolute)
	TM_Vector3 m_vView;		// Viewpoint of Camera (absolute)
	TM_Vector3 m_vUp;			// The Cameras upside (relative)
	
	TM_Vector3 f;		// Vector of camera pointing forward 
	TM_Vector3 s;		// Vector of camera pointing sidewards (right)
	TM_Vector3 u;		// Vector of camera pointing up
	
	float m_fovy, m_width, m_height;
	float m_zNear, m_zFar;
	unsigned short m_projection;	
	mat4 m_extrinsic;
	mat4 m_intrinsic;
	
	Frustum m_frustum;
	
public:
	Camera();
	
	void Load(CameraParameter camPar);

	// Define Camera
	void Set(	float posx,  float posy,  float posz,
						float viewx, float viewy, float viewz,
						float upx,   float upy,   float upz,
						float fovy=45, float width=800, float height=600,
						float zNear=0.1, float zFar=100,
						unsigned short projection=GL_PERSPECTIVE );
	void SetExtrinsic(float* M);
	void SetIntrinsic(float* M);
	void SetIntrinsic(float fovy, float width, float height);
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
	TM_Vector3 GetF(){return f;}
	TM_Vector3 GetS(){return s;}
	TM_Vector3 GetU(){return u;}
	
	TM_Vector3 GetPos(){return m_vPos;}
	TM_Vector3 GetView(){return m_vView;}
	TM_Vector3 GetUp(){return m_vUp;}
	
	float GetZNear(){ return m_zNear; }
	float GetZFar(){ return m_zFar; }
	
	mat4 GetIntrinsic(){ return m_intrinsic; }
	mat4 GetExtrinsic(){ return m_extrinsic; }
	
	Frustum* GetFrustum(){ return &m_frustum; }

	// Translations
	void Translate(TM_Vector3 v);
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
	
	void Orbit(TM_Vector3 vPoint, TM_Vector3 vAxis, float fAngle);
	
	// Movement
	void Transform();

};

} // namespace Tracking

#endif
