//CPP:
//			Title:			class Camera
//			File:				Camera.cpp
//
//			Function:		Viewing Camera for free movement in space
//									Forward, Backward, Strafe, Rotate
//
//			Author:			Thomas Mörwald
//			Date:				09.01.2007
// ----------------------------------------------------------------------------
#include "Camera.h"

Camera::Camera()
{
	f = TM_Vector3(0.0f,0.0f,-1.0f);
	s = TM_Vector3(1.0f,0.0f,0.0f);
	u = TM_Vector3(0.0f,1.0f,0.0f);
}

void Camera::Set(	float posx,  float posy,  float posz,
					float viewx, float viewy, float viewz,
					float upx,   float upy,   float upz,
					float fovy, float width, float height,
					float zNear, float zFar,
					unsigned short projection)
{
	m_vPos	= TM_Vector3(posx,  posy,  posz ); // set position
	m_vView	= TM_Vector3(viewx, viewy, viewz); // set view point
	m_vUp	= TM_Vector3(upx,   upy,   upz  ); // set the up vector
	pvu2fsu();
	
	m_fovy = fovy;
	m_width = width;
	m_height = height;
	m_zNear = zNear;
	m_zFar = zFar;
	m_projection = projection;
}

void Camera::SetExtrinsic(float* R, float* t){
	
	s.x=R[0]; u.x=R[1]; f.x=R[2];
	s.y=R[3]; u.y=R[4]; f.y=R[5];
	s.z=R[6]; u.z=R[7]; f.z=R[8];
	
	m_vPos.x=t[0]; m_vPos.y=t[1]; m_vPos.z=t[2];
	fsu2pvu();
	
	//Transform();	
}

void Camera::SetIntrinsic(float fovy, float width, float height){
	// TODO implement
	m_fovy = fovy;
	m_width = width;
	m_height = height;
}

void Camera::Activate()
{
	if(m_projection == GL_ORTHO){
		// Apply intrinsic parameters
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-m_width/2, m_width/2, -m_height/2, m_height/2, m_zNear, m_zFar);
		
		// Apply intrinsic parameters
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		Transform();
		
		// Extract frustum planes
		g_Resources->GetFrustum()->ExtractFrustum();
	}
	else if(m_projection == GL_PERSPECTIVE){
		// Apply intrinsic parameters
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective( m_fovy, m_width/m_height, m_zNear, m_zFar);
		
		// Apply extrinsic parameters
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		Transform();
		
		// Extract frustum planes
		g_Resources->GetFrustum()->ExtractFrustum();
	}
}

void Camera::Print(){
	printf("%f %f %f, %f\n", s.x, u.x, f.x, m_vPos.x);
	printf("%f %f %f, %f\n", s.y, u.y, f.y, m_vPos.y);
	printf("%f %f %f, %f\n", s.z, u.z, f.z, m_vPos.z);
}

void Camera::pvu2fsu()
{
	f = m_vView - m_vPos; f.normalize();
	s = f.cross(m_vUp); s.normalize();
	u = s.cross(f); u.normalize();
}

void Camera::fsu2pvu()
{
	m_vView = m_vPos + f;
	m_vUp = u;
}

//****************************************************************************
// Translations
void Camera::Translate(TM_Vector3 v)
{
    m_vPos = m_vPos + v;
	m_vView = m_vPos + f;
}

void Camera::Translate(float x, float y, float z, float fWay)
{
	TM_Vector3 v = TM_Vector3(x,y,z);
	v.normalize();
	m_vPos = m_vPos + v * fWay;
	m_vView = m_vPos + f;
}

void Camera::TranslateF(float fWay)
{
	m_vPos = m_vPos + f * fWay;
	m_vView = m_vPos + f;
}
void Camera::TranslateS(float fWay)
{
	m_vPos = m_vPos + s * fWay;
	m_vView = m_vPos + f;
}
void Camera::TranslateU(float fWay)
{
	m_vPos = m_vPos + s * fWay;
	m_vView = m_vPos + f;
}

//****************************************************************************
// Rotations
void Camera::Rotate(float x, float y, float z, float fAngle)
{
	TM_Vector3 v = TM_Vector3(x,y,z);
	f.rotate(fAngle, v);
	s.rotate(fAngle, v);
	u.rotate(fAngle, v);
	fsu2pvu();
}

void Camera::RotateF(float fAngle)
{
	s.rotate(fAngle, f);
	u = s.cross(f); u.normalize();
	fsu2pvu();
}

void Camera::RotateS(float fAngle)
{
	f.rotate(fAngle, s);
	u = s.cross(f); u.normalize();
	fsu2pvu();
}

void Camera::RotateU(float fAngle)
{
	f.rotate(fAngle, u);
	s = f.cross(u); s.normalize();
	fsu2pvu();
}

void Camera::RotateX(float fAngle)
{
	printf("Camera.RotateX not implemented! \n");
}

void Camera::RotateY(float fAngle)
{
	TM_Vector3 y = TM_Vector3(0.0f,1.0f,0.0f);
	f.rotate(fAngle, y);
	s = f.cross(y); s.normalize();
	u = s.cross(f); u.normalize();
	fsu2pvu();
}

void Camera::RotateZ(float fAngle)
{
	printf("Camera.RotateZ not implemented! \n");
}

void Camera::Orbit(TM_Vector3 vPoint, TM_Vector3 vAxis, float fAngle)
{
	TM_Vector3 d = m_vPos - vPoint;
	
	d.rotate(fAngle, vAxis);
	m_vPos = vPoint + d;
	
	f.rotate(fAngle, vAxis);
	s.rotate(fAngle, vAxis);
	u.rotate(fAngle, vAxis);	
}

//****************************************************************************
// Movement
void Camera::Transform()
{
	float m[16] = {	 s.x,  u.x, -f.x,  0,
				 	 s.y,  u.y, -f.y,  0,
					 s.z,  u.z, -f.z,  0,
				   	   0,	 0,	   0,  1};
	glMultMatrixf(m);
	glTranslatef(-m_vPos.x,-m_vPos.y,-m_vPos.z);
	
	/*
	float params[16];
	glGetFloatv(GL_PROJECTION_MATRIX, params);
	
	if(m_projection == GL_PERSPECTIVE){
		printf("\nModelview\n");
		printf("%f %f %f %f\n", params[0], params[4], params[8], params[12]);
		printf("%f %f %f %f\n", params[1], params[5], params[9], params[13]);
		printf("%f %f %f %f\n", params[2], params[6], params[10], params[14]);
		printf("%f %f %f %f\n\n", params[3], params[7], params[11], params[15]);
	}
	*/

}



