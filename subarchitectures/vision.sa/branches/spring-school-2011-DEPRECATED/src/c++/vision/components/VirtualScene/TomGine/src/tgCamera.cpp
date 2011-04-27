//CPP:
//			Title:			class tgCamera
//			File:				tgCamera.cpp
//
//			Function:		Viewing tgCamera for free movement in space
//									Forward, Backward, Strafe, Rotate
//
//			Author:			Thomas Mörwald
//			Date:				09.01.2007
// ----------------------------------------------------------------------------
#include "tgCamera.h"

using namespace TomGine;

tgCamera::tgCamera(){
	f = tgVector3(0.0f,0.0f,-1.0f);
	s = tgVector3(1.0f,0.0f,0.0f);
	u = tgVector3(0.0f,1.0f,0.0f);
}

void tgCamera::Set(	float posx,  float posy,  float posz,
										float viewx, float viewy, float viewz,
										float upx,   float upy,   float upz,
										float fovy, float width, float height,
										float zNear, float zFar,
										unsigned short projection){
	m_vPos	= tgVector3(posx,  posy,  posz ); // set position
	m_vView	= tgVector3(viewx, viewy, viewz); // set view point
	m_vUp	= tgVector3(upx,   upy,   upz  ); // set the up vector
	pvu2fsu();
	
	m_fovy = fovy;
	m_width = width;
	m_height = height;
	m_zNear = zNear;
	m_zFar = zFar;
	m_projection = projection;
	
	fwh2intrinsic();
	fsu2extrinsic();
}

void tgCamera::SetExtrinsic(float* M){	
	m_extrinsic = mat4(M);
	
	extrinsic2fsu();
	fsu2pvu();
	
	Activate();
}

void tgCamera::SetIntrinsic(float* M){
	m_intrinsic = mat4(M);
	
	Activate();
}

void tgCamera::SetIntrinsic(	float fovy, float width, float height,
															float zNear, float zFar,
															unsigned short projection)
{
	m_fovy = fovy;
	m_width = width;
	m_height = height;
	m_zNear = zNear;
	m_zFar = zFar;
	m_projection = projection;
	
	float m[16];
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	if(m_projection == GL_ORTHO){
		glOrtho(-m_width/2, m_width/2, -m_height/2, m_height/2, m_zNear, m_zFar);
	}
	else if(m_projection == GL_PERSPECTIVE){
		gluPerspective( m_fovy, m_width/m_height, m_zNear, m_zFar);
	}
	glGetFloatv(GL_PROJECTION_MATRIX, m);
	m_intrinsic = mat4(m);
	
	Activate();
}

void tgCamera::SetViewport(float w, float h){
	m_width = w;
	m_height = h;
}

void tgCamera::SetZRange(float _near, float _far){
	m_zNear = _near;
	m_zFar = _far;
}

void tgCamera::Activate(){
	// Apply intrinsic parameters
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(m_intrinsic);
	
	// Apply extrinsic parameters
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(m_extrinsic);
	
	glViewport(0,0,m_width,m_height);
	glDepthRange(m_zNear,m_zFar);
	
	// Extract frustum planes
	m_frustum.ExtractFrustum();
}

void tgCamera::Print(){
	float m[16];
	float p[16];
	
	glGetFloatv(GL_MODELVIEW_MATRIX, m);
	glGetFloatv(GL_PROJECTION_MATRIX, p);

	printf("Modelview matrix:\n");
	printf("%f %f %f %f\n", m[0], m[4], m[8],  m[12]);
	printf("%f %f %f %f\n", m[1], m[5], m[9],  m[13]);
	printf("%f %f %f %f\n", m[2], m[6], m[10], m[14]);
	printf("%f %f %f %f\n", m[3], m[7], m[11], m[15]);
	printf("Projection matrix:\n");
	printf("%f %f %f %f\n", p[0], p[4], p[8],  p[12]);
	printf("%f %f %f %f\n", p[1], p[5], p[9],  p[13]);
	printf("%f %f %f %f\n", p[2], p[6], p[10], p[14]);
	printf("%f %f %f %f\n", p[3], p[7], p[11], p[15]);
}

void tgCamera::pvu2fsu(){
	f = m_vView - m_vPos; f.normalize();
	s = f.cross(m_vUp); s.normalize();
	u = s.cross(f); u.normalize();
}

void tgCamera::fsu2pvu(){
	m_vView = m_vPos + f;
	m_vUp = u;
}

void tgCamera::fsu2extrinsic(){
	float fR[16] = {	s.x,  u.x, -f.x,  0.0,
										s.y,  u.y, -f.y,  0.0,
										s.z,  u.z, -f.z,  0.0,
										0.0,	0.0,	0.0,  1.0};
									
	float ft[16] = {	1.0,  0.0, 0.0, 0.0,
										0.0,  1.0, 0.0, 0.0,
										0.0,  0.0, 1.0, 0.0,
										-m_vPos.x,	-m_vPos.y, -m_vPos.z, 1.0};
	mat4 R(fR);
	mat4 t(ft);

	m_extrinsic = R * t;
}


void tgCamera::extrinsic2fsu(){
	s.x=m_extrinsic[0]; u.x=m_extrinsic[1]; f.x=-m_extrinsic[2];
	s.y=m_extrinsic[4]; u.y=m_extrinsic[5]; f.y=-m_extrinsic[6];
	s.z=m_extrinsic[8]; u.z=m_extrinsic[9]; f.z=-m_extrinsic[10];
	
	float fR[16] = {	s.x,  u.x, -f.x,  0.0,
											s.y,  u.y, -f.y,  0.0,
											s.z,  u.z, -f.z,  0.0,
											0.0,	0.0,	0.0,  1.0};
	mat4 R(fR);
	mat4 t;
	
	t = R.inverse() * m_extrinsic;
	
	m_vPos.x=-t[12]; m_vPos.x=-t[13]; m_vPos.x=-t[14];
}

void tgCamera::fwh2intrinsic(){
	float m[16];
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	if(m_projection == GL_ORTHO){
		glOrtho(-m_width/2, m_width/2, -m_height/2, m_height/2, m_zNear, m_zFar);
	}
	else if(m_projection == GL_PERSPECTIVE){
		gluPerspective( m_fovy, m_width/m_height, m_zNear, m_zFar);
	}
	glGetFloatv(GL_PROJECTION_MATRIX, m);
	m_intrinsic = mat4(m);		
}

tgPose tgCamera::GetPose(){
	printf("[tgCamera::GetPose()] Not implemented yet\n");
	return tgPose();
}
//****************************************************************************
// Translations
void tgCamera::Translate(tgVector3 v){
	m_vPos = m_vPos + v;
	m_vView = m_vPos + f;
}

void tgCamera::Translate(float x, float y, float z, float fWay){
	tgVector3 v = tgVector3(x,y,z);
	v.normalize();
	m_vPos = m_vPos + v * fWay;
	m_vView = m_vPos + f;
}

void tgCamera::TranslateF(float fWay){
	m_vPos = m_vPos + f * fWay;
	m_vView = m_vPos + f;
}

void tgCamera::TranslateS(float fWay){
	m_vPos = m_vPos + s * fWay;
	m_vView = m_vPos + f;
}

void tgCamera::TranslateU(float fWay){
	m_vPos = m_vPos + u * fWay;
	m_vView = m_vPos + f;
}

//****************************************************************************
// Rotations
void tgCamera::Rotate(float x, float y, float z, float fAngle){
	tgVector3 v = tgVector3(x,y,z);
	f.rotate(fAngle, v);
	s.rotate(fAngle, v);
	u.rotate(fAngle, v);
	fsu2pvu();
}

void tgCamera::RotateF(float fAngle){
	s.rotate(fAngle, f);
	u = s.cross(f); u.normalize();
	fsu2pvu();
}

void tgCamera::RotateS(float fAngle){
	f.rotate(fAngle, s);
	u = s.cross(f); u.normalize();
	fsu2pvu();
}

void tgCamera::RotateU(float fAngle){
	f.rotate(fAngle, u);
	s = f.cross(u); s.normalize();
	fsu2pvu();
}

void tgCamera::RotateX(float fAngle){
	printf("tgCamera.RotateX not implemented! \n");
}

void tgCamera::RotateY(float fAngle){
	tgVector3 y = tgVector3(0.0f,1.0f,0.0f);
	f.rotate(fAngle, y);
	s = f.cross(y); s.normalize();
	u = s.cross(f); u.normalize();
	fsu2pvu();
}

void tgCamera::RotateZ(float fAngle){
	printf("tgCamera.RotateZ not implemented! \n");
}

void tgCamera::Orbit(tgVector3 vPoint, tgVector3 vAxis, float fAngle){
	tgVector3 d = m_vPos - vPoint;
	
	d.rotate(fAngle, vAxis);
	m_vPos = vPoint + d;
	
	f.rotate(fAngle, vAxis);
	s.rotate(fAngle, vAxis);
	u.rotate(fAngle, vAxis);	
}

//****************************************************************************
// Movement
void tgCamera::ApplyTransform(){
	fsu2extrinsic();
}



