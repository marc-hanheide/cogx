//CPP:
//			Title:			class Camera
//			File:				Camera.cpp
//
//			Function:		Viewing Camera for free movement in space
//									Forward, Backward, Strafe, Rotate
//
//			Author:			Thomas M�rwald
//			Date:				09.01.2007
// ----------------------------------------------------------------------------
#include "Camera.h"

using namespace Tracking;

Camera::Camera(){
	f = TM_Vector3(0.0f,0.0f,-1.0f);
	s = TM_Vector3(1.0f,0.0f,0.0f);
	u = TM_Vector3(0.0f,1.0f,0.0f);
}

void Camera::Load(CameraParameter camPars){

	// intrinsic parameters
	// transform the coordinate system of computer vision to OpenGL 
	//   Vision: origin is in the up left corner, x-axis pointing right, y-axis pointing down
	//   OpenGL: origin is in the middle, x-axis pointing right, y-axis pointing up
	float fx = 2.0*camPars.fx / camPars.width;					// scale range from [0 ... 640] to [0 ... 2]
  float fy = 2.0*camPars.fy / camPars.height;					// scale range from [0 ... 480] to [0 ... 2]
  float cx = 1.0-(2.0*camPars.cx / camPars.width);		// move coordinates from left to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float cy = (2.0*camPars.cy / camPars.height)-1.0;		// flip and move coordinates from top to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
  float z1 = (camPars.zFar+camPars.zNear)/(camPars.zNear-camPars.zFar);								// entries for clipping planes
  float z2 = 2*camPars.zFar*camPars.zNear/(camPars.zNear-camPars.zFar);								// look up for gluPerspective
  
  // intrinsic matrix
  mat4 intrinsic;
  intrinsic[0]=fx;	intrinsic[4]=0;		intrinsic[8]=cx;	intrinsic[12]=0;
  intrinsic[1]=0;		intrinsic[5]=fy;	intrinsic[9]=cy;	intrinsic[13]=0;
  intrinsic[2]=0;		intrinsic[6]=0;		intrinsic[10]=z1;	intrinsic[14]=z2;  
  intrinsic[3]=0;		intrinsic[7]=0;		intrinsic[11]=-1;	intrinsic[15]=0;	// last row assigns w=-z which inverts cx and cy at w-division
  
  // computer vision camera coordinates to OpenGL camera coordinates transform 
  // rotate 180� about x-axis
  mat4 cv2gl;
  cv2gl[0]=1.0; cv2gl[4]=0.0; 	cv2gl[8]=0.0;   cv2gl[12]=0.0;  
	cv2gl[1]=0.0; cv2gl[5]=-1.0;	cv2gl[9]=0.0;   cv2gl[13]=0.0;  
	cv2gl[2]=0.0; cv2gl[6]=0.0; 	cv2gl[10]=-1.0; cv2gl[14]=0.0;  
	cv2gl[3]=0.0; cv2gl[7]=0.0; 	cv2gl[11]=0.0;  cv2gl[15]=1.0;  
	
	// extrinsic parameters
	// look up comments in tools/hardware/video/src/slice/Video.ice
	// p = R^T*(w - t) = (R^T, -R^T*t) * (w,1)
	mat3 R = camPars.rot;
	vec3 t = camPars.pos;
	mat4 extrinsic;
	extrinsic[0]=R[0];	extrinsic[4]=R[1];	extrinsic[8]=R[2];		extrinsic[12]=0.0;
	extrinsic[1]=R[3];	extrinsic[5]=R[4];	extrinsic[9]=R[5];		extrinsic[13]=0.0;	
	extrinsic[2]=R[6];	extrinsic[6]=R[7];	extrinsic[10]=R[8];		extrinsic[14]=0.0;	
	extrinsic[3]=0.0;		extrinsic[7]=0.0;		extrinsic[11]=0.0;		extrinsic[15]=1.0;
	extrinsic = extrinsic.transpose();											// R^T
	vec4 tp = -(extrinsic * vec4(t.x, t.y, t.z, 1.0));			// -R^T*t
	extrinsic[12]=tp.x; extrinsic[13]=tp.y; extrinsic[14]=tp.z;
	extrinsic = cv2gl * extrinsic;
	
	// set camera parameters
	SetViewport(camPars.width,camPars.height);
	SetZRange(camPars.zNear, camPars.zFar);
	SetIntrinsic(intrinsic);
	SetExtrinsic(extrinsic);
	SetPos(camPars.pos.x, camPars.pos.y, camPars.pos.z);
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
	
	fwh2intrinsic();
	fsu2extrinsic();
}

void Camera::SetExtrinsic(float* M)
{	
	m_extrinsic = mat4(M);
	
	extrinsic2fsu();
	fsu2pvu();
	
	Activate();
}

void Camera::SetIntrinsic(float* M)
{
	m_intrinsic = mat4(M);
	
	Activate();
}

void Camera::SetViewport(float w, float h)
{
	m_width = w;
	m_height = h;
}

void Camera::SetZRange(float _near, float _far)
{
	m_zNear = _near;
	m_zFar = _far;
}

void Camera::Activate()
{
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

void Camera::Print()
{
	printf("Modelview matrix:\n");
	printf("%f %f %f %f\n", m_extrinsic[0], m_extrinsic[4], m_extrinsic[8],  m_extrinsic[12]);
	printf("%f %f %f %f\n", m_extrinsic[1], m_extrinsic[5], m_extrinsic[9],  m_extrinsic[13]);
	printf("%f %f %f %f\n", m_extrinsic[2], m_extrinsic[6], m_extrinsic[10], m_extrinsic[14]);
	printf("%f %f %f %f\n", m_extrinsic[3], m_extrinsic[7], m_extrinsic[11], m_extrinsic[15]);
	printf("Projection matrix:\n");
	printf("%f %f %f %f\n", m_intrinsic[0], m_intrinsic[4], m_intrinsic[8],  m_intrinsic[12]);
	printf("%f %f %f %f\n", m_intrinsic[1], m_intrinsic[5], m_intrinsic[9],  m_intrinsic[13]);
	printf("%f %f %f %f\n", m_intrinsic[2], m_intrinsic[6], m_intrinsic[10], m_intrinsic[14]);
	printf("%f %f %f %f\n", m_intrinsic[3], m_intrinsic[7], m_intrinsic[11], m_intrinsic[15]);
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

void Camera::fsu2extrinsic()
{
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


void Camera::extrinsic2fsu()
{
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

void Camera::fwh2intrinsic()
{
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
	fsu2extrinsic();
}
