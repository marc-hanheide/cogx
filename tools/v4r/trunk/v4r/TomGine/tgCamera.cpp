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

tgCamera::Parameter::Parameter(){
	width = 640;
	height = 480;
	fx = 520.0f;
	fy = 520.0f;
	cx = 320.0f;
	cy = 240.0f;
	k1 = 0.0f;
	k2 = 0.0f;
	k3 = 0.0f;
	p1 = 0.0f;
	p2 = 0.0f;
	rot.fromRotVector(vec3(-2.0f,-1.0f,0.5f));
	pos = vec3(0.6f, -0.2f, 0.5f);
	zNear = 0.01f;
	zFar = 4.0f;
}

tgCamera::tgCamera(){
	f = vec3(0.0f,0.0f,-1.0f);
	s = vec3(1.0f,0.0f,0.0f);
	u = vec3(0.0f,1.0f,0.0f);
	fsu2pvu();
	ApplyTransform();
}

void tgCamera::Set(tgCamera::Parameter camPars){

	// intrinsic parameters
	// transform the coordinate system of computer vision to OpenGL 
	//   Vision: origin is in the up left corner, x-axis pointing right, y-axis pointing down
	//   OpenGL: origin is in the middle, x-axis pointing right, y-axis pointing up
	float fx = 2.0f*camPars.fx / camPars.width;					// scale range from [0 ... 640] to [0 ... 2]
	float fy = 2.0f*camPars.fy / camPars.height;					// scale range from [0 ... 480] to [0 ... 2]
	float cx = 1.0f-(2.0f*camPars.cx / camPars.width);		// move coordinates from left to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
	float cy = (2.0f*camPars.cy / camPars.height)-1.0f;		// flip and move coordinates from top to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
	float z1 = (camPars.zFar+camPars.zNear)/(camPars.zNear-camPars.zFar);								// entries for clipping planes
	float z2 = 2.0f*camPars.zFar*camPars.zNear/(camPars.zNear-camPars.zFar);								// look up for gluPerspective

	// intrinsic matrix
	mat4 intrinsic;
	intrinsic[0]=fx;	intrinsic[4]=0;	intrinsic[8]=cx;	intrinsic[12]=0;
	intrinsic[1]=0;	intrinsic[5]=fy;	intrinsic[9]=cy;	intrinsic[13]=0;
	intrinsic[2]=0;	intrinsic[6]=0;		intrinsic[10]=z1;	intrinsic[14]=z2;  
	intrinsic[3]=0;	intrinsic[7]=0;		intrinsic[11]=-1;	intrinsic[15]=0;	// last row assigns w=-z which inverts cx and cy at w-division

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
	extrinsic[0]=R[0];	extrinsic[4]=R[3];	extrinsic[8]=R[6];		extrinsic[12]=0.0;
	extrinsic[1]=R[1];	extrinsic[5]=R[4];	extrinsic[9]=R[7];		extrinsic[13]=0.0;	
	extrinsic[2]=R[2];	extrinsic[6]=R[5];	extrinsic[10]=R[8];		extrinsic[14]=0.0;	
	extrinsic[3]=0.0;	extrinsic[7]=0.0;	extrinsic[11]=0.0;		extrinsic[15]=1.0;
// 	extrinsic = extrinsic.transpose();							// R^T
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

void tgCamera::Set(	vec3 pos, vec3 view, vec3 up,
				float fovy, unsigned width, unsigned height,
				float zNear, float zFar, tgCamera::Projection proj )
{
	this->pos	= vec3(pos.x,  pos.y,  pos.z ); // set position
	this->view	= vec3(view.x, view.y, view.z); // set view point
	this->up	= vec3(up.x,   up.y,   up.z  ); // set the up vector
	pvu2fsu();
	
	m_fovy = fovy;
	m_width = width;
	m_height = height;
	m_zNear = zNear;
	m_zFar = zFar;
	m_projection = proj;
	
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

void tgCamera::SetIntrinsic(	float fovy, unsigned width, unsigned height,
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
		glOrtho(0, m_width, 0, m_height, m_zNear, m_zFar);
	}
	else if(m_projection == GL_PERSPECTIVE){
		gluPerspective( m_fovy, float(m_width)/float(m_height), m_zNear, m_zFar);
	}
	glGetFloatv(GL_PROJECTION_MATRIX, m);
	m_intrinsic = mat4(m);
	
	Activate();
}

void tgCamera::SetViewport(unsigned w, unsigned h){
	m_width = w;
	m_height = h;
}

void tgCamera::SetZRange(float _near, float _far){
	m_zNear = _near;
	m_zFar = _far;
}

vec2 tgCamera::ToImageSpace(const vec3 &world_space){
	vec2 ret;
	vec4 tmp = m_intrinsic * m_extrinsic * vec4(world_space.x,world_space.y,world_space.z,1.0f);
	
	tmp.x = tmp.x / tmp.w;
	tmp.y = tmp.y / tmp.w;
	
	ret.x = (tmp.x + 1.0f) * 0.5f * m_width;
	ret.y = (tmp.y + 1.0f) * 0.5f * m_height;
	
	return ret;
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
// 	float m[16];
// 	float p[16];
// 	
// 	glGetFloatv(GL_MODELVIEW_MATRIX, m);
// 	glGetFloatv(GL_PROJECTION_MATRIX, p);
	
	mat4 m = m_extrinsic;
	mat4 p = m_intrinsic;

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
	f = view - pos; f.normalize();
	s = cross(f, up); s.normalize();
	u = cross(s, f); u.normalize();
}

void tgCamera::fsu2pvu(){
	view = pos + f;
	up = u;
}

void tgCamera::fsu2extrinsic(){
	float fR[16] = {	s.x,  u.x, -f.x,  0.0,
						s.y,  u.y, -f.y,  0.0,
						s.z,  u.z, -f.z,  0.0,
						0.0,	0.0,	0.0,  1.0};
									
	float ft[16] = {	1.0,  0.0, 0.0, 0.0,
						0.0,  1.0, 0.0, 0.0,
						0.0,  0.0, 1.0, 0.0,
						-pos.x,	-pos.y, -pos.z, 1.0};
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
	
	pos.x=-t[12]; pos.x=-t[13]; pos.x=-t[14];
}

void tgCamera::fwh2intrinsic(){
	float m[16];
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	if(m_projection == GL_ORTHO){
		glOrtho(0, m_width, 0, m_height, m_zNear, m_zFar);
	}
	else if(m_projection == GL_PERSPECTIVE){
		gluPerspective( m_fovy, float(m_width)/float(m_height), m_zNear, m_zFar);
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
void tgCamera::Translate(vec3 v){
	pos = pos + v;
	view = pos + f;
}

void tgCamera::Translate(float x, float y, float z, float fWay){
	vec3 v = vec3(x,y,z);
	v.normalize();
	pos = pos + v * fWay;
	view = pos + f;
}

void tgCamera::TranslateF(float fWay){
	pos = pos + f * fWay;
	view = pos + f;
}

void tgCamera::TranslateS(float fWay){
	pos = pos + s * fWay;
	view = pos + f;
}

void tgCamera::TranslateU(float fWay){
	pos = pos + u * fWay;
	view = pos + f;
}

//****************************************************************************
// Rotations
void tgCamera::Rotate(float x, float y, float z, float fAngle){
	vec3 v = vec3(x,y,z);
	f.rotate(fAngle, v);
	s.rotate(fAngle, v);
	u.rotate(fAngle, v);
	fsu2pvu();
}

void tgCamera::RotateF(float fAngle){
	s.rotate(fAngle, f);
	u = cross(s,f); u.normalize();
	fsu2pvu();
}

void tgCamera::RotateS(float fAngle){
	f.rotate(fAngle, s);
	u = cross(s,f); u.normalize();
	fsu2pvu();
}

void tgCamera::RotateU(float fAngle){
	f.rotate(fAngle, u);
	s = cross(f,u); s.normalize();
	fsu2pvu();
}

//void tgCamera::RotateX(float fAngle){
//	printf("tgCamera.RotateX not implemented! \n");
//}

void tgCamera::RotateY(float fAngle){
	vec3 y = vec3(0.0f,1.0f,0.0f);
	f.rotate(fAngle, y);
	s = cross(f,y); s.normalize();
	u = cross(s,f); u.normalize();
	fsu2pvu();
}

//void tgCamera::RotateZ(float fAngle){
//	printf("tgCamera.RotateZ not implemented! \n");
//}

void tgCamera::Orbit(vec3 vPoint, vec3 vAxis, float fAngle)
{
	vec3 d = pos - vPoint;
	
	d.rotate(fAngle, vAxis);
	pos = vPoint + d;
	
	f.rotate(fAngle, vAxis);
	s.rotate(fAngle, vAxis);
	u.rotate(fAngle, vAxis);	
}

void tgCamera::LookAt(const vec3 &pov)
{
	this->view = pov;
	pvu2fsu();
}

//****************************************************************************
// Movement
void tgCamera::ApplyTransform(){
	fsu2extrinsic();
}



