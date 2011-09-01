//SOURCE:
//			Title:			class tgEngine
//			File:				tgEngine.cpp
//
//			Function:		Main file of Engine providing interfaces
//
//			Author:			Thomas Mörwald
//			Date:				20.11.2009
// ----------------------------------------------------------------------------

#include "tgEngine.h"

using namespace TomGine;
using namespace V4R;

tgEngine::tgEngine(	unsigned width,
					unsigned height,
					float far,
					float near,
					const char* name,
					bool bfc,
					bool threaded)
{
	m_window = new GLWindow(width, height, name, threaded);

	m_width = width;
	m_height = height;
	m_far = far;
	m_near = near;
	m_bfc = bfc;

	m_input_rotation_speed		= 1.0f;
	m_input_translation_speed	= 1.0f;
	m_input_zoom_speed			= 1.0f;

	m_cor = vec3(0.0,0.0,0.0);

	m_button_left = false;
	m_button_middle = false;
	m_button_right = false;

	m_wireframe = false;
	m_smoothshading = false;
	m_show_background_image = false;

	m_mouse_pos[0] = 0;
	m_mouse_pos[1] = 0;

	float da = 0.25f*(m_far-m_near);
	// Setup 3D camera
	m_camera.Set(	vec3(da, da, da),							// Position of camera
					vec3(0,0,0),							// Point where camera looks at
					vec3(0,1,0),							// UP-Vector of Camera
					45, width, height,					// field of view in degree in y, image width, image height
					near, far,							// near clipping plane, far clipping plane
					tgCamera::GL_PERSPECTIVE);					// Perspective camera
	UpdateCameraViews(m_camera);

	// Setup 2D camera
	m_cam_ortho.Set(	vec3(0,0,1),
						vec3(0,0,0),
						vec3(0,1,0),
						45, width, height,
						0.1f, 2.0f,
						tgCamera::GL_ORTHO);

	vec3 cam_f = m_camera.GetF();
	m_light0.ambient = vec4(0.4f,0.4f,0.4f,1.0f);
	m_light0.diffuse = vec4(1.0f,1.0f,1.0f,1.0f);
	m_light0.specular = vec4(1.0f,1.0f,1.0f,1.0f);
	m_light0.position = vec4(-cam_f.x, -cam_f.y, -cam_f.z, 1.0f);
	m_light0.Activate();

	m_material.Random();

	m_background_image = 0;
// 	Welcome();

	// OpenGL settings
	glEnable(GL_DEPTH_TEST);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glShadeModel(GL_SMOOTH);
	glDisable(GL_TEXTURE_2D);

	m_frametime = 0.0;
}

tgEngine::~tgEngine(){
	if(m_window) delete(m_window);
	if(m_background_image) delete(m_background_image);
//	pthread_join(thread,NULL);
//	pthread_mutex_destroy(&eventMutex);
}

void tgEngine::Welcome(){
	printf("\n   ***   TomGine Render Engine   ***\n\n");
	printf("rotate: left mouse button\n");
	printf("slide:  right mouse button\n");
	printf("zoom:   mouse wheel\n");
	printf("[f]:    toggle shading mode\n");
	printf("[w]:    draw wireframed\n");
	printf("[esc]:	quit\n");
	printf("\n");
}

void tgEngine::Update(){
	float fTime;
	Update(fTime);
}

void tgEngine::Update(float &fTime){

//	Activate3D();
// 	DrawCoordinates();

	// swap to screen and clear render buffer
	Swap();

	// update frametime
	fTime = m_frametime = (float)m_timer.Update();

	DrawBackgroundImage();

	// Light pointing along camera viewing axis
	tgLight light;
	vec3 cam_f = m_camera.GetF();
	light.ambient = vec4(0.4f,0.4f,0.4f,1.0f);
	light.diffuse = vec4(1.0f,1.0f,1.0f,1.0f);
	light.specular = vec4(1.0f,1.0f,1.0f,1.0f);
	light.position = vec4(-cam_f.x, -cam_f.y, -cam_f.z, 1.0f);
	light.Activate();
	m_material.Activate();

	Activate3D();
}

bool tgEngine::ProcessEvents(){
	V4R::Event event;
	bool run = true;
	std::vector<V4R::Event> eventlist;
	while(m_window->GetEvent(event)){
		eventlist.push_back(event);
		run = run && InputControl(event);
	}
	return run;
}

bool tgEngine::GetEventList(std::vector<V4R::Event> &eventlist){
	V4R::Event event;
	bool run = true;
	eventlist.clear();
	while(m_window->GetEvent(event)){
		eventlist.push_back(event);
		run = run && InputControl(event);
	}
	return run;
}

void tgEngine::WaitForEvent(Event &event){
	m_window->GetEventBlocking(event);
}



bool tgEngine::InputControl(Event &event){
	int x_rel = 0;
	int y_rel = 0;

	switch(event.type){

		// *********************************************************
		//			KeyCode:		/usr/include/X11/keysymdef.h
		case TMGL_Quit:
			return false;
			break;
		case TMGL_Press:
// 			printf("event.key.keysym: %x\n", event.key.keysym);
			switch(event.input){
				case TMGL_Escape:
					return false;
					break;
				case TMGL_f:
					if(m_smoothshading)
						glShadeModel(GL_SMOOTH);
					else
						glShadeModel(GL_FLAT);
					m_smoothshading = !m_smoothshading;
					break;
				case TMGL_p:
					p_pressed = true;
					break;
				case TMGL_w:
					if(m_wireframe)
						glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
					else
						glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
					m_wireframe = !m_wireframe;
					break;
				case TMGL_KP_0:
				case TMGL_z:
					m_camera = m_cam[0];
					break;
				case TMGL_KP_7:
					m_camera = m_cam[1];
					break;
				case TMGL_KP_6:
					m_camera = m_cam[2];
					break;
				case TMGL_KP_4:
					m_camera = m_cam[3];
					break;
				case TMGL_KP_2:
					m_camera = m_cam[4];
					break;
				case TMGL_KP_8:
					m_camera = m_cam[5];
					break;
				default:
					break;

				case TMGL_Button1:
					m_button_left = true;
					break;
				case TMGL_Button2:
					m_button_middle = true;
					break;
				case TMGL_Button3:
					m_button_right = true;
					break;
				case TMGL_Button4:
					m_camera.TranslateF(0.01f*(m_far-m_near)*m_input_translation_speed);
					break;
				case TMGL_Button5:
					m_camera.TranslateF(-0.01f*(m_far-m_near)*m_input_translation_speed);
					break;
			}
			break;

		// *********************************************************
		case TMGL_Release:
			switch(event.input){
				case TMGL_Button1:
					m_button_left = false;
					break;
				case TMGL_Button2:
					m_button_middle = false;
					break;
				case TMGL_Button3:
					m_button_right = false;
					break;
			}
			break;

		// *********************************************************
		case TMGL_Motion:
			x_rel = event.motion.x - m_mouse_pos[0];
			y_rel = event.motion.y - m_mouse_pos[1];

			if(m_button_left){
				m_camera.Orbit(m_cor, m_camera.GetU(), -0.05f * x_rel * m_input_rotation_speed);
				m_camera.Orbit(m_cor, m_camera.GetS(), -0.05f * y_rel * m_input_rotation_speed);
			}else if(m_button_right){
				m_camera.TranslateS(-0.0005f*(m_far-m_near)*x_rel * m_input_zoom_speed);
				m_camera.TranslateU(0.0005f*(m_far-m_near)*y_rel * m_input_zoom_speed);
			}else if(m_button_middle){
				m_camera.TranslateF(0.001f*(m_far-m_near)*x_rel*m_input_translation_speed);
				m_camera.TranslateF(0.001f*(m_far-m_near)*y_rel*m_input_translation_speed);
			}

			m_mouse_pos[0] = event.motion.x;
			m_mouse_pos[1] = event.motion.y;

			break;

		// *********************************************************
		case TMGL_Expose:
			m_width = event.expose.width; m_height = event.expose.height;
			m_camera.SetViewport(m_width, m_height);
			m_cam[0].SetViewport(m_width, m_height);
			m_cam[1].SetViewport(m_width, m_height);
			m_cam[2].SetViewport(m_width, m_height);
			m_cam[3].SetViewport(m_width, m_height);
			m_cam[4].SetViewport(m_width, m_height);
			m_cam[5].SetViewport(m_width, m_height);
			m_cam_ortho.Set(vec3(0,0,1),
							vec3(0,0,0),
							vec3(0,1,0),
							45, m_width, m_height,
							0.1f, 2.0f,
							tgCamera::GL_ORTHO);
			break;

		// *********************************************************
// 		case ClientMessage:
// 			if(event.clientmessage.stop)
// 				return false;
// 			break;

	} // switch(event.type)
return true;
}

void tgEngine::DrawCoordinates(float linelength, float linewidth){
	tgPose p;
	p.DrawCoordinates(linelength, linewidth);
}

void tgEngine::DrawFPS(){
	char charbuffer[16];
	sprintf(charbuffer, "%d", (int)(1.0/this->m_frametime));
	g_font->Print(charbuffer, 20, 10, 10);
}

void tgEngine::SetCamera(tgCamera cam){
	m_camera = cam;
	//m_camera.Print();
}

void tgEngine::UpdateCameraViews(tgCamera cam){
	m_cam[5] = m_cam[4] = m_cam[3] = m_cam[2] = m_cam[1] = m_cam[0] = cam;
	m_cam[1].Orbit(m_cor, m_cam[1].GetU(), M_PI);
	m_cam[2].Orbit(m_cor, m_cam[2].GetU(), M_PI*0.5);
	m_cam[3].Orbit(m_cor, m_cam[3].GetU(),-M_PI*0.5);
	m_cam[4].Orbit(m_cor, m_cam[4].GetS(), M_PI*0.5);
	m_cam[5].Orbit(m_cor, m_cam[5].GetS(),-M_PI*0.5);
}

void tgEngine::SetCamera(cv::Mat &intrinsic, unsigned &width, unsigned &height, cv::Mat &R, cv::Mat &T)
{
	TomGine::tgCamera::Parameter param;

	if(intrinsic.empty()){
		printf("[tgEngine::SetCameraIntrinsic] Warning, argument 'intrinsic' not valid (empty).\n");
		return;
	}

	if(intrinsic.rows < 3 || intrinsic.cols < 3){
		printf("[tgEngine::SetCameraIntrinsic] Warning, argument 'intrinsic' not valid (size).\n");
		return;
	}

	if(R.empty() || T.empty()){
		printf("[tgEngine::SetCameraPose] Warning, arguments 'R, T' not valid (empty).\n");
		return;
	}

	if(R.rows < 3 || R.cols < 3){
		printf("[tgEngine::SetCameraPose] Warning, argument 'R' not valid (size).\n");
		return;
	}

	if(T.rows < 3 && T.cols < 3){
		printf("[tgEngine::SetCameraPose] Warning, argument 'T' not valid (size).\n");
		return;
	}

	param.width = width;
	param.height = height;
	param.zFar = m_camera.GetZFar();
	param.zNear = m_camera.GetZNear();

	// Instrinsic parameters:
	// entries of the camera matrix
	cv::Mat intrinsic32 = intrinsic;
	if (intrinsic.type() != CV_32F) intrinsic.convertTo(intrinsic32, CV_32F);
	float *d = intrinsic32.ptr<float>(0);
	param.fx = d[0];
	param.fy = d[4];
	param.cx = d[2];
	param.cy = d[5];

	// radial distortion parameters
	param.k1 = 0.0;
	param.k2 = 0.0;
	param.k3 = 0.0;
	// tangential distortion parameters
	param.p1 = 0.0;
	param.p2 = 0.0;

	cv::Mat R32 = R;
	if (R.type() != CV_32F) R.convertTo(R32, CV_32F);
	param.rot = mat3(R32.ptr<float>(0));

	cv::Mat T32 = T;
	if(T.type() != CV_32F) T.convertTo(T32, CV_32F);
	param.pos = vec3(T32.ptr<float>(0));

	m_camera.Set(param);
	UpdateCameraViews(m_camera);
}

void tgEngine::SetCenterOfRotation(float x, float y, float z){
	m_cor = vec3(x,y,z);
}

void tgEngine::Activate3D(){
	if(m_bfc)	glEnable(GL_CULL_FACE);
	else		glDisable(GL_CULL_FACE);
	m_camera.ApplyTransform();
	m_camera.Activate();
	glEnable(GL_LIGHTING);
}

void tgEngine::Activate2D(){
	glDisable(GL_CULL_FACE);
	m_cam_ortho.Activate();
	glDisable(GL_LIGHTING);
}

void tgEngine::Swap(){
	m_window->Update();
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
}

void tgEngine::LoadBackgroundImage(unsigned char* image_data, int width, int height, GLenum format, bool flip){

	if(!m_background_image)
		m_background_image = new tgTexture2D();

	m_background_image->Load(image_data, width, height, GL_RGBA, format, GL_UNSIGNED_BYTE);
	m_show_background_image = true;
	m_flip_background_image = flip;
}

void tgEngine::DrawBackgroundImage(){
	if(m_background_image){
		float w = (float)m_width;
		float h = (float)m_height;

		Activate2D();
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		glDepthMask(0);
		glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
		m_background_image->Bind();
		if(m_flip_background_image){
			glBegin(GL_QUADS);
			  glTexCoord2f(0.0f,1.0f); glVertex3f(0.,	0.,	0.0f);
			  glTexCoord2f(1.0f,1.0f); glVertex3f(w,	0.,	0.0f);
			  glTexCoord2f(1.0f,0.0f); glVertex3f(w,	h, 0.0f);
			  glTexCoord2f(0.0f,0.0f); glVertex3f(0.,	h, 0.0f);
			glEnd();
		}else{
		  glBegin(GL_QUADS);
			  glTexCoord2f(0.0f,0.0f); glVertex3f(0.,	0.,	0.0f);
			  glTexCoord2f(1.0f,0.0f); glVertex3f(w,	0.,	0.0f);
			  glTexCoord2f(1.0f,1.0f); glVertex3f(w,	h, 0.0f);
			  glTexCoord2f(0.0f,1.0f); glVertex3f(0.,	h, 0.0f);
		  glEnd();
		}
		glDisable(GL_TEXTURE_2D);
		glDepthMask(1);

		if(m_wireframe)
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	}
}

void tgEngine::UnloadBackgroundImage(){
	m_show_background_image = false;
	if(m_background_image)
		delete(m_background_image);
	m_background_image = 0;
}

void tgEngine::PrintText3D(std::string text, vec3 pos, int size){
	vec2 vPos = m_camera.ToImageSpace(pos);
	PrintText2D(text, vPos, size);
}
void tgEngine::PrintText2D(std::string text, vec2 pos, int size){
	g_font->Print(text.c_str(), size, pos.x, pos.y);
}

vec3 tgEngine::Get3DPointFrom2D(int x, int y)
{
	vec3 vResult;
	int viewport[4];
	double modelview[16];
	double projection[16];
	float z;
	int y_new;
	double result[3];

	m_camera.SetZRange(0.0,1.0);

	glGetDoublev(GL_MODELVIEW_MATRIX, &modelview[0] ); //Aktuelle Modelview Matrix in einer Variable ablegen
	glGetDoublev(GL_PROJECTION_MATRIX, &projection[0] ); //Aktuelle Projection[s] Matrix in einer Variable ablegen
	glGetIntegerv(GL_VIEWPORT, &viewport[0] ); // Aktuellen Viewport in einer Variable ablegen
	y_new = viewport[3] - y; // In OpenGL steigt Y von unten (0) nach oben

	// Auslesen des Tiefenpuffers an der Position (X/Y_new)
	glReadPixels(x, y_new, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z );

	// Errechnen des Punktes, welcher mit den beiden Matrizen multipliziert (X/Y_new/Z) ergibt:
	gluUnProject((double)x, (double)y_new, (double)z, modelview, projection, viewport, &result[0], &result[1], &result[2]);


	vResult.x = (float)result[0];
	vResult.y = (float)result[1];
	vResult.z = (float)result[2];

	return vResult;
}

//bool tgEngine::GetNewPoint(vec3& v){
//	if(!p_pressed)
//		return false;
//
//	p_pressed = false;
//
//	vec3 tgv = Get3DPointFrom2D(m_mouse_pos[0], m_mouse_pos[1]);
//
//	v.x =tgv.x;
//	v.y =tgv.y;
//	v.z =tgv.z;
//
//	return true;
//}

