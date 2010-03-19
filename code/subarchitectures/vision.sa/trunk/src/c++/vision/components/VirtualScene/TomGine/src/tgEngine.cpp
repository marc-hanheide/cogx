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

tgEngine::tgEngine(){
	m_width = 640;
	m_height = 480;
	m_near = 0.1;
	m_far = 10.0;
	
	m_frametime = 0.0;
	
	m_cor = tgVector3(0.0,0.0,0.0);
	
	m_button_left = false;
	m_button_middle = false;
	m_button_right = false;
	
	m_wireframe = false;
	m_smoothshading = false;
	
// 	Welcome();
}

tgEngine::~tgEngine(){
	delete(m_window);
}

void tgEngine::Welcome(){
	printf("\n   ***   TomGine Render Engine   ***\n\n");
	printf("rotate: left mouse button\n");
	printf("slide:  right mouse button\n");
	printf("zoom:   mouse wheel\n");
	printf("f:      toggle shading mode\n");
	printf("w:      draw wireframed\n");
	printf("esc:	quit\n");
	printf("\n");
}

bool tgEngine::InitWindow(int width, int height, const char* name){	
	m_window = new tgGLXWindow(width, height, name);
}

bool tgEngine::Init(int width, int height, float far, float near, const char* name, bool bfc){
	m_width = width;
	m_height = height;
	m_far = far;
	m_near = near;
	m_bfc = bfc;
	
	// Create render context
	InitWindow(width, height, name);
	
	float da = 0.25*(m_far-m_near);
	
	// Setup 3D camera
	m_camera.Set(	da, da, da,									// Position of camera
								0.0, 0.0, 0.0,							// Point where camera looks at
								0.0, 1.0, 0.0,							// UP-Vector of Camera
								45, width, height,					// field of view in degree in y, image width, image height
								near, far,									// near clipping plane, far clipping plane
								GL_PERSPECTIVE);						// Perspective camera
	m_cam[5] = m_cam[4] = m_cam[3] = m_cam[2] = m_cam[1] = m_cam[0] = m_camera;
	
	m_cam[1].Orbit(m_cor, m_cam[1].GetU(), PI);
	m_cam[2].Orbit(m_cor, m_cam[2].GetU(), PI*0.5);
	m_cam[3].Orbit(m_cor, m_cam[3].GetU(),-PI*0.5);
	m_cam[4].Orbit(m_cor, m_cam[4].GetS(), PI*0.5);
	m_cam[5].Orbit(m_cor, m_cam[5].GetS(),-PI*0.5);
	
	
	
	// Setup 2D camera
	m_cam_ortho.Set(	0.0, 0.0, 1.0,
										0.0, 0.0, 0.0,
										0.0, 1.0, 0.0,
										45, width, height,
										0.1, 2.0,
										GL_ORTHO);
	
	glLineWidth(2);
}

bool tgEngine::Update(){
	float fTime;
	std::vector<tgEvent> eventlist;
	return Update(fTime, eventlist);
}

bool tgEngine::Update(float &fTime){
	std::vector<tgEvent> eventlist;
	return Update(fTime, eventlist);
}

bool tgEngine::Update(std::vector<tgEvent> &eventlist){
	float fTime;
	return Update(fTime, eventlist);
}
	
bool tgEngine::Update(float &fTime, std::vector<tgEvent> &eventlist){
	
	// User input handling (keyboard, mouse)
	bool quit = true;
	tgEvent event;
	while(m_window->CheckXEvent(event)){
		quit = InputControl(event);
		if(quit==false)
			return false;
		eventlist.push_back(event);
	}
	
	Activate3D();
	DrawCoordinates();
	
	Swap();
	
	// update frametime
	fTime = m_frametime = m_timer.Update();

	// clear framebuffer and depth buffer
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	
	// Light pointing along camera viewing axis
	tgLight light;
	tgVector3 cam_f = m_camera.GetF();
	light.ambient = vec4(0.4,0.4,0.4,1.0);
	light.diffuse = vec4(1.0,1.0,1.0,1.0);
	light.specular = vec4(1.0,1.0,1.0,1.0);
	light.position = vec4(-cam_f.x, -cam_f.y, -cam_f.z,1.0);
	m_lighting.ApplyLight(light,0);
	
	return quit;
}

bool tgEngine::InputControl(tgEvent &event){
	switch(event.type){
	
		// *********************************************************
		//			KeyCode:		/usr/include/X11/keysymdef.h
		case KeyPress:
// 			printf("event.key.keysym: %x\n", event.key.keysym);
			switch(event.key.keysym){
				case XK_Escape:
					return false;
					break;
				case XK_f:
					if(m_smoothshading)
						glShadeModel(GL_SMOOTH);
					else
						glShadeModel(GL_FLAT);
					m_smoothshading = !m_smoothshading;
					break;
				case XK_p:
					p_pressed = true;
					break;
				case XK_w:
					if(m_wireframe)
						glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
					else
						glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
					m_wireframe = !m_wireframe;
					break;
				case XK_KP_Insert:
				case XK_z:
					m_camera = m_cam[0];
					break;
				case XK_KP_Begin:
					m_camera = m_cam[1];
					break;
				case XK_KP_Right:
					m_camera = m_cam[2];
					break;
				case XK_KP_Left:
					m_camera = m_cam[3];
					break;
				case XK_KP_Down:
					m_camera = m_cam[4];
					break;
				case XK_KP_Up:
					m_camera = m_cam[5];
					break;
				default:
					break;
			}
			break;
			
		// *********************************************************
		case ButtonPress:
			switch(event.button.button){
				case Button1:
					m_button_left = true;
					break;
				case Button2:
					m_button_middle = true;
					break;
				case Button3:
					m_button_right = true;
					break;
				case Button4:
					m_camera.TranslateF(0.001*(m_far-m_near));
					break;
				case Button5:
					m_camera.TranslateF(-0.001*(m_far-m_near));
					break;
			}
			break;
			
		// *********************************************************
		case ButtonRelease:
			switch(event.button.button){
				case Button1:
					m_button_left = false;
					break;
				case Button2:
					m_button_middle = false;
					break;
				case Button3:
					m_button_right = false;
					break;
			}
			break;
			
		// *********************************************************
		case MotionNotify:
			m_mouse_pos[0] = event.motion.x;
			m_mouse_pos[1] = event.motion.y;
			
			if(m_button_left){
				m_camera.Orbit(m_cor, m_camera.GetU(), -0.05 * event.motion.x_rel);
				m_camera.Orbit(m_cor, m_camera.GetS(), -0.05 * event.motion.y_rel);					
			}else if(m_button_right){
				m_camera.TranslateS(-0.0003*(m_far-m_near)*event.motion.x_rel);
				m_camera.TranslateU(0.0003*(m_far-m_near)*event.motion.y_rel);
			}
			break;
			
		// *********************************************************
		case Expose:
			m_camera.SetViewport((float)event.expose.width, (float)event.expose.height);
			m_cam[0].SetViewport((float)event.expose.width, (float)event.expose.height);
			m_cam[1].SetViewport((float)event.expose.width, (float)event.expose.height);
			m_cam[2].SetViewport((float)event.expose.width, (float)event.expose.height);
			m_cam[3].SetViewport((float)event.expose.width, (float)event.expose.height);
			m_cam[4].SetViewport((float)event.expose.width, (float)event.expose.height);
			m_cam[5].SetViewport((float)event.expose.width, (float)event.expose.height);
			m_cam[6].SetViewport((float)event.expose.width, (float)event.expose.height);
			m_cam[7].SetViewport((float)event.expose.width, (float)event.expose.height);
			m_cam[8].SetViewport((float)event.expose.width, (float)event.expose.height);
			break;
			
		// *********************************************************
		case ClientMessage:
			if(event.clientmessage.stop)
				return false;
			break;
			
	} // switch(event.type)
return true;
}

void tgEngine::DrawCoordinates(){
	float l1 = 0.01*(m_far-m_near);
	
	m_lighting.Deactivate();
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_TEXTURE_2D);
		
	glBegin(GL_LINES);
		glColor3f(1.0,0.0,0.0);
		glVertex3f(0.0,0.0,0.0); glVertex3f(l1, 0.0, 0.0);
		glColor3f(0.0,1.0,0.0);
		glVertex3f(0.0,0.0,0.0); glVertex3f(0.0, l1, 0.0);
		glColor3f(0.0,0.0,1.0);
		glVertex3f(0.0,0.0,0.0); glVertex3f(0.0, 0.0, l1);
	glEnd();
	
	m_lighting.Activate();
	glEnable(GL_DEPTH_TEST);	
}

void tgEngine::SetCamera(tgCamera cam){
	m_camera = cam;	
}

void tgEngine::UpdateCameraViews(tgCamera cam){
	m_cam[5] = m_cam[4] = m_cam[3] = m_cam[2] = m_cam[1] = m_cam[0] = cam;	
	m_cam[1].Orbit(m_cor, m_cam[1].GetU(), PI);
	m_cam[2].Orbit(m_cor, m_cam[2].GetU(), PI*0.5);
	m_cam[3].Orbit(m_cor, m_cam[3].GetU(),-PI*0.5);
	m_cam[4].Orbit(m_cor, m_cam[4].GetS(), PI*0.5);
	m_cam[5].Orbit(m_cor, m_cam[5].GetS(),-PI*0.5);
}

void tgEngine::SetCenterOfRotation(float x, float y, float z){
	m_cor = tgVector3(x,y,z);
}

void tgEngine::Activate3D(){
	if(m_bfc) glEnable(GL_CULL_FACE);
	else			glDisable(GL_CULL_FACE);
	m_camera.ApplyTransform();
	m_camera.Activate();
	m_lighting.Activate();
}

void tgEngine::Activate2D(){
	glDisable(GL_CULL_FACE);
	m_cam_ortho.Activate();
	m_lighting.Deactivate();
}

void tgEngine::Swap(){
	m_window->Swap();
}

tgVector3 tgEngine::Get3DPointFrom2D(int x, int y){
	tgVector3 vResult;
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
  
    
  vResult.x = result[0];
  vResult.y = result[1];
  vResult.z = result[2];
 
 	return vResult;
}

bool tgEngine::GetNewPoint(vec3& v){
	if(!p_pressed)
		return false;
	
	p_pressed = false;
	
	tgVector3 tgv = Get3DPointFrom2D(m_mouse_pos[0], m_mouse_pos[1]);
	
	v.x =tgv.x;
	v.y =tgv.y;
	v.z =tgv.z;

	return true;
}

