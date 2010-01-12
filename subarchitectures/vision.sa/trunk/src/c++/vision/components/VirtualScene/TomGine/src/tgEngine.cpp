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

tgEngine::tgEngine(){
	m_width = 640;
	m_height = 480;
	m_depth = 1.0;
	
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

bool tgEngine::Init(int width, int height, float depth, const char* name, bool bfc){
	m_width = width;
	m_height = height;
	m_depth = depth;
	m_bfc = bfc;
	
	InitWindow(width, height, name);
		
	// Setup camera
	float da = sqrt(pow(depth,2)/3.0);
	m_camera.Set(	da, da, da,									// Position of camera
								0.0, 0.0, 0.0,							// Point where camera looks at
								0.0, 1.0, 0.0,							// UP-Vector of Camera
								45, width, height,					// field of view in degree in y, image width, image height
								0.01, 5.0*depth,					// near clipping plane, far clipping plane
								GL_PERSPECTIVE);						// Perspective camera
	
	m_camera0 = m_camera;
	
	// Setup lights	
	tgLight light0;
	light0.ambient = vec4(0.3,0.3,0.3,1.0);
	light0.diffuse = vec4(1.0,1.0,1.0,1.0);
	light0.specular = vec4(0.2,0.2,0.2,1.0);
	light0.position = vec4(1.0,1.0,1.0,0.0);
	m_lighting.ApplyLight(light0,0);
	
	tgLight light1;
	light1.ambient = vec4(0.3,0.3,0.3,1.0);
	light1.diffuse = vec4(0.7,0.9,1.0,1.0);
	light1.specular = vec4(0.2,0.2,0.2,1.0);
	light1.position = vec4(-1.0,0.0,1.0,0.0);
	m_lighting.ApplyLight(light1,1);
	
	if(m_bfc) glEnable(GL_CULL_FACE);
	else			glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
}

bool tgEngine::Update(float &fTime){
	
	bool quit = InputControl();
	m_camera.ApplyTransform();
	m_camera.Activate();
	//m_lighting.Activate();
	
	DrawCoordinates();
	Swap();
	
	// update frametime
	fTime = m_frametime = m_timer.Update();
	
	// OpenGL Render settings
	if(m_bfc) glEnable(GL_CULL_FACE);
	else			glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	
	// clear framebuffer and depth buffer
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	
	return quit;
}

bool tgEngine::InputControl(){
	tgEvent event;
	while(m_window->CheckXEvent(event)){
		switch(event.type){
		
			// *********************************************************
			case KeyPress:
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
					case XK_w:
						if(m_wireframe)
							glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
						else
							glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
						m_wireframe = !m_wireframe;
						break;
					case XK_z:
						m_camera = m_camera0;
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
						m_camera.TranslateF(10.0*m_depth*m_frametime);
						break;
					case Button5:
						m_camera.TranslateF(-10.0*m_depth*m_frametime);
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
				if(m_button_left){
					m_camera.Orbit(m_cor, m_camera.GetU(), -5*m_frametime * event.motion.x_rel);
					m_camera.Orbit(m_cor, m_camera.GetS(), -5*m_frametime * event.motion.y_rel);					
				}else if(m_button_right){
					m_camera.TranslateS(-0.2*m_depth*m_frametime*event.motion.x_rel);
					m_camera.TranslateU(0.2*m_depth*m_frametime*event.motion.y_rel);
				}
				break;
				
			// *********************************************************
			case Expose:
			/*
				m_camera0.SetIntrinsic(	m_camera.GetFOVY(), (float)event.expose.width, (float)event.expose.height,
																m_camera.GetZNear(), m_camera.GetZFar(),
																m_camera.GetProjection());
				m_camera.SetIntrinsic(	m_camera.GetFOVY(), (float)event.expose.width, (float)event.expose.height,
																m_camera.GetZNear(), m_camera.GetZFar(),
																m_camera.GetProjection());
			*/
				break;
				
			// *********************************************************
			case ClientMessage:
				if(event.clientmessage.stop)
					return false;
				break;
				
		} // switch(event.type)
	} // while(m_window->CheckXEvent(event))
	return true;
}

void tgEngine::DrawCoordinates(){
	float l1 = 0.1*m_depth;
	
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

void tgEngine::Swap(){
	m_window->Swap();
	
	// swap OpenGL framebuffer
// 	SDL_GL_SwapBuffers();
// 	SDL_Delay(5);	
}

