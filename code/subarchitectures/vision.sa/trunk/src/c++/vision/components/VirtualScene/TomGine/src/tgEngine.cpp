
#include "tgEngine.h"

using namespace TomGine;

tgEngine::tgEngine(){
	m_width = 640;
	m_height2D = 30;
	m_height = 480 + m_height2D;
	m_depth = 1.0;
	
	m_frametime = 0.0;
	
	m_window = 0;
	m_window3D = 0;
	m_window2D = 0;
	
	m_button_left = false;
	m_button_middle = false;
	m_button_right = false;
}

tgEngine::~tgEngine(){
	if(m_window2D)	delete(m_window2D);
	if(m_window3D) 	delete(m_window3D);
	if(m_window) 		delete(m_window);
}

bool tgEngine::InitWindow(int width, int height, const char* name){
	m_window = new tgGLXWindow(width, height, name);
}

bool tgEngine::Init(const char* name, int width, int height){
	m_width = (float)width;
	m_height = (float)height+m_height2D;
	
	InitWindow(m_width, m_height, name);
	
	m_window3D = new tgSubWindow3D();
	m_window3D->Init( 0, m_height2D, m_width, m_height-m_height2D, m_depth );
	
	m_window2D = new tgSubWindow2D();
	m_window2D->Init( 0, 0, m_width, m_height2D );
	
	Update(m_frametime);
}

bool tgEngine::Update(float &fTime){
	
	bool quit = InputControl();
	
// 	m_window2D->DrawCoordinates();
	m_window2D->DrawBorders();
	m_window3D->DrawCoordinates();
	
	Swap();
	
	// update frametime
	fTime = m_frametime = m_timer.Update();

	// clear framebuffer and depth buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	return quit;
}

bool tgEngine::InputControl(){
	tgVector3 vPoint;
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
// 						if(m_smoothshading)
// 							glShadeModel(GL_SMOOTH);
// 						else
// 							glShadeModel(GL_FLAT);
// 						m_smoothshading = !m_smoothshading;
						break;
					case XK_p:
						p_pressed = true;
						break;
					case XK_w:
// 						if(m_wireframe)
// 							glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
// 						else
// 							glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
// 						m_wireframe = !m_wireframe;
						break;
					case XK_z:
						m_window3D->ResetCamera();
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
						m_window3D->Camera()->TranslateF(10.0*m_depth*m_frametime);
						break;
					case Button5:
						m_window3D->Camera()->TranslateF(-10.0*m_depth*m_frametime);
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
				
				if(m_button_middle){
					m_window3D->OrbitCamera(-5*m_frametime * event.motion.x_rel, -5*m_frametime * event.motion.y_rel);
				}else if(m_button_right){
					m_window3D->Camera()->TranslateS(-0.2*m_depth*m_frametime*event.motion.x_rel);
					m_window3D->Camera()->TranslateU(0.2*m_depth*m_frametime*event.motion.y_rel);
				}
				break;
				
			// *********************************************************
			case Expose:
// 				m_window3D->Camera()->SetViewport( 0, m_height2D, (float)event.expose.width, (float)event.expose.height-m_height2D);
// 				m_window2D->Camera()->SetViewport(0, 0, (float)event.expose.width, m_height2D);
// 				
// 				printf("expose.width: %d = %f\n", event.expose.width, (float)event.expose.width);
// 				printf("Viewport2D: %f %f\n", m_window2D->Camera()->GetWidth(), m_window2D->Camera()->GetHeight());
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

void tgEngine::Swap(){
	m_window->Swap();
}


