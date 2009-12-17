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
	
	m_button_left = false;
	m_button_middle = false;
	m_button_right = false;
	
	m_wireframe = false;
	m_smoothshading = true;
	
	Welcome();
	
}

tgEngine::~tgEngine(){
	SDL_Quit();
}

void tgEngine::Welcome(){
	printf("\n   ***   TomGine Render Engine   ***\n\n");
	printf("rotate: left mouse button\n");
	printf("slide:  right mouse button\n");
	printf("zoom:   mouse wheel\n");
	printf("f:      toggle shading mode\n");
	printf("w:      draw wireframed\n");
	printf("q/esc:	quit\n");
	printf("\n");
}

bool tgEngine::InitWindow(int width, int height){
	// Initialize SDL	
	if((SDL_Init(SDL_INIT_VIDEO)==-1)) { 
		printf("[Resources::GetScreen] Error could not initialize SDL: %s\n", SDL_GetError());
		return false;
	}
	
	// Set mode for video output (width, height, resolution)
	if ( !SDL_SetVideoMode( width, height, 32, SDL_OPENGL )) {
		printf( "[Resources::GetScreen] Error setting video mode: %s\n", SDL_GetError( ) );
		SDL_Quit();
		return false;
	}
}

bool tgEngine::Init(int width, int height, float depth){
	m_width = width;
	m_height = height;
	m_depth = depth;
	
	InitWindow(width, height);
		
	// Setup camera
	float da = sqrt(pow(depth,2)/3.0);
	m_camera.Set(	da, da, da,									// Position of camera
								0.0, 0.0, 0.0,							// Point where camera looks at
								0.0, 1.0, 0.0,							// UP-Vector of Camera
								45, width, height,					// field of view in degree in y, image width, image height
								0.01, 5.0*depth,					// near clipping plane, far clipping plane
								GL_PERSPECTIVE);						// Perspective camera
	
	// Setup Lighting	
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
	
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
}

bool tgEngine::Update(){
	
	bool quit = InputControl();
	m_camera.ApplyTransform();
	m_camera.Activate();
	//m_lighting.Activate();
	
	DrawCoordinates();
	Swap();
	
	// update frametime
	m_frametime = m_timer.Update();
	
	// clear framebuffer and depth buffer
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	
	if(m_smoothshading)
		glShadeModel(GL_SMOOTH);
	else
		glShadeModel(GL_FLAT);
	
	
	return quit;
}

bool tgEngine::InputControl(){
 	SDL_Event event;
	while(SDL_PollEvent(&event)){
		switch(event.type){
			
			// Keyboard input
			case SDL_KEYDOWN:
				switch(event.key.keysym.sym){
					case SDLK_ESCAPE:
						return false;
						break;
					case SDLK_f:
						m_smoothshading = !m_smoothshading;
						break;
					case SDLK_q:
						return false;
					case SDLK_w:
						if(m_wireframe)
							glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
						else
							glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
						m_wireframe = !m_wireframe;
						break;
					default:
						break;
				}
				break;
				
			// Mouse motion
			case SDL_MOUSEMOTION:
				if(m_button_left){
					m_camera.Orbit(tgVector3(0.0,0.0,0.0), m_camera.GetU(), -5*m_frametime * event.motion.xrel);
					m_camera.Orbit(tgVector3(0.0,0.0,0.0), m_camera.GetS(), -5*m_frametime * event.motion.yrel);					
				}else if(m_button_right){
					m_camera.TranslateS(-0.2*m_depth*m_frametime*event.motion.xrel);
					m_camera.TranslateU(0.2*m_depth*m_frametime*event.motion.yrel);
				}
				break;
			
			// Mouse button
			case SDL_MOUSEBUTTONDOWN:
				switch(event.button.button){
					case SDL_BUTTON_LEFT:
						m_button_left = true;
						break;
					case SDL_BUTTON_MIDDLE:
						m_button_middle = true;
						break;
					case SDL_BUTTON_RIGHT:
						m_button_right = true;
						break;
					case SDL_BUTTON_WHEELUP:
						m_camera.TranslateF(10.0*m_depth*m_frametime);
						break;
					case SDL_BUTTON_WHEELDOWN:
						m_camera.TranslateF(-10.0*m_depth*m_frametime);
						break;
				}
				break;
			case SDL_MOUSEBUTTONUP:
				switch(event.button.button){
					case SDL_BUTTON_LEFT:
						m_button_left = false;
						break;
					case SDL_BUTTON_MIDDLE:
						m_button_middle = false;
						break;
					case SDL_BUTTON_RIGHT:
						m_button_right = false;
						break;
				}
				break;
				
			
			case SDL_QUIT:
				return false;
				break;
			default:
				break;
		}
	}

	return true;
}

void tgEngine::DrawCoordinates(){
	float l1 = 0.1*m_depth;
	
	m_lighting.Deactivate();
	glDisable(GL_CULL_FACE);
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
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);	
}

void tgEngine::Swap(){
	
	// swap OpenGL framebuffer
	SDL_GL_SwapBuffers();
	SDL_Delay(5);	
}

