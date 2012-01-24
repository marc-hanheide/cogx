
#include "GLWindow/GLWindow.h"
#include <GL/glu.h>
#include <stdio.h>

float rtri = 0.0f;

void drawTriangle(){
	// Move
	glPushMatrix();
	glTranslatef(0.0f,0.0f,-6.0f);				
	glRotatef(rtri++,0.0f,1.0f,0.0f);
	
	// Draw a triangle
	glBegin(GL_TRIANGLES);						
		glColor3f(1.0f,0.0f,0.0f);
		glVertex3f( 0.0f, 1.0f, 0.0f);
		glColor3f(0.0f,1.0f,0.0f);
		glVertex3f(-1.0f,-1.0f, 0.0f);
		glColor3f(0.0f,0.0f,1.0f);
		glVertex3f( 1.0f,-1.0f, 0.0f);
	glEnd();
	glPopMatrix();
}

int main(int argc, char *argv[])
{
	// Create OpenGL Window
	unsigned w = 800;	// width of OpenGL window
	unsigned h = 600;	// height of OpenGL window
	blortGLWindow::GLWindow window(w, h, "OpenGL Window");
	
	printf("\n Demo GLWindow\n\n");
	
	// Setup OpenGL
	glViewport(0,0,w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f,(GLfloat)w/(GLfloat)h,0.1f,100.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();	
	
	// Rendering Loop
	{
		blortGLWindow::Event event;
		bool quit = false;
		while( !quit ){
	
			// Draw Triangle in window 1
			window.Activate();					// Activate Window
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
			drawTriangle();	// Draw Triangle
			window.Update();						// Swap OpenGL Buffer

			// Get Events occured since last frame
			while(window.GetEvent(event)){
				if(event.type == blortGLWindow::TMGL_Press && event.input == blortGLWindow::TMGL_Escape)
					quit = true;
			}
			
#ifdef LINUX
			usleep(10000);
#endif
	
#ifdef WIN32
			Sleep(10);
#endif

		}
	}

	return 0;
}
