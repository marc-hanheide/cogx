
#ifdef LINUX

#include "GLWindow.h"
#include <stdio.h>
#include <stdexcept>
#include <vector>

namespace V4R{

void GLWindow::init(unsigned int width, unsigned int height, const char* name, bool threaded){
	GLint att[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None };
	dpy = XOpenDisplay(NULL);

	if(dpy == NULL){
		throw std::runtime_error("[GLWindow::init] Error cannot connect to X server");
	}
				  
	root = DefaultRootWindow(dpy);
	vi = glXChooseVisual(dpy, 0, att);

	if(vi == NULL)
		throw std::runtime_error("[GLWindow::init] Error no appropriate visual found");

	cmap = XCreateColormap(dpy, root, vi->visual, AllocNone);

	swa.colormap = cmap;
	swa.event_mask = ExposureMask | KeyPressMask | KeyReleaseMask | ButtonPressMask | ButtonReleaseMask | PointerMotionMask | StructureNotifyMask;

	glWin = XCreateWindow(dpy, root, 0, 0, width, height, 0, vi->depth, InputOutput, vi->visual, CWColormap | CWEventMask, &swa);
	wmDelete = XInternAtom(dpy, "WM_DELETE_WINDOW", true);
	XSetWMProtocols(dpy, glWin, &wmDelete, 1);

	XMapWindow(dpy, glWin);
	XStoreName(dpy, glWin, name);

	glc = glXCreateContext(dpy, vi, NULL, GL_TRUE);
	glXMakeCurrent(dpy, glWin, glc);
	
	printf("GLWindow '%s' %dx%d\nOpenGL Version: %s\n", name, width, height, glGetString(GL_VERSION));

	this->threaded = threaded;
	sem_init(&eventSem, 0, 0);

	glXSwapBuffers(dpy, glWin);
}

void GLWindow::quit(){
	glXMakeCurrent(dpy, None, NULL);
	glXDestroyContext(dpy, glc);
	XDestroyWindow(dpy, glWin);
	XFlush(dpy);
	if(threaded)
		sem_wait(&eventSem);
	sem_destroy(&eventSem);
	XCloseDisplay(dpy);
}

GLWindow::GLWindow(){
	init(320,240,"OpenGL Window");
}
GLWindow::GLWindow(unsigned int width, unsigned int height){
	init(width, height, "OpenGL Window");
}
GLWindow::GLWindow(unsigned int width, unsigned int height, const char* name){
	init(width, height, name);
}
GLWindow::GLWindow(unsigned int width, unsigned int height, const char* name, bool threaded){
	init(width, height, name, threaded);
}
GLWindow::~GLWindow(){
	quit();
}

void GLWindow::Activate(){
	glXMakeCurrent(dpy, glWin, glc);
}

void GLWindow::Update(){
	glXSwapBuffers(dpy, glWin);
}

} /* namespace */

#endif /* LINUX */

