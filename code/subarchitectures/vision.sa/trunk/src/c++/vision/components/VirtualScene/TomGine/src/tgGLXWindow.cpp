//SOURCE:
//			Title:			class tgGLXWindow
//			File:				tgGLXWindow.cpp
//
//			Function:		Handling GLX window
//
//			Links:			http://tronche.com/gui/x/xlib
//									http://www.opengl.org/wiki/Programming_OpenGL_in_Linux:_GLX_and_Xlib
//
//			KeyCode:		/usr/include/X11/keysymdef.h
//
//			Author:			Thomas Mörwald
//			Date:				20.12.2009
// ----------------------------------------------------------------------------

#include "tgGLXWindow.h"
#include <stdexcept>

using namespace TomGine;

tgGLXWindow::tgGLXWindow(int width, int height, const char* name){
	GLint att[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None };
	dpy = XOpenDisplay(NULL);
 
	if(dpy == NULL)
		throw std::runtime_error("[tgGLXWindow::tgGLXWindow] XOpenDisplay(): Cannot connect to X server");		
			
	root = DefaultRootWindow(dpy);
	vi = glXChooseVisual(dpy, 0, att);
	
	if(vi == NULL)
		throw std::runtime_error("[tgGLXWindow::tgGLXWindow] glXChooseVisual(): No appropriate visual found");

	
	cmap = XCreateColormap(dpy, root, vi->visual, AllocNone);
	
	swa.colormap = cmap;
	swa.event_mask = ExposureMask | KeyPressMask | ButtonPressMask | ButtonReleaseMask | PointerMotionMask;
	
	glWin = XCreateWindow(dpy, root, 0, 0, width, height, 0, vi->depth, InputOutput, vi->visual, CWColormap | CWEventMask, &swa);
	wmDelete = XInternAtom(dpy, "WM_DELETE_WINDOW", true);
	XSetWMProtocols(dpy, glWin, &wmDelete, 1);
	
	XMapWindow(dpy, glWin);
	XStoreName(dpy, glWin, name);
	
	glc = glXCreateContext(dpy, vi, NULL, GL_TRUE);
	glXMakeCurrent(dpy, glWin, glc);
}

tgGLXWindow::~tgGLXWindow(){
	glXMakeCurrent(dpy, None, NULL);
	glXDestroyContext(dpy, glc);
	XDestroyWindow(dpy, glWin);
	XCloseDisplay(dpy);
}

void tgGLXWindow::Swap(){
	glXSwapBuffers(dpy, glWin);
	usleep(10000);
}

bool tgGLXWindow::CheckXEvent(tgEvent &event){
	
	if(XPending(dpy)){
		
		if(XCheckTypedWindowEvent(dpy, glWin, ClientMessage, &xev)){
			if(*XGetAtomName(dpy, xev.xclient.message_type) == *"WM_PROTOCOLS"){
				event.type = ClientMessage;
				event.clientmessage.stop = true;
			}
		}else if(XCheckWindowEvent(dpy, glWin, KeyPressMask, &xev)){
			// Keysym code available in /usr/include/X11/keysymdef.h
			event.type = xev.type;
			event.key.keysym = XKeycodeToKeysym(dpy, xev.xkey.keycode, 0);
		}else	if(XCheckWindowEvent(dpy, glWin, ButtonReleaseMask, &xev)){
			event.type = xev.type;
			event.button.button = xev.xbutton.button;
		}else if(XCheckWindowEvent(dpy, glWin, ButtonPressMask, &xev)){
			event.type = xev.type;
			event.button.button = xev.xbutton.button;
		}else if(XCheckWindowEvent(dpy, glWin, PointerMotionMask, &xev)){
			event.type = xev.type;
			event.motion.x = xev.xmotion.x;
			event.motion.y = xev.xmotion.y;
			event.motion.x_rel = xev.xmotion.x-xmotev.x;
			event.motion.y_rel = xev.xmotion.y-xmotev.y;
			xmotev = xev.xmotion;
		}else if(XCheckWindowEvent(dpy, glWin, ExposureMask, &xev)){
			event.type = xev.type;
			XGetWindowAttributes(dpy, glWin, &gwa);
			event.expose.width = gwa.width;
			event.expose.height = gwa.height;
		}
		return true;
	}
	return false;
}


