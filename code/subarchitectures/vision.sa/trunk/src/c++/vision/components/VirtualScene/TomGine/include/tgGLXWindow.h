//HEADER:
//			Title:			class tgGLXWindow
//			File:				tgGLXWindow.h
//
//			Function:		Handling GLX window
//			Manual:			http://tronche.com/gui/x/xlib
//
//			Author:			Thomas Mörwald
//			Date:				20.12.2009
// ----------------------------------------------------------------------------



#ifndef TG_WINDOW
#define TG_WINDOW

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/gl.h>
#include <GL/glx.h>

#include "tgEvent.h"

class tgGLXWindow{
private:
  Display                 *dpy;
	Window                  root;
	XVisualInfo             *vi;
	Colormap                cmap;
	XSetWindowAttributes    swa;
	Window                  win;
	Atom										wmDelete;
	GLXContext              glc;
	XWindowAttributes       gwa;
	XEvent                  xev;
	XMotionEvent						xmotev;
  
public:
	tgGLXWindow(int width, int height, const char* name="GLX Window");
	~tgGLXWindow();
	void Swap();
	
	bool CheckXEvent(tgEvent &event);
};

#endif
