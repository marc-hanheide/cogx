 /**
 * @file tgGLXWindow.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief GLX Window management for handling windows.
 */



#ifndef TG_WINDOW
#define TG_WINDOW

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/gl.h>
#include <GL/glx.h>

#include "tgEvent.h"

namespace TomGine{

/** @brief Class tgGLXWindow */
class tgGLXWindow{
private:
  Display                 *dpy;
	Window                  root;
	XVisualInfo             *vi;
	Colormap                cmap;
	XSetWindowAttributes    swa;
	Window                  glWin;
	Window									btWin;
	Atom										wmDelete;
	GLXContext              glc;
	XWindowAttributes       gwa;
	XEvent                  xev;
	XMotionEvent						xmotev;
  
public:
	tgGLXWindow(int width, int height, const char* name="GLX Window");
	~tgGLXWindow();
	
	/** @brief Activate window for usage */
	void Activate();

	/** @brief Swap OpenGL Buffer */
	void Swap();
	
	/** @brief Query event from GLX */
	bool CheckXEvent(tgEvent &event);
};

} // namespace TomGine

#endif
