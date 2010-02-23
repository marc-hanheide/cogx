 /**
 * @file tgEngine.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Main file of rendering engine 'TomGine'.
 * @namespace TomGine
 */

#ifndef TG_ENGINE
#define TG_ENGINE

#include <stdio.h>
#include <GL/gl.h>

#include "tgEvent.h"
#include "tgGLXWindow.h"
#include "tgSubWindow2D.h"
#include "tgSubWindow3D.h"
#include "tgTimer.h"

namespace TomGine{

/**
* @brief Class tgEngine
*/
class tgEngine
{
private:
	float m_width;
	float m_height;
	float m_depth;
	float m_height2D;
	
	tgGLXWindow* 			m_window;
	tgSubWindow3D*		m_window3D;
	tgSubWindow2D*		m_window2D;
	tgTimer 					m_timer;
	
	int m_mouse_pos[2];
	bool p_pressed;
	
	float m_frametime;

	bool m_button_left, m_button_middle, m_button_right;
	
	bool InitWindow(int widht, int height, const char* name);
	
public:
	tgEngine();
	~tgEngine();
	
	/**
	* @brief Initialising render engine
	*	@param width Width of rendering window in pixels
	* @param height Height of rendering window in pixels
	* @param depth Depth of object to render (Distance between camera and object to render in meter)
	* @param name Caption of Window
	* @param bfc Enable / Disable back face culling (render back face of polygons or not)
	* @return Success of initialisation
	*/
	bool Init(const char* name="TomGine", int width=640, int height=480);
	
	
	/**
	* @brief Draws content to  frame screen
	* @param time Time in seconds since last Update() call
	*/
	bool Update(float &fTime);
	
	/** @brief Handles keyboard and mouse input applied to this window */
	bool InputControl();
	
	void Activate2D(){ m_window2D->Activate(); }
	void Activate3D(){ m_window3D->Activate(); }
	
	void SetCenterOfRotation(float x, float y, float z){ m_window3D->SetCenterOfRotation(x,y,z); }
	
	void SetCamera3D(tgCamera cam){ m_window3D->SetCamera(cam); }
	
	/** @brief Swaps frame buffer to screen (called by Update() aswell) */
	void Swap();
	

};

} // namespace TomGine

#endif