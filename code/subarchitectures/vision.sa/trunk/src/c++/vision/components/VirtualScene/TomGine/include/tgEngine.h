 /**
 * @file tgEngine.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Main file of rendering engine 'TomGine'.
 */

#ifndef TG_ENGINE
#define TG_ENGINE

#include <stdio.h>
#include <SDL/SDL.h>
#include <GL/gl.h>

#include "tgGLXWindow.h"
#include "Timer.h"
#include "tgCamera.h"
#include "tgLighting.h"
#include "tgTexture.h"
#include "tgModel.h"
#include "tgPose.h"
#include "tgEvent.h"

class tgEngine
{
private:
	float m_width;
	float m_height;
	float m_depth;
	
	tgGLXWindow* m_window;
	tgCamera m_camera;
	tgCamera m_camera0;
	tgLighting m_lighting;
	Timer m_timer;
	
	float m_frametime;
	bool m_bfc;
	bool m_button_left, m_button_middle, m_button_right;
	bool m_wireframe;
	bool m_smoothshading;
	
	bool InitWindow(int widht, int height, const char* name);
	
public:
	tgEngine();
	~tgEngine();
	
	void Welcome();
	
	bool Init(	int width,										// width of window in pixel
							int height,										// height of window in pixel
							float depth,									// distance from object/render to camera (virtual) in units (whatever)
							const char* name="TomGine",		// caption of X Window
							bool bfc=false);							// enable/disable backface culling
	
	bool Update(float &fTime);
	
	bool InputControl();
	
	void DrawCoordinates();
	
	void SetCamera(tgCamera cam){ m_camera = cam; m_camera0 = cam; }
	
	void Swap();
	
	tgVector3 GetCameraPosition(){ return m_camera.GetPos(); }

};

#endif