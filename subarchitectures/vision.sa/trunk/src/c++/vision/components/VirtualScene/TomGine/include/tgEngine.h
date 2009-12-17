//HEADER:
//			Title:			class tgEngine
//			File:				tgEngine.h
//
//			Function:		Main file of Engine providing interfaces
//
//			Author:			Thomas Mörwald
//			Date:				20.11.2009
// ----------------------------------------------------------------------------

#ifndef TG_ENGINE
#define TG_ENGINE

#include <stdio.h>
#include <SDL/SDL.h>
#include <GL/gl.h>

#include "Timer.h"
#include "tgCamera.h"
#include "tgLighting.h"
#include "tgTexture.h"
#include "tgModel.h"

class tgEngine
{
private:
	float m_width;
	float m_height;
	float m_depth;
	
	tgCamera m_camera;
	tgLighting m_lighting;
	Timer m_timer;
	
	float m_frametime;
	bool m_button_left, m_button_middle, m_button_right;
	bool m_wireframe;
	bool m_smoothshading;
	
	bool InitWindow(int widht, int height);
	
public:
	tgEngine();
	~tgEngine();
	
	void Welcome();
	
	bool Init(	int width,			// width of window in pixel
							int height,			// height of window in pixel
							float depth);		// distance from object/render to camera (virtual) in units (whatever)
	
	bool Update();
	
	bool InputControl();
	
	void DrawCoordinates();
	
	void Swap();
	
	tgVector3 GetCameraPosition(){ return m_camera.GetPos(); }

};

#endif