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
#include "tgVector3.h"

/**
* @brief Class tgEngine
*/
class tgEngine
{
private:
	float m_width;
	float m_height;
	float m_depth;
	
	tgGLXWindow* 	m_window;
	tgCamera 			m_camera;
	tgCamera 			m_camera0;
	tgLighting 		m_lighting;
	Timer 				m_timer;
	tgVector3			m_cor;			///< Center of Rotation
	
	float m_frametime;
	bool m_bfc;
	bool m_button_left, m_button_middle, m_button_right;
	bool m_wireframe;
	bool m_smoothshading;
	
	bool InitWindow(int widht, int height, const char* name);
	
public:
	tgEngine();
	~tgEngine();
	
	/**
	* @brief Welcome message
	*/
	void Welcome();

	/**
	* @brief Initialising render engine
	*	@param width Width of rendering window in pixels
	* @param height Height of rendering window in pixels
	* @param depth Depth of object to render (Distance between camera and object to render in meter)
	* @param name Caption of Window
	* @param bfc Enable / Disable back face culling (render back face of polygons or not)
	* @return Success of initialisation
	*/
	bool Init(	int width,
							int height,
							float depth,
							const char* name="TomGine",
							bool bfc=false);
	
	/**
	* @brief Draws content to  frame screen
	* @param time Time in seconds since last Update() call
	*/
	bool Update(float &fTime);
	
	/**
	* @brief Handles keyboard and mouse input applied to this window
	*/
	bool InputControl();
	
	/**
	*	@brief Draws a simple coordinate frame
	*/
	void DrawCoordinates();
	
	/**
	* @brief Sets Camera of rendering engine (including internal and external camera parameters)
	*/
	void SetCamera(tgCamera cam){ m_camera = cam; m_camera0 = cam; }
	
	/**
	*	@brief Sets center of rotation
	*/
	void SetCenterOfRotation(float x, float y, float z){ m_cor = tgVector3(x,y,z); }
	
	/**
	* @brief Swaps frame buffer to screen (called by Update() aswell)
	*/
	void Swap();
	
	/**
	*	@brief Returns the actual position of the camera with respect to the coordinate frame
	*/
	tgVector3 GetCameraPosition(){ return m_camera.GetPos(); }

};

#endif