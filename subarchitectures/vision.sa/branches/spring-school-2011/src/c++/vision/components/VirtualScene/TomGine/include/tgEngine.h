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

#include "tgGLXWindow.h"
#include "tgCamera.h"
#include "tgEvent.h"
#include "tgLighting.h"
#include "tgRenderModel.h"
#include "tgPose.h"
#include "tgTimer.h"
#include "tgVector3.h"
#include "tgMathlib.h"

namespace TomGine{

/** @brief Class tgEngine */
class tgEngine
{
private:
	float m_width;
	float m_height;
	float m_far;
	float m_near;
	
	tgGLXWindow* 	m_window;
	tgCamera 			m_cam[6];
	tgCamera			m_camera;
	tgCamera			m_cam_ortho;

	tgLighting 		m_lighting;
	tgTimer 			m_timer;
	tgVector3			m_cor;			///< Center of Rotation
	
	int m_mouse_pos[2];
	bool p_pressed;
	
	float m_frametime;
	bool m_bfc;
	bool m_button_left, m_button_middle, m_button_right;
	bool m_wireframe;
	bool m_smoothshading;
	
	bool InitWindow(int widht, int height, const char* name);
	
public:
	tgEngine();
	~tgEngine();
	
	/** @brief Welcome message */
	void Welcome();

	/** @brief Initialising render engine
	*	@param width Width of rendering window in pixels
	* @param height Height of rendering window in pixels
	* @param depth Depth of object to render (Distance between camera and object to render in meter)
	* @param name Caption of Window
	* @param bfc Enable / Disable back face culling (render back face of polygons or not)
	* @return Success of initialisation */
	bool Init(	int width,
							int height,
							float far,
							float near,
							const char* name="TomGine",
							bool bfc=false);
	
	/** @brief Draws content to  frame screen
	* @param time Time in seconds since last Update() call 
	* @param event GLX event */
	bool Update();
	bool Update(float &fTime);
	bool Update(std::vector<tgEvent> &eventlist);
	bool Update(float &fTime, std::vector<tgEvent> &eventlist);
	
	
	/** @brief Handles keyboard and mouse input applied to this window */
	bool InputControl(tgEvent &event);
	
	/**	@brief Draws a simple coordinate frame */
	void DrawCoordinates();
	
	/** @brief Sets Camera of rendering engine (including internal and external camera parameters) */
	void SetCamera(tgCamera cam);
	void UpdateCameraViews(tgCamera cam);
	
	/**	@brief Sets center of rotation */
	void SetCenterOfRotation(float x, float y, float z);
	
	/** @brief Activates 3D rendering mode; standard after Update() */
	void Activate3D();

	/** @brief Activates 2D rendering moder */
	void Activate2D();
	
	/** @brief Swaps frame buffer to screen (called by Update() aswell)*/
	void Swap();
	
	/**	@brief Returns the actual position of the camera with respect to the coordinate frame */
	tgVector3 GetCameraPosition(){ return m_camera.GetPos(); }

	void GetCamera0(tgCamera &cam){ cam = m_cam[0]; }
	
	bool	GetWireframeMode(){ return m_wireframe; }

	tgVector3 Get3DPointFrom2D(int x, int y);
	bool GetNewPoint(vec3& v);

};

} // namespace TomGine

#endif