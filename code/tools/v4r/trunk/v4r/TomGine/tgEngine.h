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

#include "headers.h"

#include <opencv2/core/core.hpp>

#include <v4r/GLWindow/GLWindow.h>
#include "tgCamera.h"
#include "tgLight.h"
#include "tgRenderModel.h"
#include "tgPose.h"
#include "tgTimer.h"
#include "tgMathlib.h"
#include "tgTexture.h"
#include "tgFont.h"

namespace TomGine{

/** @brief Render engine, OpenGL viewer. Manages camera movement, mouse- and keyboard-input, fonts and visualisation.
 *  This is no scene-graph. */
class tgEngine
{
private:
	unsigned m_width;
	unsigned m_height;
	float m_far;
	float m_near;
	
	float m_input_rotation_speed;
	float m_input_translation_speed;
	float m_input_zoom_speed;
	
	tgTexture2D* m_background_image;

	tgCamera 		m_cam[6];
	tgCamera		m_camera;
	tgCamera		m_cam_ortho;

	tgLight			m_light0;
	tgMaterial		m_material;
	tgTimer 		m_timer;
	vec3			m_cor;			///< Center of Rotation
	
	int m_mouse_pos[2];
	bool p_pressed;
	
	float m_frametime;
	bool m_bfc;
	bool m_button_left, m_button_middle, m_button_right;
	bool m_wireframe;
	bool m_smoothshading;
	bool m_show_background_image;
	bool m_flip_background_image;
	
// 	FTTextureFont* m_font_tf;
//	tgFont m_font;

public:
	/** @brief Initialising render engine.
	 * Warning: OpenGL commands can only be used after initialisation and are not effective before.
	* @param width		Width of rendering window in pixels
	* @param height 	Height of rendering window in pixels
	* @param far 		Far clipping plane (distance above which objects are cut of).
	* @param near 		Near clipping plane (distance below which objects are cut of).
	* @param name 		Caption of Window.
	* @param bfc 		Enable / Disable back face culling (render back face of polygons or not).
	* @param threaded 	true if 'm_window' is threaded, false if not (please see V4R::GLWindow for further information). */
	tgEngine(	unsigned width=640,
				unsigned height=480,
				float far=10.0f,
				float near=0.01f,
				const char* name="TomGine",
				bool bfc=false,
				bool threaded=false);

	/** @brief Destroy render engine.
	 * Warning: OpenGL commands can only be used before destruction and are not effective afterwards. */
	~tgEngine();
	
	V4R::GLWindow* m_window;	///< GLX window embedding the OpenGL context used (see V4R::GLWindow() for more information).

	/** @brief Welcome message */
	void Welcome();
	
	/** @brief Swaps background buffer to screen and resets rendering settings.
	* @param time Time in seconds since last Update() call  */
	void Update(float &fTime);
	/** @brief Swaps background buffer to screen and resets rendering settings. */
	void Update();
	
	/** @brief Apply keyboard and mouse input. (i.e. rotate, translate, zoom, draw wireframe, change cam, ...) */
	bool ProcessEvents();
	/** @brief Get list of events waiting in event que of this window. */
	bool GetEventList(std::vector<V4R::Event> &eventlist);
	/** @brief Wait for event in event que of this window (blocking). */
	void WaitForEvent(V4R::Event &event);

	/** @brief Draw background image. */
	void DrawBackgroundImage();

	/** @brief Handles keyboard and mouse input applied to this window */
	bool InputControl(V4R::Event &event);
	
	/**	@brief Draws a simple coordinate frame */
	void DrawCoordinates(float linelength = 1.0f, float linewidth = 1.0f);
	
	/** @brief Display frames-per-second. */
	void DrawFPS();

	/** @brief Sets camera of rendering engine (including internal and external camera parameters) */
	void SetCamera(tgCamera cam);
	/** @brief Sets camera of rendering engine (including internal and external camera parameters)
	 *  @param intrinsic	Intrinsic camera matrix (projective transformation from world in image space).
	 *  @param width		Width of camera image in pixel
	 *  @param height		Height of camera image in pixel
	 *  @param R,T			Extrinsic camera parameters (pose of camera) */
	void SetCamera(cv::Mat &intrinsic, unsigned &width, unsigned &height, cv::Mat &R, cv::Mat &T);
	/** @brief Update all cameras relative to 'cam'. */
	void UpdateCameraViews(tgCamera cam);

	/**	@brief Sets center of rotation */
	void SetCenterOfRotation(float x, float y, float z);
	
	/** @brief Activates 3D rendering mode; standard after Update() */
	void Activate3D();

	/** @brief Activates 2D rendering moder */
	void Activate2D();
	
	/** @brief Swaps frame buffer to screen (called by Update() aswell)*/
	void Swap();
	
	/** @brief Load a background image to display
	 *  @param image_data	Image pixel data according to 'width', 'height' and 'format'
	 *  @param width,hight	Dimensions of image
	 *  @param format		Format of image data (GL_RGB, GL_BGR, GL_RGBA, GL_LUMINANCE, ... see glTexImage2D in OpenGL spec.)	 */
	void LoadBackgroundImage(unsigned char* image_data, int width, int height, GLenum format=GL_RGB, bool flip=false);

	/** @brief Remove background image. */
	void UnloadBackgroundImage();
	
	/**	@brief Returns the actual position of the camera with respect to the coordinate frame */
	vec3 GetCameraPosition(){ return m_camera.GetPos(); }

	/** @brief get a copy of the current camera of the engine */
	tgCamera GetCamera0(){ return m_camera; }
	
	/** @brief get state of wireframe drawing mode (wireframe mode on/off) */
	bool GetWireframeMode(){ return m_wireframe; }
	
	/** @brief Print a text in 3D world coordinates.
	 *  @param text	Text to print.
	 *  @param pos	3D position of the text.
	 *  @param size	Size of the font.	 */
	void PrintText3D(std::string text, vec3 pos, int size=16);

	/** @brief Print a text at a 2D image coordinates.
	 *  @param text	Text to print.
	 *  @param pos	2D position of the text.
	 *  @param size	Size of the font.	 */
	void PrintText2D(std::string text, vec2 pos, int size=16);
	
	/** @brief Set input speed for mouse control (rotation). */
	void SetInputRotationSpeed(float v){ m_input_rotation_speed = v; }
	/** @brief Set input speed for mouse control (translation). */
	void SetInputTranslationSpeed(float v){ m_input_translation_speed = v; }
	/** @brief Set input speed for mouse control (zoom). */
	void SetInputZoomSpeed(float v){ m_input_zoom_speed = v; }
	/** @brief Set input speed for mouse control (rotation, translation, zoom). */
	void SetSpeeds(float rotation, float translation, float zoom){ 
		m_input_rotation_speed  = rotation;
		m_input_translation_speed = translation;
		m_input_zoom_speed = zoom;
	}
		
	/** @brief Get 3D point of nearest surface at image position (x,y) */
	vec3 Get3DPointFrom2D(int x, int y);

//	bool GetNewPoint(vec3& v);

};

} // namespace TomGine

#endif
