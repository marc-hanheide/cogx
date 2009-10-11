
#ifndef __TRACKER_H__
#define __TRACKER_H__

#include <stdio.h>
#include <math.h>
#include <SDL/SDL.h>
#include <GL/gl.h>

#include "Timer.h"
#include "OpenGLControl.h"
#include "Resources.h"
#include "mathlib.h"

#ifndef PIOVER180
#define PIOVER180 3.14159265358979323846f/180.0
#endif

#define DISTLEN 1000

class Tracker{
protected:
	typedef struct Parameter{
		float width;						// width of viewport (camera image, image processor, opengl, textures) in pixels
		float height;						// height of viewport ( --"-- ) in pixels
		float track_time;					// time for one tracking pass (the less time given, the less particle will be used)
    	Pose zP;						// zero Pose to which the tracker is reseted when pressing the zero_particles key
	} Parameter;
	
	Parameter params;
	
	Timer m_timer;
	float time_tracking;
	unsigned int m_iterations;
	
	// Resources
	ImageProcessor* m_ip;
	unsigned char* m_image;
	OrthoSearch* m_ortho_search;
	Texture* m_tex_frame;
	Texture* m_tex_frame_ip[4];
	Texture* m_tex_model;
	Texture* m_tex_model_ip[4];
	Model* m_model;
	Camera* m_cam_ortho;
	Camera* m_cam_default;
	Camera* m_cam_perspective;
	OpenGLControl m_opengl;
	
	// Matrices
	mat4 m_modelview;
	mat4 m_projection;
	mat4 m_modelviewprojection;
	
	// Controls
	bool m_lock;
	bool m_showmodel;
	bool m_zero_pose;
	bool m_draw_edges;
	bool m_tracker_initialized;
	bool m_testflag;
	bool m_bfc;

	
	// Functions (virtual)
	virtual void image_processing(unsigned char* image)=0;
	virtual bool initInternal()=0;
	
	
	// Functions
	bool isReady(unsigned char* image, Model* model, Camera* camera, Pose* p_estimate);
	void kalman_filtering(Pose* pm);
	
public:
	Tracker();
		
	bool init(	int width, int height,								// image size in pixels
				float tt=0.05, Pose zp = Pose(0.0));										// goal tracking time in seconds
	
	virtual bool track(	unsigned char* image,
						Model* model,
						Camera* camera,
						Pose p_estimate,
						Pose& p_result,
						float time)=0;
						
	virtual void textureFromImage(){}
	
	virtual void drawResult(Pose* p)=0;
	void drawCoordinates();
	void drawImage(unsigned char* image);
	void drawPixel(float u, float v, vec3 color=vec3(1.0,1.0,1.0), float size=1.0);
	void drawTest();
	void swap();
	
	void setCamPerspective(Camera* camera){ m_cam_perspective = camera; }
	void setTrackTime(float time){ params.track_time = time; }
	void setTestflag(bool val){ m_testflag = val; }
	void setBFC(bool val){ m_bfc=val; }
	
	void lock(bool val){ m_lock=val; }
	
	bool getLock(){ return m_lock; }
	bool getEdgesImage(){ return m_draw_edges; }
	bool getEdgesModel(){ return m_showmodel; }
	unsigned int getIterations(){ return m_iterations; }
	
	void showEdgesImage(bool val){ m_draw_edges = val; }
	void showEdgesModel(bool val){ m_showmodel = val; }
	void showStatistics();
	
	void zeroPose();
};

#endif
