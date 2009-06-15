
#ifndef __TRACKER_H__
#define __TRACKER_H__

#include <stdio.h>
#include <math.h>
#include <SDL/SDL.h>
#include <GL/gl.h>

#include "Timer.h"
#include "OpenGLControl.h"
#include "Resources.h"
#include "Kalman.h"
#include "mathlib.h"

class Tracker{
protected:
	typedef struct Parameter{
		float width;						// width of viewport (camera image, image processor, opengl, textures) in pixels
		float height;						// height of viewport ( --"-- ) in pixels
		int number_of_particles;			// number of particles to draw for each frame
		
		float	noise_rot_max;				// initial standard deviation for rotational noise of particles in degrees
    	float 	noise_trans_max;			// initial standard deviation for translation noise of particles in meters
    	
    	float edge_tolerance;				// maximal angular deviation of edges to match in degrees
    	
    	int viewport_width;					// matching viewport width in pixels
    	int viewport_height;				// matching viewport height in pixels
    	
    	float track_time;					// time for one tracking pass (the less time given, the less particle will be used)
	} Parameter;
	
	Parameter params;
	
	Timer m_timer;
	float time_tracking;
	
	// Kalman
	float m_zk[6];
	float m_xk[6];
	Kalman m_kalman;
	Timer m_kalmantimer;
	float m_kalman_gain[4];
	
	// Resources
	ImageProcessor* m_ip;
	Particles* m_particles;
	Texture* m_tex_frame;
	Texture* m_tex_frame_ip[4];
	Texture* m_tex_model;
	Texture* m_tex_model_ip;
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
	bool m_showparticles;
	bool m_showmodel;
	bool m_kalman_enabled;
	bool m_zero_particles;
	bool m_draw_edges;
	bool m_tracker_initialized;

	
	// Functions (virtual)
	virtual void image_processing(unsigned char* image)=0;
	virtual void particle_motion(float pow_scale = 1.0, Particle* p_ref=NULL, unsigned int distribution = GAUSS)=0;
	virtual void particle_processing(int num_particles, unsigned int num_avaraged_particles=1)=0;
	
	
	// Functions
	void kalman_filtering(Particle* pm);
	bool inputs();
	
public:
	Tracker();
	
	inline void lock(){ m_lock=true; }
	inline void unlock(){ m_lock=false; }
	
	bool init(	int width, int height,								// image size in pixels
				int nop=3000,										// maximum number of particles
				float n_r_max=45.0,									// standard deviation of rotational noise in degree
				float n_t_max=0.1,									// standard deviation of translational noise in meter
				float et=20.0,										// edge matching tolerance in degree
				float tt=0.05,										// goal tracking time in seconds
				bool kal=true,										// kalman filtering enabled
				bool lock=false);									// locked particles (press 'l' to unlock)
	
	virtual bool initInternal()=0;
	
	virtual bool track(	unsigned char* image,
						Model* model,
						Camera* camera,
						Particle p_estimate,
						Particle& p_result)=0;
	
	virtual void drawResult(Particle* p)=0;
	
	void renderCoordinates();
	void drawPixel(int u, int v, vec3 color=vec3(1.0,1.0,1.0), float size=1.0);
	void showStatistics();
		
	void swap();

};

#endif