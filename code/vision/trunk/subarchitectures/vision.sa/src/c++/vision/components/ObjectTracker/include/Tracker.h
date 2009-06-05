
#ifndef __TRACKER_H__
#define __TRACKER_H__

#include <stdio.h>
#include <math.h>
#include <SDL.h>
#include <GL/gl.h>

#include "Timer.h"
#include "OpenGLControl.h"
#include "Resources.h"
#include "Kalman.h"
#include "mathlib.h"

class Tracker{
private:
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
	Shader* m_shadeTextureCompare;
	Shader* m_shadeEdgeCompare;
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
	bool m_cascaded;
	bool m_draw_coordinates;
	bool m_draw_edges;
	bool m_zero_particles;
	bool m_result_textured;
	
	bool m_tracker_initialized;
	
	// Functions (texture tracking)
	void image_processing_texture(unsigned char* image);
	void model_processing();
	void particle_processing_texture(int num_particles, unsigned int num_avaraged_particles=1);
	
	// Functions (edge tracking)
	void image_processing_edge(unsigned char* image);
	void particle_processing_edge(int num_particles, unsigned int num_avaraged_particles=1);
	
	// Functions (both)
	void kalman_filtering(Particle* pm);
	void particle_motion(float pow_scale = 1.0, Particle* p_ref=NULL, unsigned int distribution = GAUSS);
	void draw_result_texture(Particle* p);
	void draw_result_edge();
	
	bool inputs();
	
public:
	Tracker();
	~Tracker();
	
	bool init(	int width, int height,								// image size in pixels
				int nop=3000,										// maximum number of particles
				float n_r_max=45.0,									// standard deviation of rotational noise in degree
				float n_t_max=0.1,									// standard deviation of translational noise in meter
				float et=20.0,										// edge matching tolerance in degree
				float tt=0.05,										// goal tracking time in seconds
				bool kal=true,										// kalman filtering enabled
				bool coord=false,									// draw coordinate frame at inertial 0-position
				bool lock=false);									// locked particles (press 'l' to unlock)
				
	bool trackTexture(	unsigned char* image,
						Model* model,
						Camera* camera,
						Particle p_estimate,
						Particle& p_result);
	
	bool trackEdge(	unsigned char* image,
					Model* model,
					Camera* camera,
					Particle p_estimate,
					Particle& p_result);
	
	void renderCoordinates();
	void showStatistics();
	bool release();	

};

#endif