
#ifndef __TRACKER_H__
#define __TRACKER_H__

#include <stdio.h>
#include <SDL.h>
#include <GL/gl.h>
#include <highgui.h>

#include "Timer.h"
#include "OpenGLControl.h"
#include "Resources.h"
#include "math.h"
#include "mathlib.h"

class Tracker{
private:
	typedef struct Parameter{
		float width;						// width of viewport (camera image, image processor, opengl, textures) in pixels
		float height;						// height of viewport ( --"-- ) in pixels
		int number_of_particles;			// number of particles to draw for each frame
		float camera_fovy;					// camera field of view in y-direction in degrees
		float camera_initial_position_x;	// camera initial position in x in meters
		float camera_initial_position_y;	// camera initial position in y in meters
		float camera_initial_position_z;	// camera initial position in z in meters
		
		float	noise_rot_min, 				// minimal noise for rotation of particles in degrees
				noise_rot_max;				// maximal noise for rotation of particles in degrees
    	float 	noise_trans_min,			// minimal noise for translation of particles in meters
    			noise_trans_max;			// maximal noise for translation of particles in meters
    	
    	float edge_tolerance;				// maximal angular deviation of edges to match in degrees
    	
    	int viewport_width;					// matching viewport width in pixels
    	int viewport_height;				// matching viewport height in pixels
	} Parameter;
	
	Timer m_timer;
	
	Parameter params;

	ImageProcessor* m_ip;
	Particles* m_particles;
	Texture* m_tex_frame;
	Texture* m_tex_frame_ip;
	Texture* m_tex_frame_thinn;
	Texture* m_tex_model;
	Texture* m_tex_model_ip;
	Model* m_model;
	Shader* m_shadeTextureCompare;
	Shader* m_shadeEdgeCompare;
	Camera* m_cam_ortho;
	Camera* m_cam_perspective;
	OpenGLControl m_opengl;
	
	mat4 m_modelview;
	mat4 m_projection;
	mat4 m_modelviewprojection;
	
	bool m_lockparticles;
	bool m_showparticles;
	bool m_showmodel;
	float time_tracking;
	
	void image_processing_texture(unsigned char* image);
	void image_processing_edge(unsigned char* image);
	void particle_motion_texture(float pow_scale = 1.0, Particle* p_ref=NULL, unsigned int distribution = GAUSS);
	void particle_motion_edge(float pow_scale = 1.0, Particle* p_ref=NULL, unsigned int distribution = GAUSS);
	void model_processing();
	void particle_processing_texture(int num_particles, unsigned int num_avaraged_particles=1);
	void particle_processing_edge(int num_particles, unsigned int num_avaraged_particles=1);
	void draw_result_texture();
	void draw_result_edge();
	
	void getModelViewMatrix(float* mat4x4_modelview);
	bool inputs();
	
public:
	Tracker();
	~Tracker();
	
	bool init(	int width, int height,
				int nop=500,
				float fovy=49.0,
				float cipX=0.0, float cipY=0.165, float cipZ=0.34,
				float n_r_min=0.0, float n_r_max=40.0,
				float n_t_min=0.0, float n_t_max=0.05,
				float et=20.0,
				int vw=256, int vh=256);
				
	bool trackTexture(	unsigned char* image,
						Model* model,
						Particle* p_estimate,
						Particle* p_result);
	
	bool trackEdge(	unsigned char* image,
					Model* model,
					Particle* p_estimate,
					Particle* p_result);
	
	bool render(unsigned char* image);
	bool render(Model* model); 
	bool run();
	bool release();
	
	void showStatistics();

};

#endif