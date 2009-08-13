
#ifndef __TRACKER_H__
#define __TRACKER_H__

#include <stdio.h>
#include <SDL.h>
#include <GL/gl.h>

#include "Timer.h"
#include "OpenGLControl.h"
#include "Resources.h"
#include "Kalman.h"
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
		
		float	noise_rot_max;				// initial standard deviation for rotational noise of particles in degrees
    	float 	noise_trans_max;			// initial standard deviation for translation noise of particles in meters
    	
    	int cascade_stages;					// number of cascading stages
    	int cascade_mean_max;				// number of most likely particles to average
    	
    	float edge_tolerance;				// maximal angular deviation of edges to match in degrees
    	
    	int viewport_width;					// matching viewport width in pixels
    	int viewport_height;				// matching viewport height in pixels
	} Parameter;
	
	Timer m_timer;
	
	float m_zk[6];
	float m_xk[6];
	Kalman m_kalman;
	Timer m_kalmantimer;
	FILE* pFile;
	
	
	Parameter params;

	ImageProcessor* m_ip;
	Particles* m_particles;
	Texture* m_tex_frame;
	Texture* m_tex_frame_ip;
	Texture* m_tex_frame_thinn;
	Texture* m_tex_frame_spread_1;
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
	
	bool m_lock;
	bool m_showparticles;
	bool m_showmodel;
	bool m_kalman_enabled;
	bool m_cascaded;
	bool m_draw_coordinates;
	float time_tracking;
	
	void image_processing_texture(unsigned char* image);
	void image_processing_edge(unsigned char* image);
	void particle_motion(float pow_scale = 1.0, Particle* p_ref=NULL, unsigned int distribution = GAUSS);
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
				float cipX=0.3, float cipY=0.3, float cipZ=0.3,
				float n_r_max=45.0,
				float n_t_max=0.1,
				int cs = 4, int cmm = 150,
				float et=20.0,
				int vw=128, int vh=128,
				bool kal=true,
				bool dc = false);
				
	bool trackTexture(	unsigned char* image,
						Model* model,
						Particle p_estimate,
						Particle& p_result);
	
	bool trackEdge(	unsigned char* image,
					Model* model,
					Particle p_estimate,
					Particle& p_result);
	
	void renderCoordinates();
	bool render(unsigned char* image);
	bool render(Model* model); 
	bool run();
	bool release();
	
	void showStatistics();

};

#endif