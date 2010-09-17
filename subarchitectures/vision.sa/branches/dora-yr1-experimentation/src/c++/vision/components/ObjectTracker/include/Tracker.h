
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

#ifndef PIOVER180
#define PIOVER180 3.14159265358979323846f/180.0
#endif

#define DISTLEN 1000

class Tracker{
protected:
	typedef struct Parameter{
		float width;						// width of viewport (camera image, image processor, opengl, textures) in pixels
		float height;						// height of viewport ( --"-- ) in pixels
		int number_of_particles;			// number of particles to draw for each frame
		
		float	noise_rot_max;				// initial standard deviation for rotational noise of particles in degrees
    	float 	noise_trans_max;			// initial standard deviation for translation noise of particles in meters
    	
    	float edge_tolerance;				// maximal angular deviation of edges to match in degrees
    	
    	float track_time;					// time for one tracking pass (the less time given, the less particle will be used)
    	
    	Particle zP;						// zero Particle to which the tracker is reseted when pressing the zero_particles key
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
	unsigned char* m_image;
	Particles* m_particles;
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
	bool m_showparticles;
	bool m_showmodel;
	bool m_kalman_enabled;
	bool m_zero_particles;
	bool m_draw_edges;
	bool m_tracker_initialized;
	bool m_testflag;
	bool m_bfc;

	
	// Functions (virtual)
	virtual void image_processing(unsigned char* image)=0;
	virtual void particle_motion(float pow_scale = 1.0, Particle* p_ref=NULL, unsigned int distribution = GAUSS)=0;
	virtual void particle_processing(int num_particles, unsigned int num_avaraged_particles=1)=0;
	virtual bool initInternal()=0;
	
	
	// Functions
	bool isReady(unsigned char* image, Model* model, Camera* camera, Particle* p_estimate);
	void kalman_filtering(Particle* pm);
	
public:
	Tracker();
		
	bool init(	int width, int height,								// image size in pixels
				int nop=3000,										// maximum number of particles
				float n_r_max=45.0,									// standard deviation of rotational noise in degree
				float n_t_max=0.1,									// standard deviation of translational noise in meter
				float et=20.0,										// edge matching tolerance in degree
				float tt=0.05,										// goal tracking time in seconds
				Particle zp=Particle(0));
	
	virtual bool track(	unsigned char* image,
						Model* model,
						Camera* camera,
						Particle p_estimate,
						Particle& p_result)=0;
						
	virtual void textureFromImage(){}
	
	virtual void drawResult(Particle* p)=0;
	void drawCoordinates();
	void drawImage(unsigned char* image);
	void drawPixel(float u, float v, vec3 color=vec3(1.0,1.0,1.0), float size=1.0);
	void drawTest();
	void swap();
	
	void setCamPerspective(Camera* camera){ m_cam_perspective = camera; }
	void setTrackTime(float time){ params.track_time = time; }
	void setNoise(float rot, float trans){ params.noise_rot_max=rot; params.noise_trans_max=trans; }
	void setTestflag(bool val){ m_testflag = val; }
	void setBFC(bool val){ m_bfc=val; }
	void setZeroPose(Particle zp){ params.zP = zp; }
	
	void lock(bool val){ m_lock=val; m_particles->setAll(*m_particles->getMax()); }
	
	bool getLock(){ return m_lock; }
	bool getEdgesImage(){ return m_draw_edges; }
	bool getEdgesModel(){ return m_showmodel; }
	bool getParticlesVisible(){ return m_showparticles; }
	bool getKalmanEnabled(){ return m_kalman_enabled; }
	
	void showEdgesImage(bool val){ m_draw_edges = val; }
	void showEdgesModel(bool val){ m_showmodel = val; }
	void showParticles(bool val){ m_showparticles = val; }
	void showStatistics();
	
	void zeroParticles();
	void enableKalman(bool val);
	void disableKalman(){ m_kalman_enabled = false; }	

};

#endif
