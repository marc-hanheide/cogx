
#ifndef __TRACKER_H__
#define __TRACKER_H__

#include "headers.h"
#include "Timer.h"
#include "Resources.h"
#include "mathlib.h"

#ifndef PIOVER180
#define PIOVER180 3.14159265358979323846f/180.0
#endif

#define DISTLEN 1000
#define NUM_SPREAD_LOOPS 5

class Tracker{
protected:
	typedef struct Parameter{
		float width;						// width of viewport (camera image, image processor, opengl, textures) in pixels
		float height;						// height of viewport ( --"-- ) in pixels
		int number_of_particles;			// number of particles to draw for each frame
		int recursions;
		float edge_tolerance;				// maximal angular deviation of edges to match in degrees
		float track_time;					// time for one tracking pass (the less time given, the less particle will be used)
		Particle zP;						// zero Particle to which the tracker is reseted when pressing the zero_particles key
	} Parameter;
	Parameter params;
	
	int m_tracker_id;
	
	// Timer
	Timer m_timer;
	float time_tracking;
	
	// Resources
	Particle m_pose;
	Particle m_pConstraints;
	ImageProcessor* m_ip;
	unsigned char* m_image;
	Particles* m_particles;
	Texture* m_tex_frame;
	Texture* m_tex_frame_ip[NUM_SPREAD_LOOPS];
	Model* m_model;
	Camera* m_cam_ortho;
	Camera* m_cam_default;
	Camera* m_cam_perspective;
	unsigned int m_spreadlvl;
	
	// Matrices
	mat4 m_modelview;
	mat4 m_projection;
	mat4 m_modelviewprojection;
	
	// Flags
	bool m_lock;
	bool m_showparticles;
	int  m_showmodel;
	bool m_zero_particles;
	bool m_draw_edges;
	bool m_tracker_initialized;
	bool m_bfc;

	// Functions (virtual)
	virtual bool initInternal()=0;
	virtual void evaluateParticle(Particle* p){}
	
	// Functions
	bool isReady(unsigned char* image, Model* model, Camera* camera);
	bool initGL();
	
public:
	Tracker();
	~Tracker();
	
	// Main functions (init, image_processing, tracking, reset)
	bool init(	int width, int height,					// image size in pixels
							int nop=3000,										// maximum number of particles
							int rec=2,											// recursions per image
							float et=20.0,									// edge matching tolerance in degree
							float tt=0.05,									// goal tracking time in seconds
							Particle zp=Particle(0.0));			// initial particle
	
	virtual void image_processing(unsigned char* image)=0;
	
	virtual bool track(	Model* model,
											Camera* camera,
											int num_recursions,
											int num_particles,
											Particle p_constraints, 
											Particle& p_result,
											float fTime)=0;
						
	virtual bool track(	unsigned char* image,
											Model* model,
											Camera* camera,
											Particle& p_result)=0;
	
	void reset();
	
	// Drawing to screen (result, ...)
	virtual void drawResult(Particle* p, Model* m)=0;
	void drawCoordinates();
	void drawImage(unsigned char* image);
	void drawPixel(float u, float v, vec3 color=vec3(1.0,1.0,1.0), float size=1.0);
	void drawTest();
	void drawSpeedBar(float h);
	void swap();
	void printStatistics();
	
	// set parameters for texture tracking
  virtual void setKernelSize(int val){ }
	virtual void setEdgeShader(){ }
	virtual void setColorShader(){ }
  virtual void textureFromImage(){}
  	
	// Set Parameters
	void setCamPerspective(Camera* camera){ m_cam_perspective = camera; }
	void setTrackTime(float time){ params.track_time = time; }
	void setBFC(bool val){ m_bfc=val; }
	void setSpreadLvl(unsigned int val){ m_spreadlvl = val; }
	void setInitialPose(Particle zp){ params.zP = zp; m_particles->setAll(zp); }
	
	// Get Parameters
	Camera*				getCamPerspective(){ return m_cam_perspective; }
	float					getTrackTime(){ return params.track_time; }
	bool 					getBFC(){ return m_bfc; }
	unsigned int 	getSpreadLvl(){ return m_spreadlvl; }
	Particle			getInitialPose(){ return params.zP; }
	Particle 			getLastPose(){ return m_pose; }
	
	// get Flags
	bool getLockFlag(){ return m_lock; }
	bool getEdgesImageFlag(){ return m_draw_edges; }
	int  getModelModeFlag(){ return m_showmodel; }
	bool getDrawParticlesFlag(){ return m_showparticles; }

	// set Flags
	void setLockFlag(bool val){ m_lock=val; }
	void setEdgesImageFlag(bool val){ m_draw_edges = val; }
	void setModelModeFlag(int val){ m_showmodel = val; }
	void setDrawParticlesFlag(bool val){ m_showparticles = val; }
	
	// Functions for analysing PDF
	virtual vector<float> getPDFxy(	Particle pose,
																	float x_min, float y_min,
																	float x_max, float y_max,
																	int res,
																	const char* filename=NULL, const char* filename2=NULL){}
	
	virtual void savePDF(	vector<float> vPDFMap,
												float x_min, float y_min,
												float x_max, float y_max,
												int res,
												const char* meshfile, const char* xfile){}
};

#endif
