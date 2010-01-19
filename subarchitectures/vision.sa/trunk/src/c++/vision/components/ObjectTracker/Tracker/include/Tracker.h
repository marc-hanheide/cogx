 /**
 * @file Tracker.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Main file of Tracker.
 * @namespace Tracker
 */
#ifndef __TRACKER_H__
#define __TRACKER_H__

#include "headers.h"
#include "Timer.h"
#include "Resources.h"
#include "Distribution.h"
#include "Predictor.h"
#include "mathlib.h"

#ifndef PIOVER180
#define PIOVER180 3.14159265358979323846f/180.0
#endif

#define DISTLEN 1000
#define NUM_SPREAD_LOOPS 5

/** @brief class Tracker*/
class Tracker{
protected:
	typedef struct Parameter{
		int m_tracker_id;
		float width;								// width of viewport (camera image, image processor, opengl, textures) in pixels
		float height;								// height of viewport ( --"-- ) in pixels
		int number_of_particles;		// number of particles to draw for each frame
		int recursions;							// number of recursions for each image
		float edge_tolerance;				// maximal angular deviation of edges to match in degrees
		unsigned int m_spreadlvl;		// Width of edges in pixels (automatically adjusted)
		Particle zP;								// zero Particle to which the tracker is reseted when pressing the zero_particles key
		float minTexGrabAngle;			// Angular threshold between view vector and face normal for grabing texture
		float time_tracking;
	} Parameter;
	Parameter params;
	
	Timer m_timer;
	Particle m_pose;
	Particle m_pConstraints;
	
	// Resources
	Texture* m_tex_frame;
	Texture* m_tex_frame_ip[NUM_SPREAD_LOOPS];
	
	ImageProcessor* m_ip;
	Distribution* m_distribution;
	Predictor m_predictor;
	
	unsigned char* m_image;
	TrackerModel* m_model;
	Camera* m_cam_perspective;
	Camera m_cam_default;
	
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
	bool isReady(unsigned char* image, TrackerModel* model, Camera* camera);
	bool initGL();
	
public:
	Tracker();
	~Tracker();
	
	// Main functions (init, image_processing, tracking, reset)
	bool init(	int width, int height,					// image size in pixels
							float et=45.0,									// edge matching tolerance in degree
							float mtga=3.0*PI/4.0,					// minTextureGrabAngle
							Particle zp=Particle(0.0),			// initial particle
							Particle ic=Particle(0.1));
	
	virtual void image_processing(unsigned char* image)=0;
	
	virtual bool track(	TrackerModel* model,
											Camera* camera,
											int num_recursions,
											int num_distribution,
											Particle p_constraints, 
											Particle& p_result,
											float fTime)=0;
		
	void reset();
	
	// Drawing to screen (result, ...)
	virtual void drawResult(Particle* p, TrackerModel* m)=0;
	void drawCoordinates();
	void drawImage(unsigned char* image);
	void drawPixel(float u, float v, vec3 color=vec3(1.0,1.0,1.0), float size=1.0);
	void drawCalibrationPattern();
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
	void setBFC(bool val){ m_bfc=val; }
	void setSpreadLvl(unsigned int val){ params.m_spreadlvl = val; }
	void setInitialPose(Particle zp){ params.zP = zp; }
	void setEdgeTolerance(float et){ params.edge_tolerance = et; }
	
	// Get Parameters
	Camera*				getCamPerspective(){ return m_cam_perspective; }
	bool 					getBFC(){ return m_bfc; }
	unsigned int 	getSpreadLvl(){ return params.m_spreadlvl; }
	Particle			getInitialPose(){ return params.zP; }
	Particle 			getLastPose(){ return m_pose; }
	float					getEdgeTolerance(){ return params.edge_tolerance; }
	int						getTrackerID(){ return params.m_tracker_id; }
	
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
