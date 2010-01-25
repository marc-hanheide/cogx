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
#include "CDataFile.h"
#include "ModelEntry.h"

#ifndef PIOVER180
#define PIOVER180 3.14159265358979323846f/180.0
#endif

#define DISTLEN 1000
#define NUM_SPREAD_LOOPS 5

/** @brief namespace Tracking */
namespace Tracking{

typedef std::vector<ModelEntry*> ModelEntryList;

/** @brief Main class of Tracker, defining API */
class Tracker{
protected:
	struct Parameter{
		CameraParameter camPar;
		float width;								// width of viewport (camera image, image processor, opengl, textures) in pixels
		float height;								// height of viewport ( --"-- ) in pixels
		int model_id_count;
		int num_particles;		// number of particles to draw for each frame
		int num_recursions;							// number of recursions for each image
		float edge_tolerance;				// maximal angular deviation of edges to match in degrees
		unsigned int m_spreadlvl;		// Width of edges in pixels (automatically adjusted)
		Particle variation;					// standard deviation of particle distribution in meter
		float minTexGrabAngle;			// Angular threshold between view vector and face normal for grabing texture
		std::string modelPath;
		std::string texturePath;
		std::string shaderPath;
	};
	Parameter params;
	
// 	Particle m_pose;
	
	// Resources
	Texture* m_tex_frame;
	Texture* m_tex_frame_ip[NUM_SPREAD_LOOPS];
	
	ImageProcessor* m_ip;
	
	// ModelEntry
	ModelEntryList m_modellist;
	
	unsigned char* m_image;
	
	Camera m_cam_perspective;
	Camera m_cam_default;
	
	// Flags
	bool m_lock;
	bool m_showparticles;
	int  m_showmodel;
	bool m_draw_edges;
	bool m_tracker_initialized;
	

	// Functions (virtual)
	virtual bool initInternal()=0;
	
	// Functions
	bool initGL();
	
public:
	Tracker();
	~Tracker();
	
	// Main functions (init, image_processing, tracking, reset)
	bool loadINI(const char* inifile);
	bool init(const char* inifile, int width, int height);
	
	virtual void image_processing(unsigned char* image)=0;
	
	virtual bool track()=0;
		
	void reset();
	
	// Model handling
	int 		addModel(Model& m, Pose& p, bool bfc=false);
	void 		removeModel(int id);
	void		getModelPose(int id, Pose& p);
	void		getModelInitialPose(int id, Pose& p);
	
	
	// Drawing to screen (result, ...)
	virtual void drawResult()=0;
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
	void setCamPerspective(Camera camera){ m_cam_perspective = camera; }
	void setSpreadLvl(unsigned int val){ params.m_spreadlvl = val; }
	
	// Get Parameters
	Camera*				getCamPerspective(){ return &m_cam_perspective; }
	unsigned int 	getSpreadLvl(){ return params.m_spreadlvl; }
	
	// get Flags
	bool getLockFlag(){ return m_lock; }
	bool getEdgesImageFlag(){ return m_draw_edges; }
	bool getDrawParticlesFlag(){ return m_showparticles; }
	int  getModelModeFlag(){ return m_showmodel; }
	
	// set Flags
	void setLockFlag(bool val){ m_lock=val; }
	void setEdgesImageFlag(bool val){ m_draw_edges = val; }
	void setDrawParticlesFlag(bool val){ m_showparticles = val; }
	void setModelModeFlag(int val){ m_showmodel = val; }
	
	// Functions for analysing PDF
	virtual std::vector<float> getPDFxy(	Particle pose,
																				float x_min, float y_min,
																				float x_max, float y_max,
																				int res,
																				const char* filename=NULL, const char* filename2=NULL){}
	
	virtual void savePDF(	std::vector<float> vPDFMap,
												float x_min, float y_min,
												float x_max, float y_max,
												int res,
												const char* meshfile, const char* xfile){}
};

} // namespace Tracking

#endif
