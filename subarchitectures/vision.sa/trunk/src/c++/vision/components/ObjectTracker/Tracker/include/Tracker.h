 /**
 * @file Tracker.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Main file of Tracker.
 * @namespace Tracking
 */
#ifndef _TRACKER_H_
#define _TRACKER_H_

#include "headers.h"
#include "Timer.h"
#include "Resources.h"
#include "Distribution.h"
#include "Predictor.h"
#include "mathlib.h"
#include "CDataFile.h"
#include "ModelEntry.h"
#include "tgLighting.h"

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
		int num_particles;					// number of particles to draw for each frame
		int num_recursions;					// number of recursions for each image
		int convergence;						// convergence factor
		float edge_tolerance;				// maximal angular deviation of edges to match in degrees
		unsigned int m_spreadlvl;		// Width of edges in pixels (automatically adjusted)
		Particle variation;					// standard deviation of particle distribution in meter
		float minTexGrabAngle;			// Angular threshold between view vector and face normal for grabing texture
		std::string modelPath;
		std::string texturePath;
		std::string shaderPath;
	};
	Parameter params;
	
	// Resources
	Texture* m_tex_frame;
	Texture* m_tex_frame_ip[NUM_SPREAD_LOOPS];
	
	tgLighting m_lighting;
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
	/** @brief Load parameter of tracker with an INI file */
	bool loadINI(const char* inifile);
	
	virtual bool initInternal()=0;
	
	bool initGL();
	
public:
	Tracker();
	~Tracker();
	
	// Main functions (init, image_processing, tracking, reset)
	/** @brief Initialize tracker with an INI file and image/window width and height in pixel */
	bool init(const char* inifile, int width, int height);
	
	/** @brief Perform image processing with edge detection */
	virtual void image_processing(unsigned char* image)=0;
	
	/** @brief Tracks all models by matching their edges against edges of images */
	virtual bool track()=0;
	
	/** @brief Resets the pose of the models to the initial pose */
	void reset();
	
	// Model handling
	/** @brief Adds a geometrical model to the tracker
	*		@return id of the added model (-1 if not successfull)
	*/
	int 		addModel(Model& m, Pose& p, bool bfc=false);
	
	/** @brief Remove model from tracker */
	void 		removeModel(int id);
	
	/** @brief Get current pose of a model */
	void		getModelPose(int id, Pose& p);
	
	/** @brief Get the initial pose of a model */
	void		getModelInitialPose(int id, Pose& p);
	
	/** @brief Get Confidence value of a model at current pose */
	void		getModelConfidence(int id, int& c);
	
	/** @brief Get 3D point from 2D window coordinates */
	bool		getModelPoint3D(int id, int x_win, int y_win, float& x3, float& y3, float& z3);
	
	/** @brief Set the initial pose of a model */
	void		setModelInitialPose(int id, Pose& p);
	
	/** @brief Set a model predictor */
	void		setModelPredictor(int id, Predictor* predictor);
	
	/** @brief Locks the model with id */
	void		setModelLock(int id, bool lock);
	
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
	void setFrameTime(double dTime);
	bool setCameraParameters(CameraParameter cam_par);
	void setSpreadLvl(unsigned int val){ params.m_spreadlvl = val; }
	
	// Get Parameters
	float getCamZNear(){ return m_cam_perspective.GetZNear(); }
	float getCamZFar(){ return m_cam_perspective.GetZFar(); }
	unsigned int 	getSpreadLvl(){ return params.m_spreadlvl; }
	
	// get Flags
	bool getLockFlag(){ return m_lock; }
	bool getEdgesImageFlag(){ return m_draw_edges; }
	bool getDrawParticlesFlag(){ return m_showparticles; }
	int  getModelModeFlag(){ return m_showmodel; }
	
	// set Flags
	void setLockFlag(bool val);
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
