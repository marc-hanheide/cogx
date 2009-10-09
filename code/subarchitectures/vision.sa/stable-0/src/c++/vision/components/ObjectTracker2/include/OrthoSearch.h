
#ifndef __ORTHO_SEARCH_H__
#define __ORTHO_SEARCH_H__

#include <stdio.h>

class OrthoSearch;
#include "Resources.h"
#include "Pose.h"
#include "Shader.h"
#include "Timer.h"
#include "mathlib.h"

class OrthoSearch
{
private:
	Timer m_timer;			// real-time clock
	float m_fTime;			// actual time of current pose finding
	float m_fTimeThreshold;	// time for finding pose
	
	float m_width;
	float m_height;
	
	Shader* m_shadeOrthoSearch;
	Shader* m_shadeOrthoSearchTex;
	Shader* m_shadeOrthoTexEdges;
	Texture* m_texResult;
	Texture* m_texFrame;
	Texture* m_texModel;
	Model* m_model;
	Camera* m_cam_perspective;
	mat4 m_modelviewprojection;
	
	Pose m_p[15];			// perturbed poses with respect to reference pose
	float m_gradient[7];	// gradient to solution
	float m_pf[7];			// perturbing factor
	
	float m_w2rad;			// Conversion of likelihood (w) to radians (rad)
	float m_w2m;			// Conversion of likelihood (w) to meter (m)
	float m_w2z;			// Conversion of likelihood (w) to zoom (z) in meter
	
	// Functions
	void perturb();			// perturbs poses about reference pose
	//void drawPoses(Shader* shader);	// calculates likelihood with respect to p_ref
	void drawPoses(bool useShader);	// calculates likelihood with respect to p_ref
	void getGradient();		// calculates gradient to solution using likelihood information
	void movePose();		// moves pose along gradient towards to solution
	
	bool stopSearching();	// proofes if stop condition reached
	
public:
	OrthoSearch(float width, float height);
	
	// Functions
	Pose findPose(	Pose p_estimate,
					Texture* texFrame,
					Model* model,
					Texture* texModel,
					Camera* cam_perspective);
	
	void set_w2rad(float val){ m_w2rad = val; }
	void set_w2m(float val){ m_w2m = val; }
	void set_time(float val) {m_fTimeThreshold = val; }
	
	void set_modelviewprojection(mat4 mvp) { m_modelviewprojection = mvp; }
	
};

#endif