 /**
 * @file TextureTracker.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Tracker using Texture for matching.
 * @namespace Tracker
 */
#ifndef __TEXTURE_TRACKER_H__
#define __TEXTURE_TRACKER_H__

#include "Tracker.h"
#include "tgLighting.h"

/** @brief class TextureTracker */
class TextureTracker : public Tracker
{
private:
	
	// Resources
	Shader* m_shadeTexEdgeTest;
	Shader* m_shadeTexColorTest;
	Shader* m_shadeCompare;
	Texture* m_tex_model;
	Texture* m_tex_model_ip[NUM_SPREAD_LOOPS];
	Texture* m_tex_frame_cmp;
	Texture* m_tex_model_cmp;
	tgLighting m_lighting;
	
	// Functions
	void model_processing();
	
	void particle_filtering(int num_recursions, int num_distribution);
		
public:
	TextureTracker();
	~TextureTracker();
	
	virtual void setKernelSize(int val){
		m_shadeCompare->bind();
		m_shadeCompare->setUniform("kernelsize", val);
		m_shadeCompare->unbind();
	}
	
	virtual void setEdgeShader(){ 
		m_shadeCompare = m_shadeTexEdgeTest;
	}
	virtual void setColorShader(){
		m_shadeCompare = m_shadeTexColorTest;
	}
	
	virtual bool initInternal();
	
	virtual void evaluateParticle(Particle* p);
	
	virtual void image_processing(unsigned char* image);
	
	virtual bool track(	TrackerModel* model,
											Camera* camera,
											int num_recursions,
											int num_distribution,
											Particle p_constraints, 
											Particle& p_result,
											float fTime=0.0);
	
	virtual void textureFromImage();
						
	virtual void drawResult(Particle* p, TrackerModel* m);
	
	virtual vector<float> getPDFxy(	Particle pose,
																	float x_min, float y_min,
																	float x_max, float y_max,
																	int res,
																	const char* filename=NULL, const char* filename2=NULL);
																
	virtual void savePDF(	vector<float> vPDFMap,
												float x_min, float y_min,
												float x_max, float y_max,
												int res,
												const char* meshfile, const char* xfile);
	

};

#endif