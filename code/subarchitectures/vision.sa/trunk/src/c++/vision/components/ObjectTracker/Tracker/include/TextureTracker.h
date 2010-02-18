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


namespace Tracking{

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

	
	// Functions
	void model_processing(ModelEntry* modelEntry);
	
	void particle_filtering(ModelEntry* modelEntry);
		
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
	
	virtual void evaluateParticle(ModelEntry* modelEntry);
	
	virtual void image_processing(unsigned char* image);
	
	virtual bool track();
	
	virtual void textureFromImage();
	
	virtual void untextureModels();
						
	virtual void drawResult();
	
	virtual std::vector<float> getPDFxy(	Particle pose,
																				float x_min, float y_min,
																				float x_max, float y_max,
																				int res,
																				const char* filename=NULL, const char* filename2=NULL);
																
	virtual void savePDF(	std::vector<float> vPDFMap,
												float x_min, float y_min,
												float x_max, float y_max,
												int res,
												const char* meshfile, const char* xfile);
	

};

} // namespace Tracking

#endif