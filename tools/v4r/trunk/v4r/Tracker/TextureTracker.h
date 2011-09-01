 /**
 * @file TextureTracker.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Tracker using TomGine::tgTexture2D for matching.
 * @namespace Tracker
 */
#ifndef _TEXTURE_TRACKER_H_
#define _TEXTURE_TRACKER_H_

#include "Tracker.h"


namespace Tracking{

/** @brief class TextureTracker */
class TextureTracker : public Tracker
{
private:
	
	// Resources
	TomGine::tgShader* m_shadeTexEdgeTest;
	TomGine::tgShader* m_shadeTexColorTest;


	
	// Functions
	void model_processing(ModelEntry* modelEntry);
	
	void particle_filtering(ModelEntry* modelEntry);

	void drawModelEntry(ModelEntry* modelEntry, int mode, float linewidth=1.0f);
		

	TomGine::tgTexture2D m_tex_model;
	std::vector<TomGine::tgTexture2D*> m_tex_model_ip;

public:
	TextureTracker();
	~TextureTracker();
		
	virtual void setKernelSize(int val){
		params.kernel_size = val;
		m_shadeTexEdgeTest->bind();
		m_shadeTexEdgeTest->setUniform("kernelsize", params.kernel_size);
		m_shadeTexEdgeTest->unbind();
		m_shadeTexColorTest->bind();
		m_shadeTexColorTest->setUniform("kernelsize", params.kernel_size);
		m_shadeTexColorTest->unbind();
	}
	
	virtual bool initInternal();
	
	virtual float evaluateParticle(ModelEntry* modelEntry);
	virtual float evaluateParticle(ModelEntry* modelEntry, TomGine::tgShader* shader);
	
	virtual void image_processing(unsigned char* image, GLenum format=GL_BGR);
	virtual void image_processing(unsigned char* image, const TomGine::tgModel &m, const TomGine::tgPose &p, GLenum format=GL_BGR);
	virtual void image_processing(unsigned char* image, int model_id, const TomGine::tgPose &p, GLenum format=GL_BGR);
	
	virtual void model_processing(ModelEntry* modelEntry, TomGine::tgTexture2D &tex_model_ip);

	virtual void tsd_processing(	ModelEntry* modelEntry,
									unsigned segx=5, unsigned segy=5,
									double vis_ratio=0.75);

	virtual bool track();
	bool track(ModelEntry *modelEntry);
	virtual bool track(int id);

	virtual void textureFromImage(bool use_num_pixels=true);
	
	virtual void textureFromImage(int id, const TomGine::tgPose &pose, bool use_num_pixels=true);
	
	virtual void untextureModels();
						
	virtual void drawResult(float linewidth=2.0f);
	
	virtual void drawModelEntry(int id, int mode, float linewidth=1.0f);
	
	virtual void drawModelEdgeImage();

	virtual void evaluatePDF( int id,
														float x_min, float y_min,
														float x_max, float y_max,
														int res,
														const char* meshfile, const char* xfile);

	
	virtual std::vector<float> getPDFxy(	ModelEntry* modelEntry,
																				float x_min, float y_min,
																				float x_max, float y_max,
																				int res);
																
	virtual void savePDF(	std::vector<float> vPDFMap,
												float x_min, float y_min,
												float x_max, float y_max,
												unsigned res,
												const char* meshfile, const char* xfile);
	

};

} // namespace Tracking

#endif
