
#ifndef __TEXTURE_TRACKER_H__
#define __TEXTURE_TRACKER_H__

#include "Tracker.h"
#include "Lighting.h"

class TextureTracker : public Tracker
{
private:
	
	// Resources
	Shader* m_shadeTextureCompare;
	Shader* m_shadeTexturing;
	Lighting m_lighting;
	
	// Functions
	void model_processing();
	virtual void image_processing(unsigned char* image);
	virtual void particle_processing(int num_particles, unsigned int num_avaraged_particles=1);
	virtual void particle_motion(float pow_scale = 1.0, Particle* p_ref=NULL, unsigned int distribution = GAUSS);
	void particle_filtering(Particle* p_estimate);
		
public:
	TextureTracker();
	
	virtual bool initInternal();
				
	virtual bool track(	unsigned char* image,
						Model* model,
						Camera* camera,
						Particle p_estimate,
						Particle& p_result);
						
	virtual void textureFromImage();
						
	virtual void drawResult(Particle* p);
	

};

#endif