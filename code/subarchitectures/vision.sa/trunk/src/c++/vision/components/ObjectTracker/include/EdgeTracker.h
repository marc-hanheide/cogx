
#ifndef __EDGE_TRACKER_H__
#define __EDGE_TRACKER_H__

#include "Tracker.h"

class EdgeTracker : public Tracker
{
private:
	
	// Resources
	Shader* m_shadeEdgeCompare;

	// Functions
	virtual void image_processing(unsigned char* image);
	virtual void particle_processing(int num_particles, unsigned int num_avaraged_particles=1);
	virtual void particle_motion(float pow_scale = 1.0, Particle* p_ref=NULL, unsigned int distribution = GAUSS);
	void particle_filtering(Particle* p_estimate);
	
public:
	EdgeTracker();
	
	virtual bool initInternal();
				
	virtual bool track(	unsigned char* image,
						Model* model,
						Camera* camera,
						Particle p_estimate,
						Particle& p_result);
						
	virtual void drawResult(Particle* p);

};

#endif