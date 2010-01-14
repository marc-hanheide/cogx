
#ifndef __EDGE_TRACKER_H__
#define __EDGE_TRACKER_H__

#include "Tracker.h"

class EdgeTracker : public Tracker
{
private:
	
	// Resources
	Shader* m_shadeEdgeCompare;

	// Functions
	void particle_processing();
	void particle_filtering(Particle& p_result, Particle p_constraints, int recursions=1, float fTime=0.0);
	
public:
	EdgeTracker();
	
	virtual bool initInternal();
	
	virtual void image_processing(unsigned char* image);
	virtual bool track(	TrackerModel* model,
											Camera* camera,
											int num_recursions,
											int num_particles,
											Particle p_constraints, 
											Particle& p_result,
											float fTime=0.0);
						
	virtual void drawResult(Particle* p, TrackerModel* m);

};

#endif