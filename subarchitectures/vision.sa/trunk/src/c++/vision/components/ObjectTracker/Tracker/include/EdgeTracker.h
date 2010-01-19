 /**
 * @file EdgeTracker.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Tracker using geometry edges for matching.
 * @namespace Tracker
 */
#ifndef EDGE_TRACKER_H
#define EDGE_TRACKER_H

#include "Tracker.h"

namespace Tracking{

/** @brief class EdgeTracker */
class EdgeTracker : public Tracker
{
private:
	
	// Resources
	Shader* m_shadeEdgeCompare;

	// Functions
	void particle_processing(TrackerModel* model, Shader* shadeCompare);
	void particle_filtering(Particle& p_result, Particle p_constraints, int recursions=1, float fTime=0.0);
	
public:
	EdgeTracker();
	
	virtual bool initInternal();
	
	virtual void image_processing(unsigned char* image);
	virtual bool track(	TrackerModel* model,
											Camera* camera,
											int num_recursions,
											int num_distribution,
											Particle p_constraints, 
											Particle& p_result,
											float fTime=0.0);
						
	virtual void drawResult(Particle* p, TrackerModel* m);

};

} // namespace Tracking

#endif