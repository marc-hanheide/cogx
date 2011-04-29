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
// 	void particle_processing(TrackerModel* model, Shader* shadeCompare);
	void particle_filtering(ModelEntry* modelEntry);
	
public:
	EdgeTracker();
	
	virtual bool initInternal();
	
	virtual void image_processing(unsigned char* image);
	
	virtual bool track();
						
	virtual void drawResult();

};

} // namespace Tracking

#endif