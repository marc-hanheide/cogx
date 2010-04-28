
#ifndef __EDGE_TRACKER_H__
#define __EDGE_TRACKER_H__

#include "Tracker.h"

class EdgeTracker : public Tracker
{
private:
	
	// Functions
	virtual void image_processing(unsigned char* image);
	
public:
	EdgeTracker();
	
	virtual bool initInternal();
				
	virtual bool track(	unsigned char* image,
						Model* model,
						Camera* camera,
						Pose p_estimate,
						Pose& p_result,
						float time);
						
	virtual void drawResult(Pose* p);

};

#endif
