
#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include "Tracker.h"

#ifndef _TRACKING_ENTRY_
#define _TRACKING_ENTRY_

class TrackingEntry{
public:
	bool valid;
	bool bfc;
	bool textured;
	Tracker* tracker;
	TrackerModel* model;
	Camera* camera;
	VisionData::VisualObjectPtr obj;
	cast::cdl::WorkingMemoryAddress castWMA;
	Particle constraints;
	Particle detectpose;
	Particle trackpose;
	
	TrackingEntry(){
		valid = false;
		bfc = true;
		textured = false;
		tracker = 0;
		model = 0;
		camera = 0;
	}
	
	void track(){ 
		tracker->track(model, camera,	3, 100,	constraints, trackpose,	0.0);
	}
};

#endif
