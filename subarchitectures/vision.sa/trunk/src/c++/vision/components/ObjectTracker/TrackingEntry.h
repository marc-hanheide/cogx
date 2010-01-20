
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
	int recursions;
	int particles;
	Tracking::Tracker* tracker;
	Tracking::TrackerModel* model;
	Tracking::Camera* camera;
	VisionData::VisualObjectPtr obj;
	cast::cdl::WorkingMemoryAddress castWMA;
	Tracking::Particle constraints;
	Tracking::Particle detectpose;
	Tracking::Particle trackpose;
	
	TrackingEntry(){
		valid = false;
		bfc = true;
		textured = false;
		tracker = 0;
		model = 0;
		camera = 0;
		recursions = 1;
		particles = 20;
	}
	
	void track(){
		tracker->track(model, camera,	recursions, particles,	constraints, trackpose,	0.0);
	}
};

#endif
