
#include <cast/architecture/ManagedComponent.hpp>

#ifndef _TRACKING_ENTRY_
#define _TRACKING_ENTRY_

class TrackingEntry{
public:
	bool lock;
	std::string visualObjectID;
	VisionData::VisualObjectPtr obj;
	int id;
	VisionData::GeometryModelPtr model;
	cogx::Math::Pose3 pose;
	
	TrackingEntry(){
		id = -1;
		lock=false;
	}
};

#endif
