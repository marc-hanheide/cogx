
#include <cast/architecture/ManagedComponent.hpp>

#ifndef _TRACKING_ENTRY_
#define _TRACKING_ENTRY_

class TrackingEntry{
public:
	bool lock;
	std::string visualObjectID;
	VisionData::VisualObjectPtr obj;
	int id;
	
	TrackingEntry(){
		id = -1;
		lock=false;
	}
};

#endif
