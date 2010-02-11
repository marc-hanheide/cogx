
#include <cast/architecture/ManagedComponent.hpp>

#ifndef _TRACKING_ENTRY_
#define _TRACKING_ENTRY_

class TrackingEntry{
public:
	std::string visualObjectID;
	VisionData::VisualObjectPtr obj;
	int id;
	
	TrackingEntry(){
		id = -1;
	}
};

#endif
