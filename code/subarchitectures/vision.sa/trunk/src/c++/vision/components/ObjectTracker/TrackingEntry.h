
#include <cast/architecture/ManagedComponent.hpp>

#ifndef _TRACKING_ENTRY_
#define _TRACKING_ENTRY_



class TrackingEntry{
public:
	enum Command { ADD, CHANGE, REMOVE, TRACK };
	
	cast::cdl::WorkingMemoryAddress castWMA;
	VisionData::VisualObjectPtr obj;
	int id;
	Command cmd;
	
	TrackingEntry(){
		id = -1;
		cmd = ADD;
	}
};

#endif
