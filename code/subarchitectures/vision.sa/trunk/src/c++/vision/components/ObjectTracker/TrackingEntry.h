
#include <cast/architecture/ManagedComponent.hpp>

#ifndef _TRACKING_ENTRY_
#define _TRACKING_ENTRY_



class TrackingEntry{
public:
	enum Command { TRACK, ADD, REMOVE, LOCK, UNLOCK, GETPOINT3D  };
	
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
