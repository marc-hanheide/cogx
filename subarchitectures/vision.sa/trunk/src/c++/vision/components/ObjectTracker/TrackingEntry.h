
#include <cast/architecture/ManagedComponent.hpp>

#ifndef _TRACKING_ENTRY_
#define _TRACKING_ENTRY_



class TrackingEntry{
public:
	enum Command { TRACK, ADD, REMOVE, LOCK, UNLOCK, GETPOINT3D  };
	
	std::string visualObjectID;
	std::string trackingCommandID;
	VisionData::VisualObjectPtr obj;
	VisionData::TrackingCommandPtr track_cmd;
	int id;
	Command cmd;
	
	TrackingEntry(){
		id = -1;
		cmd = ADD;
	}
};

#endif
