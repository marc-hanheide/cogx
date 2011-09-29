#ifndef INTENTIONS_ICE
#define INTENTIONS_ICE

#include <cast/slice/CDL.ice>

module de {
module dfki {
module lt { 
module tr { 
module beliefs {
module slice {
module intentions {

dictionary<string, string> StringContentMap;
dictionary<string, cast::cdl::WorkingMemoryPointer> PointerContentMap;

enum ProcessingState {
	READY,
	INPROGRESS,
	NEEDUPDATE,
	ACHIEVED,
	FAILED
};

class BaseIntention {
	StringContentMap stringContent;
	PointerContentMap pointerContent;
};

class InterpretedIntention extends BaseIntention {
	ProcessingState state;
//	WorkingMemoryAddress origin;
	string agent;
	float confidence;
};

class IntentionToAct extends BaseIntention {
//	WorkingMemoryAddress origin;
};

};
};
};

}; 
}; 
}; 
}; 

#endif