#ifndef INTENTIONS_ICE
#define INTENTIONS_ICE

#include <cast/slice/CDL.ice>
#include <beliefs.ice>

module de {
module dfki {
module lt { 
module tr { 
module beliefs {
module slice {
module intentions {

dictionary<string, string> StringContentMap;
dictionary<string, cast::cdl::WorkingMemoryAddress> AddressContentMap;

enum ProcessingState {
	READY,
	INPROGRESS,
	NEEDUPDATE,
	ACHIEVED,
	FAILED
};

class BaseIntention {
	StringContentMap stringContent;
	AddressContentMap addressContent;
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

dictionary<cast::cdl::WorkingMemoryAddress, InterpretedIntention> AddressToIntentionMap;
dictionary<cast::cdl::WorkingMemoryAddress, sitbeliefs::dBelief> AddressToBeliefMap;

class PossibleInterpretedIntentions {
	AddressToIntentionMap intentions;
	AddressToBeliefMap beliefs;
	//if set, this indicates which intention in the intentions map was the correct one
	cast::cdl::WorkingMemoryAddress resolvedIntention;
};

};
};
};

}; 
}; 
}; 
}; 

#endif