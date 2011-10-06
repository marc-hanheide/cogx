#ifndef PRODUCTION_ICE
#define PRODUCTION_ICE

#include <cast/slice/CDL.ice>

module de {
module dfki {
module lt { 
module tr { 
module dialogue { 
module production { 

	class ReferenceGenerationRequest {
		cast::cdl::WorkingMemoryAddress obj;
	};

	["java:type:java.util.ArrayList<String>"] sequence<string> stringSeq;

	class ReferenceGenerationResult {
		cast::cdl::WorkingMemoryAddress requestAddress;
		string category;
		string relation;
		string location;
	};

};
};
};
};
};
};

#endif