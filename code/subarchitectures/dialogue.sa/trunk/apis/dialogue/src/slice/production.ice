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
		bool shortNP;
		bool spatialRelation;
		string disabledProperty;
	};

	["java:type:java.util.ArrayList<String>"] sequence<string> stringSeq;

	class ReferenceGenerationResult {
		cast::cdl::WorkingMemoryAddress requestAddress;
		string refEx;
	};

};
};
};
};
};
};

#endif