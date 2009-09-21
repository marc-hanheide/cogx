#ifndef MOTIVATION_ICE
#define MOTIVATION_ICE

#include <cast/slice/CDL.ice>

module motivation {
    module slice {
    	class TestSource {
    		string name;
    		cast::cdl::CASTTime time;
    	};
    
    
    	enum MotiveStatus {
    		UNSURFACED,
    		SURFACED,
    		POSSIBLE,
    		IMPOSSIBLE,
    		ACTIVE
    	};
    		
    	class Motive {
			cast::cdl::CASTTime created;
			cast::cdl::CASTTime updated;
			cast::cdl::WorkingMemoryAddress referenceEntry;
			cast::cdl::WorkingMemoryAddress thisEntry;
    		MotiveStatus status;
    	};
    	
    	class TestMotive extends Motive {
    		string value;
    		
    	};

    	class ExploreMotive extends Motive {
    		/**
	 		* The ID of the place.
	 		*/
			long placeID;
    		
    	};
    	
    	class PlanProxy {
    	    cast::cdl::WorkingMemoryAddress planAddress;
    	};
    };
};

#endif
