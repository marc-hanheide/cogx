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
    		ACTIVE,
    		COMPLETED
    	};
    		
    	class Motive {
			cast::cdl::CASTTime created;
			cast::cdl::CASTTime updated;
			cast::cdl::WorkingMemoryAddress referenceEntry;
			cast::cdl::WorkingMemoryAddress thisEntry;
    		MotiveStatus status;
    		/** a counter for the number of tries this motive has been planned */
    		long tries; 	
    		/** [0-1] encoding for a priority */
    		float priority;
    		/** [0-inf] encoding for costs */
    		float costs;
    		/** [0-1] encoding for information gain */
    		float informationGain;
    		/** rank of activated motives */
    		int rank;
    	};
    	
    	class TestMotive extends Motive {
    		string value;
    		
    	};

    	class HomingMotive extends Motive {
    		long homePlaceID;
    	};

    	class ExploreMotive extends Motive {
    		/**
	 		* The ID of the place.
	 		*/
			long placeID;
    		
    	};
    	
    	class CategorizePlaceMotive extends Motive {
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
