#ifndef MOTIVATION_ICE
#define MOTIVATION_ICE

#include <cast/slice/CDL.ice>

module motivation {
    module slice {
    	enum MotiveStatus {
    		UNSURFACED,
    		POSSIBLE,
    		IMPOSSIBLE,
    		ACTIVE
    	};
    		
    	class Motive {
    		MotiveStatus status;
    		string goal;
    	};
    };
};

#endif
