#ifndef MANIPULATION_EXE_ICE
#define MANIPULATION_EXE_ICE

#include <cast/slice/CDL.ice>

module manipulation {
	module execution {
	module slice {
	
		/**
		* Enum for reporting status of manipulation commands. 
		*/
		enum ManipulationTaskStatus {
  		MCSUCCEEDED,
  		MCFAILED,
  		MCREQUESTED
		};
		
		enum ManipulationTaskType {
  		RETRACTARM,
  		POINTOBJ0
		};
		
		sequence<cast::cdl::WorkingMemoryPointer> WMPointerSeq; 

		class ArmMovementTask {
    	// REQUEST:
    	ManipulationTaskType taskType;
    	WMPointerSeq objPointerSeq;

    	// RESPONSE
    	ManipulationTaskStatus status;
  	};
		
	};
	};
};

#endif
