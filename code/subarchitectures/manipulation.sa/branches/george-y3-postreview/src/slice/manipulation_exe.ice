#ifndef MANIPULATION_EXE_ICE
#define MANIPULATION_EXE_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>

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
  		POINTOBJ0,
  		IDLE
		};
		
		sequence<cast::cdl::WorkingMemoryPointer> WMPointerSeq; 

		class ArmMovementTask {
    	// REQUEST:
    	ManipulationTaskType taskType;
    	WMPointerSeq objPointerSeq;

    	// RESPONSE
    	ManipulationTaskStatus status;
  	};
  	
  	class ArmStatus {
  		cogx::Math::Pose3 currentPose;
  		cogx::Math::Pose3 targetPose;
    
    	ManipulationTaskType currentTask;
    	ManipulationTaskType lastTask;
    	ManipulationTaskStatus lastStatus;
  	};
		
	};
	};
};

#endif
