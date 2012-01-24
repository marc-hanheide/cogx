#ifndef COGX_GEORGE_ICE
#define COGX_GEORGE_ICE

#include <cast/slice/CDL.ice>
#include <Execution.ice>
#include <motivation.ice>


module execution {
  module slice {

	/*
	* Class used to store information about robot state. All accesses must be surrounded by lockEntry/unlockEntry as it is updated by multiple components.
	*/
	class Robot {
			cast::cdl::WorkingMemoryPointer currentViewCone;
			bool armIsResting;
			bool excludeColor;
			bool excludeShape;
	};

    module actions {
		module george {
			module yr3 {


      class MoveToViewCone extends execution::slice::actions::SingleBeliefAction {
      };    

      class AnalyzeProtoObject  extends execution::slice::actions::SingleBeliefAction {
      };    

    };

    };
    };
  };
};

// module motivation {
//     module slice {
// 		class AnalyzeProtoObjectMotive extends Motive {};	
// 	};
// };

#endif
