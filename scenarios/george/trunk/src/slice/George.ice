#ifndef COGX_GEORGE_ICE
#define COGX_GEORGE_ICE

#include <cast/slice/CDL.ice>
#include <Execution.ice>
#include <motivation.ice>


module execution {
  module slice {

	/*
	* Class used to store information about last viewcone moved to. Hopefully name should change.
	*/
	class Robot {
			cast::cdl::WorkingMemoryPointer currentViewCone;
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
