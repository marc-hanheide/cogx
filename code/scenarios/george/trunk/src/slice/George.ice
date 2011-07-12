#ifndef COGX_GEORGE_ICE
#define COGX_GEORGE_ICE

#include <cast/slice/CDL.ice>
#include <Execution.ice>


module execution {
  module slice {


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

#endif
