#ifndef DORA_EXECUTION_ICE
#define DORA_EXECUTION_ICE

#include <cast/slice/CDL.ice>
#include <Execution.ice>
//#include <beliefs.ice>
#include <Planner.ice>

module execution {
  module slice {

	module person {
		class PersonObservation {
			double existProb;
		};
	};

    /**
     * Module to contain some actions, although these might also go in
     * the slice files for the repsective SAs (or, better still,
     * entirely separately).
     */
    module actions {

      /**
       * Generate view cones for a particular model
       */
      class CreateRelationalConesForModel extends Action {
  
  	///The object to look generate cones for
	string model;
    string relation;
    string supportObject;
    string supportObjectCategory;
    int roomID;
      };

      /**
       * Process all cones at a given cone group
       */
      class ProcessConeGroupAction extends Action {
	/**
	 * Cone group ID 
	 */
	long coneGroupID;

	/**
	 * Cone group Belief ID on the Binder
	 */
	cast::cdl::WorkingMemoryAddress coneGroupBeliefID;
      };

    };
  };
};

#endif
