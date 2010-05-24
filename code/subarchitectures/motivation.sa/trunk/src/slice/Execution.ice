#ifndef EXECUTION_ICE
#define EXECUTION_ICE

#include <cast/slice/CDL.ice>
#include <Planner.ice>


module execution {
  module slice {

    /**
     * A struct written to instruct the execution system to start
     * executing a plan.
     */
     ///    class 

    /**
     * Enum for boolean values + indeterminate
     */
    enum TriBool {
      TRITRUE,
      TRIFALSE,
      TRIINDETERMINATE
    };


    /**
     * Enum for action state transitons.
     */
    enum ActionStatus {
      PENDING,
      ACCEPTED,
      INPROGRESS,
      COMPLETE
    };

    class Action {
      ActionStatus status;
      TriBool success;
    };
    
    
    exception ActionExecutionException extends cast::CASTException {

    };
    
    /**
     * Module to contain some actions, although these might also go in
     * the slice files for the repsective SAs (or, better still,
     * entirely separately).
     */
    module actions {

      /**
       * comsys action
       */
      class ComsysQueryFeature extends Action {
	    /**
	     * The belief ID
	     */
	    string beliefID;
	    /**
	     * The name of the feature to query for
	     */
	    string featureID;
      };

      /**
       * Move the robot to a particular place.
       */
      class GoToPlace extends Action {
	/**
	 * The ID of the place.
	 */
	long placeID;
      };

      sequence<long> LongSeq;	
      /**
       * Move the robot to a particular place.
       */
      class ActiveVisualSearch extends Action {
	/**
	 * The IDs of the places to search
	 */
	LongSeq placeIDs;
      };

      /** Explore place. Not actually executed. **/
      class ExplorePlace extends Action {};
      

      /**
       * Dummy action to just print a message. Used for testing.
       */
      class PrintMessage extends Action {
	string message;
      };
      
      /**
       * Dummy action to just print a message. Used for testing.
       */
      class LogMessage extends Action {
	string message;
      };


    };



  };
};

#endif
