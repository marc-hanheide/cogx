#ifndef EXECUTION_ICE
#define EXECUTION_ICE

#include <cast/slice/CDL.ice>
#include <Planner.ice>


module execution {
  module slice {

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
       * Move the robot to a particular place.
       */
      class GoToPlace extends Action {
	/**
	 * The ID of the place.
	 */
	long placeID;
      };
      

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
