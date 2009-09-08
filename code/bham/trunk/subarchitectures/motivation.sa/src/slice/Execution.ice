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
    
    exception ActionExecutionException extends cast::CASTException {

    };


  };
};

#endif
