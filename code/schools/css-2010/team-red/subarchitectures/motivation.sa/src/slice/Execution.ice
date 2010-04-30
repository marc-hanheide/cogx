#ifndef EXECUTION_ICE
#define EXECUTION_ICE

#include <cast/slice/CDL.ice>
#include <beliefmodels.ice>
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
	    /**
	     * Generic question to ask... if empty, belief information is used
	     */
	    string question;
      };

      /**
       * comsys action
       */
      class ComsysTestFeatureValue extends Action {
	    /**
	     * The belief ID
	     */
	    string beliefID;
	    /**
	     * The feature type
	     */
	    string featureType;
	    /**
	    * The value to query
	    */
	    beliefmodels::autogen::featurecontent::FeatureValue featureValue;
	    /**
	     * Generic question to ask... if empty, belief information is used
	     */
	    string question;
	    

      };

      sequence<double> DoubleSeq;

      /**
       * Move the robot to a particular place.
       */
      class GoToPlaceRough extends Action {
	/**
	 * The ID of the place.
	 */
	long placeID;

	/**
	 * The tolerance with which we go there
	 **/
	DoubleSeq tol;
      };

      sequence<long> LongSeq;	
      sequence<string> StringSeq;	

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



      /**
       * Try to detect objects with these labels.
       */
      class DetectObjects extends Action {
	StringSeq labels;
      };
      
      /**
       * Run the people detector without moving the robot.
       */
      class DetectPeople extends Action {};

      /**
       * Turn the robot around on the spot to test the navigation without vision
       */
      class SpinAround extends Action {};
	
      /**
       * Turn the robot in stages to look for people.
       */
      class LookForPeople extends Action {};

      /** Just fart */
      class Fart extends Action {};
      
      /**
       * Turn the robot in stages to look for objects.
       */
      class LookForObjects extends Action {
	StringSeq labels;
      };

      /**
       * Turn the robot in stages to look for objects and people.
       */
      class LookForObjectsAndPeople extends Action {
	StringSeq labels;
      };
      
      /**
       * Use a combination of ptu and rotation of the platform to look for objects.
       */
      class PTULookForObjects extends Action {
	StringSeq labels;
      };

    };



  };
};

#endif
