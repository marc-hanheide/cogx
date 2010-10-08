#ifndef EXECUTION_ICE
#define EXECUTION_ICE

#include <cast/slice/CDL.ice>
//#include <beliefs.ice>
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
       * Move the robot to a particular place.
       */
      class GoToPlace extends Action {
	/**
	 * The ID of the place.
	 */
	long placeID;
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
       * Turn the robot in stages to look for people.
       */
      class LookForPeople extends Action {};

      /**
       * Turn the robot in stages to look for objects.
       */
      class LookForObjects extends Action {
	StringSeq labels;
      };



      /**
       * Foreground (i.e. enable for recognition) the listed models.
       */
      class ForegroundModels extends Action {
	StringSeq models;
      };

      /**
       * Background (i.e. disable for recognition) the listed models.
       */
      class BackgroundModels extends Action {
	StringSeq models;
      };

      /**
       * Run object recogniser using previously foregrounded models
       */
      class RecogniseForegroundedModels extends Action {

      };


      /**
       * Generate view cones for a particular model
       */
      class CreateConesForModel extends Action {
  
	///The object to look generate cones for
	string model;
	///The IDs of the places to search
	LongSeq placeIDs;      
      };

      /**
       * Go to a particular view cone.
       */
      class ProcessCone extends Action {
	///I think this should be the belief which refers to a particular cone
	//  string beliefID;
	cast::cdl::WorkingMemoryAddress coneAddress;
      };

      class SingleBeliefAction extends Action {
	cast::cdl::WorkingMemoryAddress beliefAddress;	
      };

      class BeliefPlusStringAction extends SingleBeliefAction {
	string value;	
      };

      class SingleStringAction extends Action {
	string value;	
      };


      class AskForColour extends SingleBeliefAction {
      };

      class AskForShape extends SingleBeliefAction {
      };

      class AskForIdentity extends SingleBeliefAction {
      };

      class AskPolarColour extends BeliefPlusStringAction {
      };

      class AskPolarShape extends BeliefPlusStringAction {
      };

      class AskPolarIdentity extends BeliefPlusStringAction {
      };


      class LearnColour extends BeliefPlusStringAction {
      };

      class LearnShape extends BeliefPlusStringAction {
      };

      class LearnIdentity extends BeliefPlusStringAction {
      };

      class AskForObjectWithFeatureValue extends SingleStringAction {
      };    

      /**
       * comsys action
       */
      // class ComsysTestFeatureValue extends Action {
      // 	    /**
      // 	     * The belief ID
      // 	     */
      // 	    string beliefID;
      // 	    /**
      // 	     * The feature type
      // 	     */
      // 	    string featureType;
      // 	    /**
      // 	    * The value to query
      // 	    */
      // 	    beliefmodels::autogen::featurecontent::FeatureValue featureValue;
      // 	    /**
      // 	     * Generic question to ask... if empty, belief information is used
      // 	     */
      // 	    string question;


      // };


    };
  };
};

#endif
