#ifndef SPATIALDATA_ICE
#define SPATIALDATA_ICE

#include <cast/slice/CDL.ice>

/**
 * Defines data structures pertaining to Places, for system-wide interfacing.
 *
 * @author Kristoffer Sjöö
 * @see
 */

module SpatialData {

  sequence<double> DoubleOpt;
  sequence<long> PlaceIDSeq;
  sequence<long> LongOpt;

  /**
   * Defines the possible states of the definition of a Place
   * 
   * @author Kristoffer Sjöö
   * @see
   */
  enum PlaceStatus { TRUEPLACE,
    		   PLACEHOLDER };
  /**
   * Represents a single Place as created by the Place layer.
   *
   * @author Kristoffer Sjöö
   * @see
   */
  class Place {
    // Unique ID number of this Place
    long id;

    // Status of Place definition
    PlaceStatus status;

    // Connection counter
    long connectionCounter;
  };


  class MaryCommand
  {
    string text;
  };


  class PlayMp3Command
  {
    long song;
  };


  /**
   * Message struct indicating the merging of two extant Places
   * into a single Place.
   *
   * @author Kristoffer Sjöö
   * @see
   */
  class PlaceMergeNotification {
    // The places that were merged
    // Will contain exactly two entries
    PlaceIDSeq mergedPlaces;

    // ID of the resulting Place
    // May be one of the mergedPlaces
    long resultingPlace;
  };

  /**
   * Message struct indicating the splitting of an extant Place
   * into two Places.
   *
   * @author Kristoffer Sjöö
   * @see
   */
  class PlaceSplitNotification {
    // The place that was split
    long splitPlace;

    // The resulting places.
    // Will contain exactly two entries
    // May contain the original Place ID
    PlaceIDSeq resultingPlaces;
  };

  /**
   * Possible states of a value request from Spatial.sa
   * 
   * @author Kristoffer Sjöö
   * @see
   */
  enum PlaceQueryStatus {PENDING,	//Not processed
    			 COMPLETED,	//Return value available
			 PLACEINVALID,  //Failed: place ID invalid
			 INTERNALERROR};//Failed: could not compute likelihood
  /**
   * Command struct for querying the likelihood of
   * being in a given Place
   *
   * @author Kristoffer Sjöö
   * @see
   */
  class PlaceLikelihoodRequest {
    // The place inquired about (input)
    long queryPlaceID;

    // Likelihood value (output)
    double likelihood;

    // Status of the request (output)
    PlaceQueryStatus status;
  };

  struct PlaceProbability {
    long placeID;
    double prob;
  };
  sequence<PlaceProbability> PlaceProbSeq;

  /**
   * Command struct for querying the posterior probability of
   * being in a given Place
   *
   * @author Kristoffer Sjöö
   * @see
   */
  class PlacePosteriorRequest {
    // The place inquired about (input)
    long queryPlaceID;

    // Posterior value (output)
    double posterior;

    // Status of the request (output)
    PlaceQueryStatus status;
  };

  /**
   * Possible states of a path value request from Spatial.sa
   * 
   * @author Kristoffer Sjöö
   * @see
   */
  enum PathQueryStatus {QUERYPENDING,	//Not processed
    			 QUERYCOMPLETED,	//Return value available
			 QUERYPLACE1INVALID, //Failed: start place ID invalid
			 QUERYPLACE2INVALID, //Failed: goal place ID invalid
			 QUERYINTERNALERROR};//Failed: could not compute value
  /**
   * Command struct for querying the transition probabilities
   * involved in traversing a path
   *
   * @author Kristoffer Sjöö
   * @see
   */
  class PathTransitionProbRequest {
    // Start place (input)
    long startPlaceID;
    // Goal place (input)
    long goalPlaceID;
    // Number of requested successor states (input)
    // Negative value requests probability for all Places
    int noSuccessors;

    // Probability of successful transition from start to goal
    // (output)
    double successProb;

    // Highest probabilites of successor Places (goal included)
    // Ordered high->low (output)
    PlaceProbSeq successors;

    // Status of request (output)
    PathQueryStatus status;
  };

  /**
   * Command struct for querying the modelled transition costs
   * of traversing a path
   *
   * @author Kristoffer Sjöö
   * @see
   */
  class PathTransitionCostRequest {
    // Start place (input)
    long startPlaceID;
    // Goal place (input)
    long goalPlaceID;

    // Cost of transition (output)
    double cost;

    // Status of request (output)
    PathQueryStatus status;
  };

 /**
  * This enum defines the different commands that one can issue to the nav.sa
  *
  * @author Dorian Galves Lopez, Patric Jensfelt, Kristoffer Sjöö
  * @see
  */
  enum CommandType {GOTOPLACE,                
                    //GOTONODE,
		    GOTOPOSITION,
  		    GOFORWARD, 
		    GOBACK, 
		    //PERSONFOLLOW, 
		    TURN, // relative
		    TURNTO, // absolute
		    //EXPLORE,
		    //EXPLOREFLOOR,
		    STOP,
  	    // This is a special command to make the rest of the tasks
	    // be pending until the blockcontrol command is removed by
	    // the sender or cancelled by an urgent task.  It allows to
	    // send NavCtrlCommands bypassing the TranslationProcess in
	    // a safe way.
	            BLOCKCONTROL};

 /**
  * Different states that the current command being executed is in
  *
  * @author Patric Jensfelt
  * @see
  */
  enum Completion {COMMANDPENDING,     // hasnt started 
		   COMMANDINPROGRESS, // started but not finished
		   COMMANDABORTED,     // aborted due to higher priority command	
	           COMMANDFAILED,      // finished unsuccessfully
		   COMMANDSUCCEEDED};  // finished successfully

 /**
  * Error codes for the status information for a command
  *
  * @author Patric Jensfelt
  * @see
  */
  enum StatusError {PERSONNOTFOUND,
                    TARGETUNREACHABLE,
                    REPLACEDBYNEWCMD,
                    CMDMALFORMATTED,
                    NONE,
                    UNKNOWN};
 /**
  * Priority levels that can be used to tell where in the command queue a 
  * command will end up
  *
  * @author Patric Jensfelt
  * @see
  */
  enum Priority {URGENT,   // removes everything else from the queue
		 HIGH,     // goes to the start of the queue
		 NORMAL};  // gets done if nothing else of higher priority
                           // is in the queue.  

  /**
   * Command for moving the robot from the current Place
   * to a goal Place
   * 
   * @author Kristoffer Sjöö (cut 'n paste from Patric Jensfelt)
   * @see
   */
  class NavCommand {
    // What do you want the robot to do?
    CommandType cmd;

    // What priority?
    Priority prio;

    // Place ID for GOTOPLACE
    LongOpt destId;

    // Target location, in the coordinate system of the Sensory layer
    // (I.e. robocentric, not necessarily robot-relative) (input)
    DoubleOpt  pose; //(x,y) or (x,y,a); for GOTOPOSITION
    DoubleOpt angle; //Used for TURN, TURNTO and ROTATEABS
    DoubleOpt distance;

    // A tolerance can be specified; exact semantics as yet
    // not defined
    DoubleOpt tolerance;

    // Possible errors
    StatusError status; 

    // Field that tells if the command is in progress, still pending, etc
    Completion comp;
  };

  /**
   * Commands for visual search
   * 
   * @author Alper Aydemir
   * @see
   */
  enum AVSAction {PLAN,	//Plan and then execute the generated plan
    			 STOPAVS};	// Stop the execution of a plan (not pause)
	
  /**
   * Command structure for visual search
   * 
   * @author Alper Aydemir
   * @see
   */	
  class AVSCommand {
    PlaceIDSeq placestosearch;
    AVSAction cmd;
  };

};
#endif // SPATIALDATA_ICE
