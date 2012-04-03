#ifndef SPATIALDATA_ICE
#define SPATIALDATA_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>

/**
 * Defines data structures pertaining to Places, for system-wide interfacing.
 *
 * @author Kristoffer Sjoo
 * @see
 */

module SpatialData {

  class NodeHypothesis {
    double x;
    double y;

    int hypID;
    int originPlaceID;
    int originNodeID;
  };

  sequence<double> DoubleOpt;
  sequence<bool> BoolSeq;
  sequence<DoubleOpt> CoordinateSeq;
  sequence<long> PlaceIDSeq;
  sequence<int> NodeIDSeq;  
  sequence<long> LongOpt;
  sequence<cogx::Math::Vector3> PlanePointSeq;
  sequence<NodeHypothesis> NodeHypothesisSeq;

 /**
   * Struct for passing 3D points belonging to a plane
   * Mainly for visualization on PB to compare against the
   * matching plane and see how well matching works.
   * @author Alper Aydemir
   * @see @do
   */
   
   class PlanePoints {
     PlanePointSeq points;
   };

  /**
   * Defines the possible states of the definition of a Place
   * 
   * @author Kristoffer Sjoo
   * @see
   */
  enum PlaceStatus { TRUEPLACE,
    		   PLACEHOLDER };
  /**
   * Represents a single Place as created by the Place layer.
   *
   * @author Kristoffer Sjoo
   * @see
   */
  class Place {
    // Unique ID number of this Place
    long id;

    // Status of Place definition
    PlaceStatus status;
  };

  /**
   * Message struct indicating the merging of two extant Places
   * into a single Place.
   *
   * @author Kristoffer Sjoo
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
   * @author Kristoffer Sjoo
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
   * @author Kristoffer Sjoo
   * @see
   */
  enum PlaceQueryStatus {PENDING,	
       			 //Not processed
    			 COMPLETED,	
			 //Return value available
			 PLACEINVALID,  
			 //Failed: place ID invalid
			 INTERNALERROR};
			//Failed: could not compute likelihood
  /**
   * Command struct for querying the likelihood of
   * being in a given Place
   *
   * @author Kristoffer Sjoo
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
   * @author Kristoffer Sjoo
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
   * @author Kristoffer Sjoo
   * @see
   */
  enum PathQueryStatus {QUERYPENDING,	
       		         //Not processed
    			 QUERYCOMPLETED,	
			 //Return value available
			 QUERYPLACE1INVALID, 
			 //Failed: start place ID invalid
			 QUERYPLACE2INVALID, 
			 //Failed: goal place ID invalid
			 QUERYINTERNALERROR};
			 //Failed: could not compute value
  /**
   * Command struct for querying the transition probabilities
   * involved in traversing a path
   *
   * @author Kristoffer Sjoo
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
   * @author Kristoffer Sjoo
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
  * @author Dorian Galves Lopez, Patric Jensfelt, Kristoffer Sjoo
  * @see
  */
  enum CommandType {GOTOPLACE,                
                    //GOTONODE,
		    GOTOPOSITION,
  		    GOFORWARD, 
		    GOBACK, 
		    //PERSONFOLLOW, 
		    TURN, 
		    // relative
		    TURNTO,
		    // absolute
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
  enum Completion {COMMANDPENDING,     
       		   // hasnt started 
		   COMMANDINPROGRESS, 
		   // started but not finished
		   COMMANDABORTED,     
		   // aborted due to higher priority command	
	           COMMANDFAILED,      
		   // finished unsuccessfully
		   COMMANDSUCCEEDED};  
		   // finished successfully

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
  enum Priority {URGENT,   
       		 // removes everything else from the queue
		 HIGH,     
		 // goes to the start of the queue
		 NORMAL};  
		 // gets done if nothing else of higher priority
                 // is in the queue.  

  /**
   * Command for moving the robot from the current Place
   * to a goal Place
   * 
   * @author Kristoffer Sjoo (cut 'n paste from Patric Jensfelt)
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
    DoubleOpt  pose; 
    //(x,y) or (x,y,a); for GOTOPOSITION
    DoubleOpt angle; 
    //Used for TURN, TURNTO and ROTATEABS
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
  enum AVSStatus {INPROGRESS,	
    		   SUCCESS,
		   FAILED
};	
	

  class SpatialObject {
    //    int id;	  
    // This is an ID unique among SpatialObjects
    string label; 
    // This is the same as in VisualObject
    cogx::Math::Pose3 pose;   
    // World coordinates
  };

  enum SpatialRelation{ ON,
  						INOBJECT,
  						INROOM
  };
  
    class ViewPointGenerationCommand {
    ///Object to generate the viewpoints for. TODO: what should this label contain?
     string label;
     PlaceIDSeq placestosearch; 
     AVSStatus status;
  };
  
  class RelationalViewPointGenerationCommand {
  	string searchedObjectCategory;
  	// INOBJECT if we are looking indirectly, INROOM if directly, see below
  	SpatialRelation relation;
  	// this is "" if we're looking directly, i.e. "mug in room1"
  	string supportObject; 
  	string supportObjectCategory;
  	// always (and I mean always) fill this
  	int roomId; 
    AVSStatus status;
  };

 class ObjectSearchResult{
	// this is really "mug", "table" etc.
  	string searchedObjectCategory;
  	// INOBJECT if we are looking indirectly, INROOM if directly, see enum:SpatialRelation
  	SpatialRelation relation; 
  	// this is "" if we're looking directly, i.e. "mug in sroom1"
  	string supportObjectCategory; 
  	// this is "" if we're looking directly, i.e. "mug in room1"
  	string supportObjectId; 
  	// always (and I mean always) fill this
  	int roomId; 
  	// The percentage of the relation probability mass that was already explored.
  	double beta; 
  };
  
  class ProcessConeGroup {
	AVSStatus status;     

	long coneId;
  };
  
  /**
   * Class for exposing AVS plan to planner
   */

  class ViewPoint{
	  cogx::Math::Vector3 pose;
	  double tilt;
	  double probability;
	  string label;
	  int closestPlaceId;
	  int areaId;
  };

  sequence<ViewPoint> ViewPointSeq;

  class SearchPlan{
	  ViewPointSeq plan;
  };

  sequence<byte> CellSeq;

  struct LocalGridMap
  {
    double xCenter;
    double yCenter;
    double cellSize;

    int size;
    CellSeq data;
  };
  
  /**
   * Command to process to viewpoint at the given WM address with the given object models (which should be sent to the recogniser).
   * @author Nick Hawes
   */
class ProcessViewPointCommand {
	AVSStatus status;     
	///The view point to process
	ViewPoint vp;
	///The objects that should be identified from the view point
	cast::cdl::StringSeq objectModels;

  };

  /**
   * Information struct that lets components know the status of map loading
   * @author Kristoffer S
   */
class MapLoadStatus {
  bool nodesWritten;		// NavGraphProcess signals all nav nodes (and edges) have been written to WM
  bool placesWritten;		// PlaceManager signals all Places (including placeholders) have been written to WM
  bool roomsWritten;		// Coma signals all ComaRooms have been written to WM
  bool categoryDataWritten;	// Categorical.SA signals all category place/placeholder properties have been written to WM
  bool obstacleMapsLoaded;	// SpatialControl signals all grid map data has been loaded
  bool localMapsLoaded;		// LocalMapManager signals all grid map data has been loaded
};
  
  interface AVSInterface
  {
  	void simulateViewCones(RelationalViewPointGenerationCommand cmd);
  };
  
  interface MapInterface {
    NodeHypothesisSeq refreshNodeHypothesis();
    bool isCircleObstacleFree(double x, double y, double radius);
    int findClosestNode(double x, double y);
    LocalGridMap getBoundedMap(double minx, double maxx, double miny, double maxy);
    DoubleOpt getGridmapRaytrace(double startAngle, double angleStep, int beamCount);
  };

};

#endif 
// SPATIALDATA_ICE
