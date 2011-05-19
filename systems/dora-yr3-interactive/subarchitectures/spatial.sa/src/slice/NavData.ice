#ifndef NAVDATA_ICE
#define NAVDATA_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>

/**
 * The NavData structures are the structures from the nav.sa and include 
 * things like the robot pose, the line and navigation map.
 *
 * @author Patric Jensfelt
 * @see
 */
module NavData {

  struct ViewPoint{
    cogx::Math::Vector3 pos;
    double pan;
    double tilt;
    double length;
  };

  
  sequence<ViewPoint> SearchPlan; 
 
class ObjectSearchPlan{
  SearchPlan planlist;
};


 sequence<cogx::Math::Vector3> PointCloud;
 sequence<int> PointCloudLabels;
  class PlanePopout
  {
	PointCloud pcloud;
	PointCloudLabels plabels;
  };
  


interface NavGraphInterface{
  double getPathLength(double xS, double yS, double aS,
      double xG, double yG, double aG);
  int getAreaId(double x, double y, double a, double maxDist);
  int getClosestNodeId(double x, double y, double a, double maxDist);
};

    sequence<double> DoubleOpt;
    sequence<string> StringOpt;
  sequence<long> LongOpt;

  /**
   * Encodes the robot pose in in 2D, i.e. x,y and theta (orientation). 
   * It also contains the uncertainty. The pose is typically expressed in
   * the world frame.   
   *      
   * @author Patric Jensfelt
   * @see         
   */
  class RobotPose2d {
    // The time when this pose was updated
    cast::cdl::CASTTime time;

    // x-coordinate
    double x;

    // y-coordinate
    double y;

    // orientation
    double theta;

    // Optional covariance matrix.
    // If present should encode the covariance matrix by its elements in the
    // order: c11 c12 c13 c22 c23 c33
    // Note that we only encode the upper triangle as it is symmetric
    DoubleOpt cov;
  };

 /**
  * This struct models an end point in the line map. Each line is 
  * represented by its start and end points. These points each have an id 
  * along with the x,y-coordinates. A point can be shared between more than
  * one line segement, that is why we do not represent the line directly but
  * say that it consists of points 
  * 
  * @author Patric Jensfelt
  * @see
  */
  struct LineMapEndPt {
    // The ide of the point
    long id;

    // x-coordinate
    double x;

    // y-coordinate
    double y;
  };

 /**
  * A struct for a line segment consisting of a start and end point.
  *
  * @author Patric Jensfelt
  * @see
  */
  struct LineMapSegement {
    // The start point of the line segment
    LineMapEndPt start;

    // The end point of the lne segment
    LineMapEndPt end;
  };

  /**
   * A vector of line segments that in effect defines the data in the line map.
   *
   * @author Patric Jensfelt
   * @see LineMap
   */
  sequence<LineMapSegement> LineMapSeq;

 /**
  * The line map combining the data with a timestamp that tells when the map
  * was last updated,
  *
  * @author Patric Jensfelt
  * @see
  */
  class LineMap {
    // The time when the map was last updated
    cast::cdl::CASTTime  time;

    // The line segments contained in the map
    LineMapSeq lines;    
  };

 /**
  *
  *
  * @author Patric Jensfelt
  * @see
  */
 struct AreaType {
   /// The area type/class string
   string name; 

   /// The area type/class number for debugging
   long id;     
 };

 sequence<AreaType> AreaTypeOpt;

 /**
  * A struct that represents an area in the topology. If a place labeling 
  * system is running such an area might also contain information about 
  * the type of area.
  *
  * @author Patric Jensfelt
  * @see
  */
  class Area { 
    long id;
    AreaTypeOpt type;
  };

 /**
  * The robot position in the topological map, i.e. what area is the robot in?
  *
  * @author Patric Jensfelt
  * @see
  */
  class TopologicalRobotPos { 
    // High-level robot pose
    long areaId;
  };

 /**
  * A free space node representing a position that the robot has been to 
  * meaning that at least at that time this point was free space. Each node 
  * has a unique id, the id of the area it belongs to and a pose. In case the
  * node represents a gateway (door) signified by the variable gateway=1 it 
  * will also contain the with of the door. For doors the direction is defined
  * as the direction from the left to the right door post. That is the line 
  * defines the directio of the local boarder between the two areas. For
  * normal nodes the direction is just the direction that the robot had when 
  * it visited the node the first time.
  *
  * @author Patric Jensfelt
  * @see
  */
  class FNode {  // Free Node

    // The (unique) id of the node
    long nodeId;

    // Pose of the free space node
    double x;
    double y;
    double theta;

    // Id of the area where the node is
    long areaId;

    // gateway (1) or not (0)
    short gateway; 

    // If this is a gateway, the width gives the width of it
    DoubleOpt width;

    // Optional information about the area type
    AreaTypeOpt type;
  };

 /**
  * An edge between two free space nodes that encodes the connectivety of 
  * free space. The edges tells the robot how to move from one node to 
  * another and the cost can be used to bias the robot towards certain paths. 
  * By default the costs is just teh distance between the nodes.
  *
  * @author Patric Jensfelt
  * @see
  */
  class AEdge { // Accessibility Edge
  
    // Id of the start node
    long startNodeId;

    // Id of the end node
    long endNodeId;

    // Cost for this edge 
    double cost; 
  };

 /**
  * This class holds information about the observation of an object. 
  * The information contain in this class can all be provided by vision. 
  * The distance and angles are expressed in the camera frame. The is that 
  * this information can then be transformed by the nav.sa into the object 
  * location in a world frame.
  *
  * @author Patric Jensfelt
  * @see
  */
  class ObjObs {
    // The time when the object was observed
    cast::cdl::CASTTime time;
    
    // The category of the objects like rice (what else!)
    string category;

    // Optional object id
    LongOpt id;

    // Optional distance estimate from the camera to the object
    DoubleOpt dist;

    // Optional angle observation. The first angle (if available) is
    //interpretaed as the "pan" angle and the second (if available) as
    //the "tilt" angle in the image
    DoubleOpt angles;

  };
  
 /**
  * This class holds information about an object and its location
  *
  * @author Patric Jensfelt
  * @see
  */
  class ObjData {

    // Category of the objects
    string category;

    // Id of this object
    long objectId;

    // x-coordinate of the object
    double x;

    // y-coordinate of the object
    double y;

    // z-coordinate of the object
    double z;

    // Optional sequence that contains angles if used. If only one
    // angle it is yaw, i.e. rotation around the z-axis.  
    DoubleOpt angles;

    // Id of area where it was observed
    long areaId;

    // x-coordinate of robot when object was observed
    double camX;

    // y-coordinate of robot when object was observed
    double camY;
    
    // Probability of existance
    double probability;
  };

  sequence<Area> AreaSequence;
  sequence<FNode> FNodeSequence;
  sequence<AEdge> AEdgeSequence;
  sequence<ObjData> ObjectSequence;

 /**
  * The so called Navigation Graph which contains the free space nodes and 
  * the edges that connect these along with the object.
  *
  * @author Patric Jensfelt
  * @see
  */
  class NavGraph {
    
    // The nodes
    FNodeSequence fNodes;

    // The edges
    AEdgeSequence aEdges;

    // The objects
    ObjectSequence objects;
   
  };

 /**
  * This enum defines the different commands that one can issue to the nav.sa
  *
  * @author Dorian Galves Lopez, Patric Jensfelt
  * @see
  */
  enum CommandType {GOTOAREA,                
                    GOTONODE,
		    GOTOPOSITION,
  		    GOFORWARD, 
		    GOBACK, 
		    PERSONFOLLOW, 
		    TURN, 
		    // relative
		    TURNTO, 
		    // absolute
		    EXPLORE,
		    EXPLOREFLOOR,
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
  enum Completion {PENDING,     
       		   // hasnt started 
		   INPROGRESS, 
		   // started but not finished
		   ABORTED,     
		   // aborted due to higher priority command	
	           FAILED,      
		   // finished unsuccessfully
		   SUCCEEDED};  
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
   * This is the command that clients wanting to control the robot should write 
   * to the nav.sa working memory
   */
  class NavCommand {

    // What do you want the robot to do?
    CommandType cmd;

    // What priority?
    Priority prio;

    // area id for GOTOAREA
    // node if for GOTONODE
    LongOpt destId;
   
    // the string can be things like "kitchen" or "tv" 
    // for some of the commands the string will be used 
    // and for others it will be ignored.    
    StringOpt target;
    
    // for commands in terms of x, y, theta
    // in case there are some humans that want to communicate
    // in those terms
    DoubleOpt pose;

    // for commands in polar coordinates
    DoubleOpt angle;

    // for movement commands in polar coordinates or 
    // commands like "go forward 1m".
    DoubleOpt distance;

    // You can specify an optional tolerance for a goto type command.
    // When the robot comes closer than this it still consider the point
    // reached. For a goto x,y type command it will be interpreted as the 
    // tolerance in distance from the goal [m], for turn type commands it is 
    // the angle error [rad] allowed and when both apply the vector should
    // first the distance tolerance and then the angle tolerance
    DoubleOpt tolerance;

    // Possible errors
    StatusError status; 

    // Field that tells if the command is in progress, still pending, etc
           Completion comp;
  };

  /**
   * If you write an instance of this class to working memory and the
   * NavControl will be inhibited and will not send any motion commands to 
   * the hardware. This can be handy if you want to take control of the 
   * motion.
   * 
   * @author Patric Jensfelt
   * @see 
   */
  class InhibitNavControl {
    // Optional message tell why NavControl should be inhibited, 
    // used for debugging
    StringOpt message;
  };

  
  // Local code for the communication within the subarch.
  enum InternalCommandType {lGOTOXYA, 
		            lGOTOXY,
                            lGOTOXYROUGH,
                            lGOTOPOLAR,
		            lGOTOAREA,
		            lGOTONODE,
		            lROTATEREL, 
		            lEXPLORE,
		            lEXPLOREFLOOR, 
		            lROTATEABS,
		            lBACKOFF,
		            lFOLLOWPERSON,
                            lSTOPROBOT};
  
  /**
   * This is the nav.sa internal representation for the command to execute. 
   * 
   * NOTE: Should not be written to WM by anyone unless you know what you do
   *
   * @author Dorian Galves Lopez, Patric Jensfelt
   */
  class InternalNavCommand {
    
    InternalCommandType cmd;
    
    double x;
    double y;
    double r;
    double theta;
    double distance;
    long areaId;
    long nodeId;
    bool SetExplorerConfinedByGateways;

    // You can specify an optional tolerance for a goto type command.
    // When the robot comes closer than this it still consider the point
    // reached. For a goto x,y type command it will be interpreted as the 
    // tolerance in distance from the goal [m], for turn type commands it is 
    // the angle error [rad] allowed and when both apply the vector should
    // first the distance tolerance and then the angle tolerance
    DoubleOpt tolerance;

    // Feedback about progress
    StatusError status;
    Completion comp;       
  };

  /**
   * This struct contains information about a person that we are tracking
   *
   * @author Patric Jensfelt
   */
  class Person {
    cast::cdl::CASTTime  time; 
    // timestamp when last updated

    // Id of the person
    long id;

    // Position of the person
    double x;
    double y;

    // NOTE that this is very hard to estimate and next to impossible
    // when the person is standing still
    double direction;

    // Speed of the person
    double speed;

    // Area where the person is (or rather where the robot was when observed)
    long areaId;
  };


  /**
   * This struct contains information about which of the tracked
   * people is followed right now
   *
   * @author Patric Jensfelt
   */
  class PersonFollowed {
    cast::cdl::CASTTime  time; 
    // timestamp when last updated
    long id;                   
    // -1 means no person followed
  };
 
};

#endif 
// NAVDATA_ICE
