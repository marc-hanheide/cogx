//
// = FILENAME
//    NewNavController.hh
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = DESCRIPTION
//    
// = COPYRIGHT
//    Copyright (c) 2005 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef NewNavController_hh
#define NewNavController_hh

#ifndef DEPEND
#include <sstream>
#include <list>
#endif

#include <cast/architecture/ManagedComponent.hpp>
#include <Utils/RoboLookProxy.h>
#include <Navigation/NDMain.hh>
#include "NewTurnMotion.hpp"
#include <Navigation/NavGraphNode.hh>
#include <Navigation/MotionAlgorithm.hh>

namespace Cure {

class PoseProvider;
class NavGraph;
class LocalMap;


/**
 * This is the base class for classes that can listen to event from a
 * NavControler.
 *
 * @see NavControler::addEventListener
 * @see NavControler::delEventListener
 */
class NewNavControllerEventListener {
public:

  /**
   * Constructor
   * @param name name to identify this client during debugging
   */
  NewNavControllerEventListener(const std::string &name);

  /** 
   * Destructor
   */
  virtual ~NewNavControllerEventListener();


  /**
   * @return name of this client
   */
  std::string getName() const { return m_Name; }


  /**
   * Function that gets called when a task is aborted. Overload if
   * interested in this info
   *
   * @param taskID id of the task that was aborted
   */
  virtual void abortTask(int taskID) = 0;

  /**
   * Function that gets called when a task is done. Overload if
   * interested in this info
   *
   * @param taskID id of the task that was done
   */
  virtual void doneTask(int taskID) = 0;

  /**
   * Function that gets called when a task fails. Overload if
   * interested in this info
   *
   * @param taskID id of the task that failed
   */
  virtual void failTask(int taskID, int error) = 0;
protected:
  std::string m_Name;
};

/**
 * This class puts together the competences that are need to perform
 * navigation control, for example moving from point A to a point B or
 * rotating to a certain angle.
 *
 * @author Patric Jensfelt
 * @see NewNavControllerEventListener
 */
class NewNavController {
public:
  enum TaskType {
    TASKTYPE_OFF = 0,
    TASKTYPE_STOP,
    TASKTYPE_DIRECT,
    TASKTYPE_GOTOXY,
    TASKTYPE_GOTOXYA,
    TASKTYPE_ROTATE,
    TASKTYPE_APPROACH_POINT,
    TASKTYPE_APPROACH_XYA,
    TASKTYPE_BACKOFF,
    TASKTYPE_FOLLOWPERSON
  };

  enum ReturnValues {
    RETVAL_OK = 0,
    RETVAL_ERROR = 1,
    RETVAL_GOALNOTFOUND = 2,
    RETVAL_NOPATH = 3
  };

public:
  /// Reference to a PoseProvider. This is used as a source for the
  /// robot position
  Cure::PoseProvider *m_PP;

  /// Reference to NavGraph. This is used to plan the path between
  /// different parts of the environment
  Cure::NavGraph &m_Graph;

  /// Reference to LocalMap containing the local obstacles. This is
  /// used by the MotionAlgorithms
  Cure::LocalMap &m_LMap;

public:
  /**
   * Constructor
   */
  NewNavController(NavGraph &graph, LocalMap &lmap, cast::ManagedComponent* sc);

  /**
   * Destructor
   */
  virtual ~NewNavController();

  void setPoseProvider(PoseProvider &pp) { m_PP = &pp; }

  /**
   * You must call this function before you can use this class. It
   * will setup this like the shape of the robot
   *
   * @param configfile name of config file to read config data from
   */
  int config(const std::string &configfile);

  /**
   * Use this function to set the minumim velocities that are used
   * when the velocities are not zero. On some platforms there will be
   * no motion at all unless the velocity is over some threshold.
   *
   * @param minV min abs translation speed to send to send to motors
   * @param minW min abs rotation speed to send to send to motors
   */
  void setMinNonzeroSpeeds(double minV, double minW);

  /** 
   * Use this function if you want to configure the controller so that
   * it automatically turns all (v,angle) type commands into (v,w)
   * type commands, that is turn angles into angular velocities.
   *
   * @param doit true if you want to turn angles into angle speeds
   * @param gain P-controller gain
   */
  void setTurnAngleIntoSpeed(bool doit, double gain = 0.5);

  /**
   * Use this function to set the upper limits for translation and
   * rotation speed in gotomode, i.e. when the robot is moving towards
   * a position.
   * @param maxV max translation speed when reaching a position [m/s]
   * @param maxW max rotation speed when reaching a position [rad/s]
   */
  void setGotoMaxSpeeds(double maxV, double maxW);

  /**
   * Use this function to set the upper limits for translation and
   * rotation speed in rotation mode, i.e. when the robot is rotating
   * to reach a desired angle.
   *
   * @param maxV the max translation speed when reaching an angle
   * [m/s]. It must be non-zero since translation is sometime needed
   * when turning close to obstacles.
   * @param maxW the max rotation speed [rad/s] when reaching an angle
   */
  void setTurnMaxSpeeds(double maxV, double maxW);

  /**
   * Use this function to set the upper limits for translation and
   * rotation speed when movin torward the gateway node and the one after
   *
   * @param maxV max translation speed when reaching a position [m/s]
   * @param maxW max rotation speed when reaching a position [rad/s]
   */
  void setGatewayMaxSpeeds(double maxV, double maxW);

  /**
   * Use this function to set the speed for translation during backing off
   * @param v the backoff speed [m/s]
   */
  void setBackOffSpeed(double v);

  /**
   * Use this function to set the upper limits for translation and
   * rotation speed in follow mode
   *
   * @param maxV max translation speed  [m/s]
   * @param maxW max rotation speed [rad/s]
   *
   * @see followPerson
   * @see setFollowTolerances
   * @see setFollowDistance
   */
  void setFollowMaxSpeeds(double maxV, double maxW);

  /**
   * Use this function to specify how close the robot has to come to
   * an intermediate node along the path before it is said to be
   * reached. 
   *
   * @param tol tolerance to say that an intermediate node position is
   * reached [m]
   * @see NewNavController::setPositionToleranceFinal
   * @see NewNavController::setPositionToleranceGateway
   */
  void setPositionToleranceIntermediate(double tol) { m_InterGoalTol = tol; }

  /**
   * Use this function to specify how close the robot has to come to
   * the final node position it is said to be reached.
   * 
   * @param tol tolerance to say that final goal position is reached [m]
   * @see NewNavController::setPositionToleranceIntermediate
   * @see NewNavController::setPositionToleranceGateway
   */
  void setPositionToleranceFinal(double tol) { m_FinalGoalTol = tol; }
  
  /**
   * Use this function to specify how close the robot has to come to a
   * node before, in or after a gateway before it is said to be
   * reached. If a node after, in or before a gateway is the final
   * node along the path the tolerance for the final goal point will
   * be used
   *
   * @param tol tolerance to say that an node after, in or before a
   * gateway is reached [m]
   * @see NewNavController::setPositionToleranceIntermediate
   * @see NewNavController::setPositionToleranceFinal
   */
  void setPositionToleranceGateway(double tol) { m_GatewayGoalTol=tol; }

  /**
   * Use this function to specify how close to the target angle the
   * robot must get before it is satisfied during the during part of
   * the motion. [rad]
   *
   * @param tol tolearance to say when the target orientation has been
   * reached [rad].
   */
  void setOrientationTolerance(double tol) { m_FinalGoalTolRot = tol; }

  /**
   * Use this function to set the tolerances for the approach task
   */
  void setApproachTolerances(double tolDist, double tolRot) 
  {
    m_ApproachTolDist = tolDist;
    m_ApproachTolRot = tolRot;
  }
  
  /**
   * Set the desired distance that the following mode should keep to
   * the target location. That is if you call follow with (x,y) and
   * set this distance to 1m the robot will try to stay at a point
   * that is 1m from (x,y) pointing at (x,y). Notice that the distance
   * is measured from the center of the robot to the center of the
   * target position.
   *
   * @param dist the desired distance to keep to the point we follow
   * @param backupDist if the robot is closer than this is starts backing
   * @param goalDistToTarget the distance from the target to put the
   * goal for the robot. This could be the same as the dist but
   * preferrable something closer that so that the robot does not
   * start fine adjusting
   *
   * @see followPerson
   * @see setFollowMaxSpeeds
   * @see setFollowTolerances
   */
  void setFollowDistances(double dist, double backupDist, 
                          double goalDistToTarget = -1) 
  { 
    m_FollowDist = dist; 
    m_FollowBackupThreshold = backupDist; 
    if (goalDistToTarget >= 0) m_FollowBeforeTargetDist = goalDistToTarget;
  }

  /**
   * Use this function to give the setting for the following behavior
   * in terms of how far away from the desired the distance the person
   * can be before the robot starts to decrease the distance
   *
   * @param tolDist if we are within this distance from the desired
   * distance from the point we are following we start turning towards
   * it
   * @param tolRot when turnings toward the point to follow without
   * moving with stop turning when we are this close to the right
   * angle
   *
   * @see followPerson
   * @see setFollowMaxSpeeds
   * @see setFollowDistance
   */
  void setFollowTolerances(double tolDist, double tolRot)
  {
    m_FollowTolDist = tolDist;
    m_FollowTolRot = tolRot;
  }

  /**
   * Use this function to add an object to the list of objects being
   * called on done/fail/abort etc events
   * 
   * @param l objects that want to listen to events
   */
  void addEventListener(NewNavControllerEventListener *l);

  /**
   * Use this function to remove an object from the list of objects
   * being called on done/fail/abort etc events
   * 
   * @param l objects that want to listen to events
   */
  void delEventListener(NewNavControllerEventListener *l);


  /**
   * Turn on motion calculations. The ctrl will start in STOP mode but
   * execCtrl will be called.
   */
  void turnOn();

  /**
   * Use this function to turn motion calculation off and thus stop
   * calling execCtrl. This is useful when having more than one
   * process that want to control the robot motion.
   */
  void turnOff();
  
  /**
   * @return true if ctrl is not OFF. It could be in STOP mode
   * though. The main thing is that it calls execCtrl
   */
  bool isOn() const { return m_TaskType != TASKTYPE_OFF; }

  /**
   * @return true if the ctrl is turned OFF, i.e. not performing any
   * calculations and thus not calling execCtrl.
   */
  bool isOff() const { return m_TaskType == TASKTYPE_OFF; }

  /**
   * @return true if the ctrl is in STOP mode, not moving anywhere
   * but sending speed command 0,0 to execCtrl
   */
  bool isStop() const { return m_TaskType == TASKTYPE_STOP; }

  /**
   * @return the max error in position to say that an intermediate node
   * is reached [m]
   * @see NewNavController::setPositionToleranceFinal
   * @see NewNavController::setPositionToleranceIntermediate
   * @see NewNavController::getPositionToleranceFinal
   */
  double getPositionToleranceIntermediate() const { return m_InterGoalTol; }


  /**
   * @return the max error in position to say that the final goal
   * position is reached [m]
   * @see NewNavController::setPositionToleranceFinal
   * @see NewNavController::getPositionToleranceIntermediate
   * @see NewNavController::setPositionToleranceIntermediate
   */
  double getPositionToleranceFinal() const { return m_FinalGoalTol; }

  /**
   * @return the max error in position to say that a node before or
   * after gateway is reached
   * @see NewNavController::setPositionToleranceFinal
   * @see NewNavController::getPositionToleranceIntermediate
   * @see NewNavController::setPositionToleranceIntermediate
   */
  double getPositionTolerancGateway() const { return m_GatewayGoalTol; }

  /**
   * @return the max error in orientation to say that the goal angle
   * is reached [rad]
   */
  double getOrientationTolerance() const { return m_FinalGoalTolRot; }

  /**
   * Use this function to turn on trimming of the path so that the
   * robot will try to remove nodes from the grpah that it need not
   * visit because the path to the node after is close enough. This
   * way the motion will be smoother as the goal is further away. No
   * trimming is done with node just before or after a door and not of
   * the door itself.
   *
   * @param v true if you want to trim the path
   */
  void setUsePathTrimming(bool v) { m_UsePathTrimming = v; }

  /**
   * Use thsi functin to tell if the trimming of the path can operate
   * also on gateway nodes, and node before them or not.
   * 
   * @param v true if you want to allow trimming of gateway nodes
   */
  void setCanTrimGateways(bool v) { m_CanTrimGateways = v; }

  /**
   * @return true if trimming of th epath is turned on
   * @see NewNavController::setUsePathTrimming
   */
  bool isUsingPathTrimming() const { return m_UsePathTrimming; }

  /**
   * Use this function to set how far to allow the trimming of the
   * path to be done. This function sets the minimum number of
   * remaining nodes required to try to perform trimming.
   *
   * @param len minimum number of remaining nodes needed to do any
   * trimming if trimming is activated. Must be >= 1
   */
  void setMinPathLengthForTrim(unsigned int len);

  /**
   * Use this to set the max distance ahead that nodes can be trimmed away
   *
   * @param d the max distance to trim ahead
   * @see NewNavController::setUsePathTrimming
   */
  void setMaxPathTrimDist(double d) { m_MaxTrimDist = d; }

  /**
   * @return the configured path trimming distance
   * @see NewNavController::setUsePathTrimming
   */
  double getMaxPathTrimDist() const 
  { return (m_UsePathTrimming?m_MaxTrimDist:0); }

  /**
   * Set the max time without proess toward an intermediate node to
   * not fail task. If this timeout is set to <= 0 this mechanism will
   * not be used. For rotation and backOff maneuvers the timeout is
   * from that start of the task so you must make sure that the
   * timeout is long enough so that you can finish a rotation and
   * backOff task in time.
   *
   * @param t time in [s] before task fails due to timeout for lack of progress
   */
  void setProgressTimeout(double t);

  /**
   * @return the time before a timeout and the task is considered
   * failed. If <=0 this mechanism is disabled.
   */
  double getProgressTimeout() const { return m_ProgressTimeout; }

  /**
   * Call this function to calculate the next motion command and call
   * execCtrl with the right arguments. If the controller is turned
   * off function will not do anything
   */
  int updateCtrl();


  /**
   * Call this function to make the robot stop and go into STOP mode
   */
  int stop();

  /**
   * Use this function to set the speed directly as (v,w),
   * i.e. translation and rotation speed.
   */
  int setDirectSpeed(int taskID, double v, double w, double timeout);

  /**
   * Use this function to stop what is being done and move to a certain node
   *
   * @param taskID the id of the task that will be reported on done/fail/abort
   * @param id the id of the node to move to
   * 
   * @return 0 if OK, else non-zero
   */
  int gotoNode(int taskID, int id);

  /**
   * Use this function to go to a certain position.
   *
   * @param taskID the id of the task that will be reported on done/fail/abort
   * @param x x-coordinate for target location [m]
   * @param y y-coordinate for target location [m]
   * 
   * @return 0 if OK, else non-zero
   */
  int gotoXY(int taskID, double x, double y);

  /**
   * Use this function to goto a certain pose, i.e. position and
   * orientation. The robot will first try to get to the desired
   * positoina nd then turn on the spot.
   *
   * @param taskID the id of the task that will be reported on done/fail/abort
   * @param x x-coordinate for target location [m]
   * @param y y-coordinate for target location [m]
   * @param a target orientation [rad]
   * 
   * @return 0 if OK, else non-zero
   */
  int gotoXYA(int taskID, double x, double y, double a);

  /**
   * Use this function to move to a certain point exporessed in polar
   * coordinates in the map frame 
   *
   * @param taskID the id of the task that will be reported on done/fail/abort
   * @param a angle to move in [rad]
   * @param r distance to move in the direction [m]
   * 
   * @return 0 if OK, else non-zero
   */
  int gotoPolar(int taskID, double a, double r);

  /**
   * Use this function to rotate a certain amount [rad]
   *
   * @param taskID the id of the task that will be reported on done/fail/abort
   * @param da amount to turn [rad]
   * 
   * @return 0 if OK, else non-zero
   */
  int rotateRel(int taskID, double da);

  /**
   * Use this function to rotate to a certan angle [rad]
   *
   * @param taskID the id of the task that will be reported on done/fail/abort
   * @param a angle to rotate to [rad]
   * 
   * @return 0 if OK, else non-zero
   */
  int rotateAbs(int taskID, double a);

  /**
   * Use this function to approach a certain position. This was
   * designed for people following.
   *
   * @param taskID the id of the task that will be reported on done/fail/abort
   * @param x x-coordinate for target location [m]
   * @param y y-coordinate for target location [m]
   * 
   * @return 0 if OK, else non-zero
   */
  int approachXY(int taskID, double x, double y);

  /**
   * Use this function to approach a certain position and turn towards
   * a direction. This was designed for people following.
   *
   * @param taskID the id of the task that will be reported on done/fail/abort
   * @param x x-coordinate for target location [m]
   * @param y y-coordinate for target location [m]
   * @param a target orientation [rad]
   * 
   * @return 0 if OK, else non-zero
   */
  int approachXYA(int taskID, double x, double y, double a);

  /**
   * Use this function to backoff a certain distance
   *
   * @param taskID the id of the task that will be reported on done/fail/abort
   * @param dist distance to move backwards [m]
   * 
   * @return 0 if OK, else non-zero
   */
  int backOff(int taskID, double dist);

  /**
   * Call this function to have the robot follow a point
   * (x,y). Obvously you need to call this method repeatedly if the
   * target point is moving. as a following task is not really over no
   * abort event will be triggered as long as you issue new follow
   * commands. That is, as long as you keep calling followPerson you will
   * not get an abort event. But if you call stop or something else an
   * abort will be sent.
   *
   * @param taskID the id of the task that will be reported on done/fail/abort
   * @param x x-coordinate of point to follow
   * @param y y-coordinate of point to follow
   * @param dir the direction of motion for the point to follow
   * @param speed the speed for the points to follow (the speed of the point)
   * @param visibility the visibility of the point we follow
   * 
   * @return 0 if OK, else non-zero
   *
   * @see setFollowMaxSpeeds
   * @see setFollowTolerances
   * @see setFollowDistance
   */
  int followPerson(int taskID, double x, double y,
                   double dir, double speed, double visibility);

  
  /**
   * Use this function to order the robot to go to a specific area
   * 
   * @param taskID the id of the task that will be reported on done/fail/abort
   * @param areaId the id of the area we want to go to
   *
   * @return 0 if OK, else non-zero
   */
  int gotoArea(int taskID, int areaId);

  /**
   * Call tis function if you have calculated a path to follow and you
   * want it executed. You can choose to turn to the orientation
   * defined by the final node or not.
   *
   * @param taskID the id of the task that willbe reported on done/fail/abort
   * @param path list of nodes defining the path to follow
   * @param finalTurn true if we should turn according to the final node
   * 
   * @return 0 if OK, else non-zero
   */
  int execPath(int taskID, 
               const std::list<NavGraphNode> &path, bool finalTurn);

  /**
   * Function to get the node that the controller is curretly working
   * with, either trying to rotate to reach a certain angle or
   * translating there.
   * 
   * @return true if a node exists, else false
   */
  bool getCurrentNode(NavGraphNode &n) const;

  /**
   * Function to get the target node, i.e. the last node in the last
   * node along the path.
   * 
   * @return true if a node exists, else false
   */
  bool getTargetNode(NavGraphNode &n) const;

  /**
   * Use this function to tell what type of mission we are current conducting.
   *
   * @return the current state of the controller
   */
  int getTaskType() const { return m_TaskType; }

  /**
   * @return position of the robot according to the poseprovided that
   * was given to the costructor of this class
   */
  Pose3D getPose(bool usePrediction = false) const;

  /**
   * @return width of the robot [m]
   */
  double getRobotWidth() const { return m_ND.getRobotWidth(); }

//  /**
//   * Function to display in RoboLook
//   *
//   * @param pointert to proxy to RoboLook server
//   * @param clearEllipseTarget true if you want to clear ellipses
//   *        before you draw the new to mark the target location
//   * @param displayLocalMap true if you want to display the local map
//   * @param true if you want to display the graph
//   */
//  void displayRL(RoboLookProxy *rlp, bool clearEllipseTarget, 
//                 bool displayLocalMap = false, bool displayGraph = false);

protected:

  /**
   * This function is the one that actuall sends the commands to the hardware
   */
  virtual void execCtrl(Cure::MotionAlgorithm::MotionCmd &cmd) = 0;


  /**
   * Called internally when we task is aborted. This function then
   * calls abortTask() for any listening clients to tell about this
   */
  void reportAbort(int taskID);


  /**
   * Called internally when task is done. This function then calls
   * doneTask() for any listening clients to tell about this.
   */
  void reportDone(int taskID);


  /**
   * Called internally when we detect a failure. This function then
   * calls failTask() for any listening clients to tell about this.
   */
  void reportFail(int taskID, int error);


  /**
   * Call this function when you have constructed they graph
   * describing the path to follow to adjust the initial part of it to
   * remove cases where the robot is moving away from the final goal
   * to reach the first point on the path
   */
  void initTrimPath(const Cure::Pose3D &currPose,
                    std::list<NavGraphNode> &path);

  /**
   * Call this function to trim away nodes in the graph that we do not
   * need to visit to make the path smoother. We do not trim through
   * doors.
   */
  void trimPath(const Cure::Pose3D &currPose, std::list<NavGraphNode> &path);

  /**
   * @return 0 if OK to carry out task, else 1
   */
  int prepForTask(const std::string &mission);

  /**
   * Call this function to setup for monitoring progress against the
   * goal to signal for timeout when there is not enough progress
   */
  void startProgressMonitoring();

  /**
   * Call this function when the current task does not require
   * progress monitoring
   */
  void stopProgressMonitoring();

  /**
   * Calls NavGraph::findPath unless the target position is reachable
   * from the start pose. The rational behind this is that in cases
   * when the navigation graph is not complete the robot might be led
   * on a long detaour otherwise.
   *
   * @param goal goal position
   * @param path list of nodes to build with the path
   *
   * @param 0 if OK, else error code
   */
  int findPath(const Pose3D &goal, std::list<NavGraphNode> &path);

  /**
   * Calls NavGraph::findPath unless the target position is reachable
   * from the start pose. The rational behind this is that in cases
   * when the navigation graph is not complete the robot might be led
   * on a long detaour otherwise.
   *
   * @param goal pointer to goal node
   * @param path list of nodes to build with the path
   *
   * @param 0 if OK, else error code
   */
  int findPath(NavGraphNode *goal, std::list<NavGraphNode> &path);

  /**
   * Calls NavGraph::findPath unless the target position is reachable
   * from the start pose. The rational behind this is that in cases
   * when the navigation graph is not complete the robot might be led
   * on a long detaour otherwise.
   *
   * @param x x-coordinate of goal point
   * @param y y-coordinate of goal point
   * @param a xorientation of goal point
   * @param path list of nodes to build with the path
   *
   * @param 0 if OK, else error code
   */
  int findPath(double x, double y, double a, std::list<NavGraphNode> &path);

protected:
  std::list<NewNavControllerEventListener*> m_EventListeners;
  Cure::NewTurnMotion m_Turn;
  Cure::NDMain m_ND;
  int m_TaskType;

  /// Id of the current task. Tasks shoul have ids >= 0
  int m_TaskID;

  std::list<Cure::NavGraphNode> m_Path;

  /// Maximum distance ahead of the robot to trim away nodes
  double m_MaxTrimDist;

protected:
  cast::ManagedComponent* m_sc;
  double m_InterGoalTol;
  double m_FinalGoalTol;
  double m_GatewayGoalTol;
  double m_FinalGoalTolRot;

  double m_ApproachTolDist;
  double m_ApproachTolRot;

  double m_BackOffDist;

  double m_FollowDist;
  double m_FollowBackupThreshold;
  double m_FollowTolDist;
  double m_FollowTolRot;
  double m_MaxGotoV;
  double m_MaxGotoW;
  double m_MaxTurnV;
  double m_MaxTurnW;
  double m_MaxGatewayV;
  double m_MaxGatewayW;
  double m_BackOffV;
  double m_FollowBackOffV;
  double m_MaxFollowV;
  double m_MaxFollowW;

  double m_MinNonZeroV;
  double m_MinNonZeroW;
  bool m_TurnAngleIntoSpeed;
  double m_AngleGain;

  bool m_PrevWasGateway;

  /// true if we should use path trimming
  bool m_UsePathTrimming;  

  bool m_CanTrimGateways;

  /// Do not trim the path when it is this long or shorter, measured
  /// in number of remaining nodes.
  unsigned int m_MinPathLengthForTrim;

  /// Time when we made progress towards the goal last
  double m_LastProgressTime;

  /// Min distance to the goal so far to check for progress against
  double m_MaxProgressDist;

  /// Max time without progress towards the goal without getting a timeout
  double m_ProgressTimeout;

  /// The distance from the person target location and the position
  /// that the robot heads for
  double m_FollowBeforeTargetDist;

  double m_FollowX;   // The current x-position of the thing we follow
  double m_FollowY;   // The current y-position of the thing we follow
  double m_FollowDir; // The direction of motion of the thing we follow
  double m_FollowV;   // The speed of the thing we follow

  double m_DirectV;   // Translation speed when in direct mode
  double m_DirectW;   // Rotation speed when in direct mode
  double m_DirectTimeout;  // Timeout for direct speed mode
};

}; // namespace Cure

std::ostream&
operator<< (std::ostream &os, const Cure::NewNavController &a);


#endif // NewNavController_hh
