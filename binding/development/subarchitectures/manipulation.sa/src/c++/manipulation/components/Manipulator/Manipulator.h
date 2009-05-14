/**
 * Manipulator
 * Grasps, prods, pushes stuff.
 *
 * @author Michael Zillich
 * @date October 2006
 */

#ifndef CAST_MANIPULATOR_H
#define CAST_MANIPULATOR_H

#include <cast/architecture/ManagedProcess.hpp>
#include <CoSyCommon/idl/Math.hh>
#include <vision/idl/Vision.hh>
#include <vision/utils/VideoClientProcess.h>

//default useful namespaces, fix to reflect your own code
using namespace cast; using namespace cast::cdl; 
using namespace cast::cdl::guitypes; 
using namespace std; using namespace boost; 

class Manipulator : public VideoClientProcess
{
private:
  enum ArmGoal {
    GOAL_IDLE = 0,         // no current goal
    GOAL_MOVE_ABOVE_OBJ,   // move above an object
    GOAL_REFINE_ABOVE_OBJ, // use visual servoing to refine pose above
    GOAL_APPROACH_OBJ,     // approaching an object prior to grasping
    GOAL_CLOSE_GRASP,      // closing the grasp around an object
    GOAL_LIFT_OBJ,         // lift grasped object
    GOAL_CARRY_OBJ,        // carry grasped object
    GOAL_PUTDOWN_OBJ,      // put down carried object, until it touches ground
    GOAL_RELEASE_GRASP,    // release grasp around object
    GOAL_RETREAT_OBJ,      // move away from object to safe distance
    GOAL_PNP_DONE,         // pick and place task completed
    GOAL_PAUSE,            // waiting for the pre-pause goal to be resumed
    GOAL_UNDEF
  };

  /**
   * Holds status information to be displayed in the GUI.
   */
  class StatusInfo  {
  public:
    Manipulator *man;
    Vision::ImageFrame img;
    bool have_img;
    std::vector<Math::Vector2D> rect2_points;
    bool have_rect2;
    std::vector<Math::Vector3D> rect3_points;
    Math::Vector3D rect3_center;
    Math::Vector3D rect3_axis;
    bool have_rect3;
    string doing;
    
    StatusInfo() : have_img(false), have_rect2(false), have_rect3(false) {}
    void Draw2D();
    void Draw3D();
    void Doing(const string &str) {doing = str;}
    void Done() {doing = "";}
  }; 

  ArmGoal curGoal;      // current internal goal
  ArmGoal pausedGoal ;  // active goal before we were paused
  ArmGoal stoppedGoal;  // active goal before we were stopped
  map<ArmGoal, ArmGoal> nextGoals;  // map of goal successions
  string curCmdId;  // framework id of currently executed command
  string curActionId;  // framework id of action that provided the command
  string curObjId;  // working memory id of current object of interest
  Vision::SceneObject *curObj;  // local copy of object of interest
  Vision::ROI *curROI;    // local copy of the ROI of interest
  Math::Pose3D curPutdownPose;
  std::string hand_wm_id;       // working memory address of hand pose
  StatusInfo stat_info;
  Math::Pose3D katana_base;  // pose of the Katana arm base w.r.t. world origin

  void InitController();
  void ExitController();

  void InitVision();
  void ExitVision();

  void GetCurrentSensors();
  void GetCurrentTCP();
  bool TargetReached();
  int HoldingObject();
  int CarriedObjectCollides();
  int CarriedObjectTouches();
  bool ObjectBetweenFingers();
  bool ObjectSlidesBetweenFingers();
  bool MoveArmTo(Math::Pose3D &pose);
  void StopArm();
  void StopGripper();
  void CloseGripper();
  void OpenGripper();

  void AddObstacle(const string &objId);
  void MoveObstacle(const string &objId);
  void RemoveObstacle(const string &objId);

  bool GetCamera(Vision::Camera &cam);
  bool FindRectangle(Vision::ImageFrame *img,
		     std::vector<Math::Vector2D> &points_prev, 
		     std::vector<Math::Vector2D> &points);
  bool FindRectangleRobust(std::vector<Math::Vector2D> &points);
  void ProjectRectangleToBase(const Vision::Camera &cam, double height,
			      const std::vector<Math::Vector2D> &img_points,
			      std::vector<Math::Vector3D> &cam_points);
  bool FindTopSurface3D(std::vector<Math::Vector3D> &points,
			Math::Vector3D &center, Math::Vector3D &axis, Math::Vector3D &norm);

  const char* GoalName(ArmGoal goal);
  void ClearGoals();
  bool ObjectInGrasp();

  // InitiateX methods set up a chain of goals for a given behaviour and execute
  // the first goal of that chain. The current goal is watched and if it is
  // finished the next goal of the chain is executed.
  bool InitiatePickAndPlaceBehaviour(const string &cmdId);
  void NewPickAndPlaceCmd(const WorkingMemoryChange &cmdId);
  void NewAction(const WorkingMemoryChange &cmdId);
  void StopCurrentBehaviour(const WorkingMemoryChange &cmdId);
  void PauseCurrentBehaviour(const WorkingMemoryChange &cmdId);
  void ResumePausedBehaviour(const WorkingMemoryChange &cmdId);
  void SetGoal(ArmGoal newGoal);
  void ExecuteGoal(ArmGoal goal);
  bool WatchGoal();

  void ExecuteMoveAboveObj();
  void ExecuteRefineAboveObj();
  void ExecuteApproachObj();
  void ExecuteGrasp();
  void ExecuteLiftObj();
  void ExecuteCarryObj();
  void ExecutePutdownObj();
  void ExecuteRelease();
  void ExecuteRetreatObj();
  void ExecutePnpDone(bool success);

  bool WatchIdle();
  bool WatchMoving();
  bool WatchRefineAbove();
  bool WatchApproach();
  bool WatchGrasping();
  bool WatchLiftObj();
  bool WatchCarryObj();
  bool WatchPutdownObj();
  bool WatchRelease();
  bool WatchRetreat();
  bool WatchPnpDone();

  void FindConvenientOrientation(const Math::Vector3D &pos, Math::Vector3D &ori);
  void FindPointingOrientation(const Math::Vector3D &pos, Math::Vector3D &ori);
  void FindPoseAbove(Vision::SceneObject &obj, Math::Pose3D &pose);
  bool FindTouchPoseVisual(Math::Pose3D &pose, bool offset);
  bool FindGraspPoseVisual(Math::Pose3D &pose);
  void AbortPickAndPlace();

  void StateChanged();
  void RegisterAction(const string & _action);
  void UpdateWorkingMemory();

  ///writes a struct to vision to prevent it from creating new objects
  void inhibitSceneProcessing();
  ///deletes the struct preventing vision from creating new objects
  void uninhibitSceneProcessing();

  cast::cdl::WorkingMemoryAddress m_inibitStructAddress;

protected:
  virtual void taskAdopted(const string &_taskID);
  virtual void taskRejected(const string &_taskID);
  virtual void redrawGraphics2D();
  virtual void redrawGraphics3D();
  virtual void redrawGraphicsText();
  virtual void runComponent();

public:
  Manipulator(const string &_id);
  virtual ~Manipulator();
  virtual void configure(map<string,string> & _config);
  virtual void start();
};

#endif
