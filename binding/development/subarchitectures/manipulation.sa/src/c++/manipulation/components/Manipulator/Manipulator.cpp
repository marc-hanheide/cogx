/**
 * Manipulator
 * Grasps, prods, pushes stuff.
 *
 * @author Michael Zillich
 * @date October 2006
 *
 * Note: we use lots of global variables here instead of putting them inside
 * the Manipulator class (as would be good practice) because of cross-references
 * with cmake and instantiations and stuff.
 *
 * TODO: select "best" rectangle not with nearness to image center, but use
 *       projected object pose
 * TODO: calibrate proximity sensors with finger opening
 * TODO: filter sensor values, take mean of 3 or something
 * TODO: short (4 or so) history of goal poses, drawn in decreasing brightness
 *       draw goal in blue, once reached in green, if unreachable in red
 */

#include <cstdio>
#include <csignal>
#include <cmath>
#include <algorithm>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include <cast/architecture/ChangeFilterFactory.hpp>

#include "vision/utils/VisionUtils.h"
#include "vs2/VisionCore.hh"
#include "vs2/BufferVideo.hh"
#include "vs2/Rectangle.hh"
#include "vs2/LJunction.hh"

#include "manipulation/ManipulationGoals.h"
#include "manipulation/idl/Manipulation.hh"

#include <katanaclnt/KatanaClient.h>
#include "Manipulator.h"

// MHN: TEMPORARY UNTIL PLANNING KICKS IN...
#include <planning/idl/PlanningData.hh>

using namespace Manipulation; using namespace Vision; using namespace Z; using namespace boost;

static const bool POINTING_HACK = false;

const double TINY = 1e-9;

// history of goals to be displayed
#define GOAL_HISTORY_SIZE 1

// TODO: incremental image processing uses a timer, which seems to
// collide with timers in robot controller. in anyh case it leads to crashes.
// so disable for now
#define INCREMENTAL_IMAGE_PROC false

// The manipulator is also a "head" carrying a camera.
// Robot head is 0, hand "head" is 1.
#define MANIPULATOR_HEAD_NUM 1

// Workspace:
// We assume that workspace ground plane is parallel to x-y plane, z is pointing
// up.  The robot base is placed in the workspace origin (0,0,0).

// how much to move above an object after grasping, while lifting and
// carrying the object...
static const double POST_GRASP_LIFT_OFFSET_Z = 0.15;
static const double POST_GRASP_CARRY_OFFSET_Z  = 0.15;

// how much to back away after putting down
static const double POST_GRASP_RETREAT_OFFSET_Z = 0.10; // 0.40...

// MHN: How much to move above object before looking for the top surface...
static const double MOVE_ABOVE_OFFSET_Z = 0.09; 
// MHN: Magic offsets to actually position above the object, and to
// grasp it once valid top surface has been found...
static const double ZERO_OFFSET_X = 0.10;
static const double OBSERVE_GRASP_OFFSET_Z = 0.15;
static const double MAGIC_GRASP_OFFSET_Y = 0.02;
// MHN: Threshold on the gripper encoder values, which determines if
// the gripper has actually managed to grasp the target object...
static const int OBJECT_GRASPED_ENCODER_THRESH_HIGH = 16000;
static const int OBJECT_GRASPED_ENCODER_THRESH_LOW = 10500;

// MHN: The z-axis value below which the arm will be stopped from
// moving -- set artificially in <MoveArmTo()>...
static const double LOWEST_TCP_Z = -0.10;

// MHN: Threshold on the distance between the projection of the
// image-plane candidate object into 3D space, and the actual object
// centroid in 3D space...
static const double IMG_PROJECT_3D_OBJECT_DIST_THRESH = 0.13;

// acceptable position error for visual servoing
#define VISUAL_POS_ERR 0.05
// acceptable angular error for visual servoing
#define VISUAL_ANGLE_ERR M_PI/9.

// tactile sensor threshold for put down
static int grasp_putdown_thr = 10;
// tactile sensor threshold for mid-movment collision of carried object
static int grasp_collision_thr = 10;
// tactile sensor threshold for power grasp
static int grasp_firm_thr = 120;
// proximity sensor threshold
// TODO: find out vie calibration observing noise
static int prox_near_thr = 20;
static int prox_slide_thr = 5;

// finger sensor (tactile and proximity) zero readings
static vector<int> sensors_zero;
// current readings
static vector<int> sensors_cur;
// readings when object was grasped
static vector<int> sensors_hold;
// current tool center pose
static Pose3D tcp_cur;
// current goal pose
static list<Pose3D> tcp_goals;
// height of last grasp which is used as the height for putting down
static double grasp_z = 0.;
// ignore new objects while manipulating stuff
static bool ignore_new_objects = false;
// target pose of currently executed movement
static Pose3D curTargetPose;

// Gripper encoder values...
static vector<int> gripper_encoder;

// vision stuff
VisionCore *vcore = 0;
BufferVideo *bufvid = 0;

// connection to the Katana arm
KatanaClient *kat_clnt = 0;

// for translating goals to strings
const char *goalnames[] =
{
  "GOAL_IDLE",
  "GOAL_MOVE_ABOVE_OBJ",
  "GOAL_REFINE_ABOVE_OBJ",
  "GOAL_APPROACH_OBJ",
  "GOAL_CLOSE_GRASP",
  "GOAL_LIFT_OBJ",
  "GOAL_CARRY_OBJ",
  "GOAL_PUTDOWN_OBJ",
  "GOAL_RELEASE_GRASP",
  "GOAL_RETREAT_OBJ",
  "GOAL_PNP_DONE",
  "GOAL_PAUSE",
  "UNDEF"
};

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new Manipulator(_id);
  }
}


Manipulator::Manipulator(const string &_id)
: 
  WorkingMemoryAttachedComponent(_id),
  VideoClientProcess(_id)
{
  
  // working memory changes
  setReceiveXarchChangeNotifications(true);

  m_queueBehaviour = cdl::QUEUE;

  SetIdentity(katana_base);
  curGoal = GOAL_IDLE;
  curObjId = "";
  curObj = 0;
  curROI = 0;
  curCmdId = "";
  curActionId = "";
  stat_info.man = this;

}

Manipulator::~Manipulator()
{
  ExitVision();
  ExitController();
  delete curObj;
  delete curROI;
}

/**
 * Options are:
 * --katanapose posefile .. pose of katana arm base w.r.t. world origin
 *                          e.g. instantiations/bham/katana-20071113.pose
 */
void Manipulator::configure(map<string,string> & _config)
{
  // first let the base class configure itself
  VideoClientProcess::configure(_config);
  
  if(!_config["--katanapose"].empty())
    ReadPose(_config["--katanapose"].c_str(), katana_base);

  InitController();
  InitVision();
}

void Manipulator::taskAdopted(const string &_taskID)
{
  /* TODO: right now we don't care about tasks
  if(_taskID == curTaskId)
  {
    //ExecuteX();
    taskComplete(_taskID, cdl::PROCESSING_COMPLETE_SUCCESS);
    curTaskId = "";
  }
  else
  {
    log("task " + _taskID + " is not my current task, ignore");
  }*/
}

void Manipulator::taskRejected(const string &_taskID)
{
  /*curTaskId = "";*/
}

void Manipulator::runComponent()
{
  RegisterAction("move");

  ClearGoals();
  SetGoal(GOAL_IDLE);
  while(m_status == STATUS_RUN)
  {
    lockProcess();
    // blocks until new sensor data available, typically 100 ms
    GetCurrentSensors();
    GetCurrentTCP();
    UpdateWorkingMemory();
    // if watched goal is reached
    if(WatchGoal())
    {
      SetGoal(nextGoals[curGoal]);
      ExecuteGoal(curGoal);
    }
    unlockProcess();
    //StateChanged();  // HACK: disable for now until gui crashes solved
    // HACK: should not be needed! make sure others get chance to run
    sleepProcess(100);
  }
}

void Manipulator::start() {

  VideoClientProcess::start();

  // MHN:
  addChangeFilter(createLocalTypeFilter<planning::autogen::Action>(cdl::ADD),
		  new MemberFunctionChangeReceiver<Manipulator>(this,
								&Manipulator::NewAction));
  
  addChangeFilter(createLocalTypeFilter<StopCmd>(cdl::ADD),
		  new MemberFunctionChangeReceiver<Manipulator>(this,
								&Manipulator::StopCurrentBehaviour));

  addChangeFilter(createLocalTypeFilter<PauseCmd>(cdl::ADD),
		  new MemberFunctionChangeReceiver<Manipulator>(this,
								&Manipulator::PauseCurrentBehaviour));
  
  addChangeFilter(createLocalTypeFilter<ResumeCmd>(cdl::ADD),
		  new MemberFunctionChangeReceiver<Manipulator>(this,
								&Manipulator::ResumePausedBehaviour));

  // After initialization, send a command to move the arm to a steady
  // out-of-view position, same as the one used to after an action is
  // aborted...
  std::vector<double> joint_pos;
  joint_pos.clear();
  joint_pos.push_back( 2.09 );   // 120 deg
  joint_pos.push_back( -1.57 );  // -90 deg
  joint_pos.push_back( 0.5235 ); // 30 deg
  joint_pos.push_back( 0.7854 );   // 45 deg
  joint_pos.push_back( -2.618 );   // -150 deg
  kat_clnt->MoveJointPositions( joint_pos );

  // nah: wait until target has been reached.
  while(!TargetReached()) {
    sleepProcess(100);
  }

}


void Manipulator::InitController()
{
  kat_clnt = new KatanaClient();
  kat_clnt->GetSensors(sensors_zero);
}

void Manipulator::ExitController()
{
  delete kat_clnt;
}

/**
 * -p posefile .. pose file for camera/hand calibration
 */
void Manipulator::InitVision()
{
  bufvid = new BufferVideo();
  vcore = new VisionCore(bufvid);
  // manually enable the gestalt principles we need, saves loading a config
  // file, therefore less configuration issues
  vcore->EnableGestaltPrinciple(GestaltPrinciple::FORM_SEGMENTS);
  vcore->EnableGestaltPrinciple(GestaltPrinciple::FORM_LINES);
  vcore->EnableGestaltPrinciple(GestaltPrinciple::FORM_JUNCTIONS);
  vcore->EnableGestaltPrinciple(GestaltPrinciple::FORM_CLOSURES);
}

void Manipulator::ExitVision()
{
  // note: core also deletes video
  delete vcore;
}

/**
 * Blocks until new sensor data available, typically 100 ms.
 */
void Manipulator::GetCurrentSensors()
{
  stat_info.Doing("get katana sensors");
  kat_clnt->GetSensors(sensors_cur);
  stat_info.Done();
}

/**
 * Calculate tool center pose from current joints.
 */
void Manipulator::GetCurrentTCP()
{
  vector<double> pose6D(6);
  Pose3D tcp_wrt_base;

  stat_info.Doing("get katana TCP");
  kat_clnt->GetTCP(pose6D);
  stat_info.Done();

  // tcp_base is the TCP w.r.t katana base
  tcp_wrt_base.m_position.m_x = pose6D[0];
  tcp_wrt_base.m_position.m_y = pose6D[1];
  tcp_wrt_base.m_position.m_z = pose6D[2];
  tcp_wrt_base.m_orientation.m_x = pose6D[3];
  tcp_wrt_base.m_orientation.m_y = pose6D[4];
  tcp_wrt_base.m_orientation.m_z = pose6D[5];
  // transform that to the TCP w.r.t. world origin
  tcp_cur = TransformPoseToGlobal(katana_base, tcp_wrt_base);
}

bool Manipulator::TargetReached()
{
  stat_info.Doing("check katana target reached");
  return kat_clnt->TargetReached();
  stat_info.Done();
}

/**
 * Return whether tactile sensors change more than given threshold
 * over given base.
 * If symmetric is true, then both fingers are required to be above threshold
 * (AND), otherwise at least one finger is required to be above threshold (OR).
 * TODO: This function is not very nice as it is. Eventually a much finer
 * grained information about sensor states will be required.
 */
static int TactileChange(vector<int> &base, vector<int> cur, int thr,
  bool symmetric)
{
  int d_front_left = cur[TACT_SENS_FRONT_LEFT] - base[TACT_SENS_FRONT_LEFT];
  int d_front_right = cur[TACT_SENS_FRONT_RIGHT] - base[TACT_SENS_FRONT_RIGHT];
  int d_back_left = cur[TACT_SENS_BACK_LEFT] - base[TACT_SENS_BACK_LEFT];
  int d_back_right = cur[TACT_SENS_BACK_RIGHT] - base[TACT_SENS_BACK_RIGHT];
  // if any left sensor and any right sensor register (sufficient) pressure,
  // we are holding an object
  int d_left = max(d_front_left, d_back_left);
  int d_right = max(d_front_right, d_back_right);
  if(symmetric)
  {
    if(d_left >= thr && d_right >= thr)
      return (d_left + d_right)/2;
    else
      return 0;
  }
  else
  {
    if(max(d_left, d_right) >= thr)
      return max(d_left, d_right);
    else
      return 0;
  }
}

/**
 * If tactile sKatArm::SensorDataSetensors of both fingers register pressure,
 * we are holding an object.
 * Returns 0 if not holding, else the average pressure, i.e. the
 * "firmness" of the grip.
 */
int Manipulator::HoldingObject()
{
  // closing gripper stops as soon as _one_ sensor reaches threshold.
  // So if we check for _both_ fingers reaching threshold, we might get to
  // the situation that the gripper is closed and stopped and we do not sense
  // a held object! -> set symmetric to false.
  return TactileChange(sensors_zero, sensors_cur, grasp_firm_thr, false);
}

/**
 * Mid-movment collision detection of carried object.
 * If while carrying an object tactile sensors change, the carried object
 * is bumping into something.
 */
int Manipulator::CarriedObjectCollides()
{
  return TactileChange(sensors_hold, sensors_cur, grasp_collision_thr, false);
}

/**
 * Sense touchdown of carried object,
 * Similar to mid-movement collision detection, but with a more sensitive
 * threshold.
 */
int Manipulator::CarriedObjectTouches()
{
  return ObjectSlidesBetweenFingers() ||
    TactileChange(sensors_hold, sensors_cur, grasp_putdown_thr, false);
}

/**
 * Whenever any inner proximity sensor registers, we have an object
 * between fingers. Or when the tactile sensors register that we are already
 * colliding.
 */
bool Manipulator::ObjectBetweenFingers()
{
  int near = 0;

  /* for now don't use the foward pointing sensors, they react too early
     near = max(near,
     sensors_cur[sensor_idx[PROX_SENS_FRONT_LEFT]].value -
     sensors_zero[sensor_idx[PROX_SENS_FRONT_LEFT]].value);
     near = max(near,
     sensors_cur[sensor_idx[PROX_SENS_FRONT_RIGHT]].value -
     sensors_zero[sensor_idx[PROX_SENS_FRONT_RIGHT]].value);*/
  near = max(near,
             sensors_cur[PROX_SENS_MID_LEFT] -
             sensors_zero[PROX_SENS_MID_LEFT]);
  near = max(near,
             sensors_cur[PROX_SENS_MID_RIGHT] -
             sensors_zero[PROX_SENS_MID_RIGHT]);
  near = max(near,
             sensors_cur[PROX_SENS_CENTER] -
             sensors_zero[PROX_SENS_CENTER]);
  return near >= prox_near_thr ||
    TactileChange(sensors_zero, sensors_cur, grasp_putdown_thr, false);
}

/**
 * Check if grasped object slides towards the center proximity sensor,
 * e.g. during put-down.
 */
bool Manipulator::ObjectSlidesBetweenFingers()
{
  int d = sensors_cur[PROX_SENS_CENTER] - sensors_hold[PROX_SENS_CENTER];
  return d >= prox_slide_thr;
}

/**
 * The actual arm movment command.
 * Finds a collision free path and starts following it.
 * If wait is true, waits for completion of movement. Else returns
 * immediately.
 */
bool Manipulator::MoveArmTo(Pose3D &pose)
{
  bool ret = false;
  vector<double> tcp(6);
  Pose3D tcp_wrt_base;

  // HACK: don't allow positions lower than the Katatanas own ground plane
  if( pose.m_position.m_z < LOWEST_TCP_Z ) { // 0.05;
    pose.m_position.m_z = LOWEST_TCP_Z;
  }
  // HACK END

  // push to goal history
  tcp_goals.push_front(pose);
  if(tcp_goals.size() > GOAL_HISTORY_SIZE)
    tcp_goals.pop_back();

  // pose is the TCP w.r.t. world origin, transform that to TCP w.r.t. katana
  // base
  tcp_wrt_base = TransformPoseToLocal(katana_base, pose);
  tcp[0] = tcp_wrt_base.m_position.m_x;
  tcp[1] = tcp_wrt_base.m_position.m_y;
  tcp[2] = tcp_wrt_base.m_position.m_z;
  tcp[3] = tcp_wrt_base.m_orientation.m_x;
  tcp[4] = tcp_wrt_base.m_orientation.m_y;
  tcp[5] = tcp_wrt_base.m_orientation.m_z;

  // Get current gripper values...
  std::cout << "\nObject Grasped = " << ObjectInGrasp() << "\n";

  stat_info.Doing("katana move");
  ret = kat_clnt->MoveTCP(tcp);
  stat_info.Done();
  if(!ret)
    log("arm controller failed to plan path");

  return ret;
}

void Manipulator::StopArm()
{
  stat_info.Doing("katana stop arm");
  kat_clnt->StopArm();
  stat_info.Done();
}

void Manipulator::StopGripper()
{
  stat_info.Doing("katana stop gripper");
  kat_clnt->StopGripper();
  stat_info.Done();
}

void Manipulator::CloseGripper()
{
  stat_info.Doing("katana close gripper");
  kat_clnt->CloseGripper(grasp_firm_thr);
  stat_info.Done();
}

void Manipulator::OpenGripper()
{
  stat_info.Doing("katana open gripper");
  kat_clnt->OpenGripper();
  stat_info.Done();
}

/**
 * Add a visual working memory scene object to the controllers obstacle map.
 */
void Manipulator::AddObstacle(const string &objId)
{
//   shared_ptr<const CASTData<SceneObject> > oobj =
//     getWorkingMemoryEntry<SceneObject>( objId,"vision.sa");
//   if(oobj != 0)
//   {
//     const SceneObject *obj = oobj->data();
//   }
}

void Manipulator::MoveObstacle(const string &objId)
{
  RemoveObstacle(objId);
  AddObstacle(objId);
}

void Manipulator::RemoveObstacle(const string &objId)
{
}

bool Manipulator::GetCamera(Camera &cam)
{
  string vis_subarch = "vision.sa";
  vector <shared_ptr<const CASTData<Camera> > > blorgs;

  getWorkingMemoryEntries(vis_subarch, 0, blorgs);
 
  bool found = false;
  for(size_t i = 0; i < blorgs.size() && !found; i++)
  {
    if(blorgs[i]->getData()->m_num == CAM_ARM)
    {
      cam = *(blorgs[i]->getData());
      found = true;
    }
  }
  return found;
}

static void DrawPoly(IplImage *img, const vector<Vector2D> &pts, CvScalar col)
{
  for(unsigned i = 0; i < pts.size(); i++)
  {
    int j = (i < pts.size() - 1 ? i + 1 : 0);
    cvLine(img, cvPoint((int)pts[i].m_x, (int)pts[i].m_y),
        cvPoint((int)pts[j].m_x, (int)pts[j].m_y), col);
  }
}

static bool RectangleSidesParallel(Rectangle *r)
{
  static const double phi_max = 0.3;  // parallelity threshold
  Vector2 l[4];           // sides
  double phi0_2, phi1_3;  // angles between opposing sides
  int i, j;

  // normalise might throw an exception
  try
  {
    // l[i] is the line joining i and i+1
    for(i = 0; i < 4; i++)
    {
      j = (i < 3 ? i + 1 : 0);
      l[i] = LJunctions(r->jcts[j])->isct - LJunctions(r->jcts[i])->isct;
      l[i].Normalise();
    }
    phi0_2 = fabs(asin(Cross(l[0], l[2])));
    phi1_3 = fabs(asin(Cross(l[1], l[3])));
    return phi0_2 < phi_max && phi1_3 < phi_max;
  }
  catch(...)
  {
    return false;
  }
}


/**
 * Function computesif the input rectangle's aspect ratio (largest
 * length/smallest length) is above a certain minimum threshold, and
 * below a certain maximum threshold...
 */
static bool RectangleValidAspectRatio(Rectangle* r, double minLWThresh, 
				      double maxLWThresh) {
  Vector2 l[4];   // sides
  double minLen = 1e7;
  double maxLen = 2.;
  
  int i, j;
  // l[i] is the line joining i and i+1
  for(i = 0; i < 4; ++i) {
    j = (i < 3 ? i + 1 : 0);
    l[i] = LJunctions(r->jcts[j])->isct - LJunctions(r->jcts[i])->isct;
    maxLen = max(maxLen, Length(l[i]));
    minLen = min(minLen, Length(l[i]));
  }
  
  std::cout << " RAR: " << maxLen/minLen << " ";
  return( ( maxLen/minLen >= minLWThresh ) && 
	  ( maxLen/minLen <= maxLWThresh ) );
}


static bool RectangleBigEnough(Rectangle *r)
{
  static const double len_min = 20.;  // min length in pixels // 40.
  Vector2 l[4];   // sides
  double len = 0.;
  int i, j;

  // l[i] is the line joining i and i+1
  for(i = 0; i < 4; i++)
    {
      j = (i < 3 ? i + 1 : 0);
      l[i] = LJunctions(r->jcts[j])->isct - LJunctions(r->jcts[i])->isct;
      len = max(len, Length(l[i]));
    }
  return len > len_min;
}


static bool RectangleAreaThresh(Rectangle *r, double maxAreaThresh, Vector2D bboxSize) {
  Vector2 l[4];   // sides
  double minLen = 1e7;
  double maxLen = 2.;
  
  int i, j;
  // l[i] is the line joining i and i+1
  for( i = 0; i < 4; ++i ) {
    j = (i < 3 ? i + 1 : 0);
    l[i] = LJunctions(r->jcts[j])->isct - LJunctions(r->jcts[i])->isct;
    maxLen = max(maxLen, Length(l[i]));
    minLen = min(minLen, Length(l[i]));
  }
  double rectArea = maxLen * minLen;
  double bboxArea = bboxSize.m_x * bboxSize.m_y;
  std::cout << "RAT: " << rectArea << ", " << bboxSize.m_x << ", " << bboxSize.m_y
	    << ", " << maxAreaThresh * bboxArea << endl;
  return( rectArea <= maxAreaThresh * bboxArea ); 
}


/**
 * Return rectangle center and axis in 3D.
 *
 * I am too stupid to calculate moments and therefore in dire need of a
 * thorough lobotomy. Use the midpoints-of-side method instead (C) Aaron.
 for(i = 0; i < 4; i++)
 m11 += ((x[i] - xm)*(y[i] - ym));
 m11 /= 4.;

 for(i = 0; i < 4; i++)
 m20 += pow(x[i] - xm, 2.);
 m20 /= 4.;

 for(i = 0; i < 4; i++)
 m02 += pow(y[i] - ym, 2.);
 m02 /= 4.;

 // orientation of strongest eigenvector of covariance matrix:
 // angle is relative to image x axis
 th = 0.5*atan(2.*m11/(m20 - m02));
 // we want things vertical, i.e. angle ralative to y axis
 grasp_angle = th - M_PI/2;
*/
static void RectangleMainAxis(const vector<Vector3D> &points, Vector3D &center,
    Vector3D &axis)
{
  // for filtering the axis direction of surfaces: axis direction of previously
  // found surface
  /*static Vector3D axis_prev;
  static bool have_prev = false;*/
  Vector3D l[4];  // sides
  Vector3D a, b;

  center = Center(points);
  // l[i] is the line joining i and i+1
  for(unsigned i = 0; i < 4; i++)
  {
    unsigned j = (i < 3 ? i + 1 : 0);
    l[i] = points[j] - points[i];
  }
  // find the midpoints of the two shorter sides:
  // if l[0] and l[2] are the short sides
  if(Length(l[0]) < Length(l[1]))
  {
    a = (points[0] + points[1])/2.;
    b = (points[2] + points[3])/2.;
  }
  // else l[1] and l[3] are the short sides
  else
  {
    a = (points[1] + points[2])/2.;
    b = (points[3] + points[0])/2.;
  }
  axis = Normalise(b - a);
  // Note that the axis direction is ambiguous. To avoid unnecessary arm
  // rotations (especially annoying during approach) make the current axis
  // point in the same principal direction as the previously found axis.
  /*if(have_prev)
  {
    if(Dot(axis, axis_prev) < 0.)
      axis = -axis;
  }
  axis_prev = axis;
  have_prev = true;*/
}

/**
 * Find the "best" rectangle.
 * Rectangles are first filtered according to size (large enough), roughly
 * parallel sides. Then the rectangle neariest to image center is selected.
 * If this rectangle is near to the previously found rectangle return true,
 * otherwise false.
 * TODO: this procedure is rather hacky, but for now it will have to do.
 */
bool Manipulator::FindRectangle(ImageFrame *img,
    vector<Vector2D> &points_prev, vector<Vector2D> &points)
{
  const int TIME_STEP = 20;   // time step for incremental grouping in [ms]
  const int MAX_TIME = 200;   // max time allowed for image processing in [ms]
  const double RECT_POS_THR = 4.;  // threshold for "same location"
  int elapsed_time = 0;       // time spent so far on image processing
  // double min_dist = HUGE;
  bool ret = false;
  vector<Rectangle*> rects;
  Rectangle *rect_best = 0;
  static IplImage *out = 0;  // TODO: for debugging only

  // TODO: for debugging only
  if(out == 0)
    out = cvCreateImage(cvSize(img->m_width, img->m_height), IPL_DEPTH_8U, 3);
  memcpy(out->imageData, (char*)&(img->m_image[0]), img->m_image.length());

  // get framework image into our video
  // HACK: taking the address of a CORBA sequence is dirty, but seems to work
  bufvid->Init(img->m_width, img->m_height, BGR24, (char*)&(img->m_image[0]),
      false);
  vcore->NewImage();

  if(INCREMENTAL_IMAGE_PROC)
  {
    // spend some time on image processing
    while(elapsed_time < MAX_TIME)
    {
      vcore->ProcessImage(TIME_STEP); // TODO: crashes
      // note that the actual time spent in image processing might differ from
      // the time specified, we don't care here, rough times are OK
      elapsed_time += TIME_STEP;

      // TODO: if we found a rectangle at the previous position, we can stop
      //  earlier
    }
  }
  else
  {
    vcore->ProcessImage();
  }

  std::cout << "\n";

  // filter rectangles:
  // accept only big enough rectangles with roughly parallel sides
  for(unsigned i = 0; i < NumRectangles(); i++) {
    Rectangle *r = Rectangles(RankedGestalts(Gestalt::RECTANGLE, i));
    vector<Vector2D> tmp_points2(4);
    for(unsigned j = 0; j < 4; j++) {
      tmp_points2[j].m_x = LJunctions(r->jcts[j])->isct.x;
      tmp_points2[j].m_y = LJunctions(r->jcts[j])->isct.y;
    }
    // TODO: for debugging only, draw in green...
    DrawPoly(out, tmp_points2, CV_RGB(0, 255, 0));

    if( RectangleSidesParallel(r) && RectangleBigEnough(r) && 
	r->IsUnmasked() && RectangleValidAspectRatio(r, 1.5, 4.5) &&
	RectangleAreaThresh(r, 0.75, curROI->m_bbox.m_size) ) {
      rects.push_back(r);
      std::cout << "MNP: findrectangles, rect " << i << " accepted...\n";
    }
    else {
      std::cout << "MNP: findrectangles, rect " << i << " rejected: " 
		<< RectangleSidesParallel(r) << "," << RectangleBigEnough(r) 
		<< "," << r->IsUnmasked() << "," 
		<< RectangleValidAspectRatio(r, 2.0, 4.0) << ","
		<< RectangleAreaThresh(r, 0.75, curROI->m_bbox.m_size) << endl;
    }
  }
  
  std::cout << "MNP: findrectangles, Number of valid rectangles " 
	    << NumRectangles() << ", " << rects.size() << endl;
  
  // Find valid rectangle that projects closest to the actual object
  // center (in 3D space)...
  Camera cam;
  GetCamera(cam);
  Vector2D top_surface_center;
  Vector3D curObj_centroid = curObj->m_bbox.m_centroid;
  double min_projection_object_dist = IMG_PROJECT_3D_OBJECT_DIST_THRESH;
  // For each valid rectangle...
  for( size_t i = 0; i < rects.size(); ++i ) {
    top_surface_center.m_x = 0.0;
    top_surface_center.m_y = 0.0;
    vector<Vector2D> tmp_points(4);

    for( size_t j = 0; j < 4; ++j ) {
      top_surface_center.m_x += LJunctions(rects[i]->jcts[j])->isct.x;
      top_surface_center.m_y += LJunctions(rects[i]->jcts[j])->isct.y;

      tmp_points[j].m_x = LJunctions(rects[i]->jcts[j])->isct.x;
      tmp_points[j].m_y = LJunctions(rects[i]->jcts[j])->isct.y;
    }
    top_surface_center.m_x /= 4.;
    top_surface_center.m_y /= 4.;
    Vector3D ground_point = ProjectImagePointToGroundplane( GROUND_HEIGHT, 
							    cam, top_surface_center );
    // Select rectangle if it is within a distance of the actual
    // centroid of the 3D bbox of the object that triggered the
    // manipulation call -- remember to include the weird x-axis
    // offset...
    double projection_object_dist = sqrt( pow(ground_point.m_x - (curObj_centroid.m_x+ZERO_OFFSET_X), 2) +
					  pow(ground_point.m_y - curObj_centroid.m_y, 2) );
    if( projection_object_dist < min_projection_object_dist ) {
      min_projection_object_dist = projection_object_dist;
      rect_best = rects[i];
      DrawPoly(out, tmp_points, CV_RGB(0, 0, 255));
      drawBox3D( ground_point.m_x, ground_point.m_y, ground_point.m_z, 
		 0.01, 0.01, 0.01, 0, 255, 0,  0 );
    }
    else {
      std::cout << "MNP: findrectangles, rejected rect because dist = " 
		<< projection_object_dist << endl;
    }
  }

  // if we found a rectangle at all
  if(rect_best != 0) {
    std::cout << "MNP: findrectangles, best rect found...\n";
    points.resize(4);
    for(unsigned j = 0; j < 4; j++) {
      points[j].m_x = LJunctions(rect_best->jcts[j])->isct.x;
      points[j].m_y = LJunctions(rect_best->jcts[j])->isct.y;
    }
    // filter position: check if it falls on the same position as last image
    if(Length(Center(points_prev) - Center(points)) < RECT_POS_THR) {
      std::cout << "MNP: findrectangles, best rect position valid...\n";
      ret = true;
    }
    else {
      std::cout << "MNP: findrectangles, best rect position changed...\n";
    }

    points_prev = points;
    // TODO: for debugging only, draw in red
    DrawPoly(out, points, CV_RGB(255, 0, 0));
  }
  else {
    std::cout << "MNP: findrectangles, no best rect found...\n";
  }
  
  // TODO: for debugging, move into GUI
  //cvShowImage("top surface", out);
  //cvWaitKey(10);
  static int j = 0;
  char filename[256];
  sprintf(filename, "manipTest%04d.ppm", j++);
  std::cout << "Saving image " << filename << endl;
  cvSaveImage(filename, out);

  return ret;
}



/**
 * A somewhat robust method to find the "best" rectangle.
 * Just try several times and until we get a consistent result.
 */
bool Manipulator::FindRectangleRobust(vector<Vector2D> &points)
{
  const int TRIALS = 10;  // number of trials to find a rectangle
  const int RECT_FOUND_MIN = 3; // require the top rectangle to be found e.g. 3
                                // times in the same location
  int rect_found = 0;
  vector<Vector2D> points_prev;

  points_prev.resize(4);
  for(unsigned i = 0; i < points_prev.size(); i++)
    points_prev[i].m_x = points_prev[i].m_y = 0.;

  std::cout << "\n MNP: FindRectRobust, the fun starts now...\n";
  // get rectangle position and orientation
  // try several times until a "stable" position is found
  for(int i = 0; i < TRIALS && rect_found < RECT_FOUND_MIN; i++) {
    //ImageFrame *img = GetImage(CAM_ARM);
    ImageFrame img;
    getImage(CAM_ARM, img);

    stat_info.img = img;
    stat_info.have_img = true;
    if(FindRectangle(&img, points_prev, points)) {
      rect_found++;
    }
    else {
      std::cout << "MNP: FindRectRobust, findrect failed for trial " 
		<< i << endl;
    }
  }
  if(rect_found >= RECT_FOUND_MIN) {
    stat_info.rect2_points = points;
    stat_info.have_rect2 = true;
  }
  return rect_found >= RECT_FOUND_MIN;
}



/**
 * Find point on z = height plane parellel to the groundplane for a given image
 * point.
 * @param cam  camera parameters
 * @param height  height (z-value) of the plane
 * @param p  image point in pixel coordinates
 * Returns 3D point on ground plane, with z = 0.
 * Note: Does not do image undistortion!
 */
static Vector3D ProjectImagePointToPlane(const Camera &cam, double height,
    const Vector2D &p) throw(BALTException)
{
  Vector3D e;  // eye point = origin of line of sight [m]
  Vector3D d;  // direction of line of sight [m]
  Vector3D g;  // point on ground plane [m]
  double l;    // length of line of sight until intersection

  // first find direction in camera coordinates
  d.m_x = p.m_x - cam.m_cx;
  d.m_y = p.m_y - cam.m_cy;
  // note: i assume that fx is (nearly) equal to fy, so it does not matter
  // whether we use fx or fy
  d.m_z = cam.m_fx;
  // up to now d is in [pixel], normalise to 1 [m]
  d = Normalise(d);

  // transform dir and eye point to world coordinates
  d = TransformDirToGlobal(cam.m_pose, d);
  e = cam.m_pose.m_position;

  // find intersection with z = h plane
  if(fpclassify(d.m_z) == FP_ZERO)
    throw BALTException(__HERE__, "divison by 0");
  l = (height - e.m_z)/d.m_z;
  g.m_x = e.m_x + l*d.m_x;
  g.m_y = e.m_y + l*d.m_y;
  g.m_z = height;

  return g;
}

/**
 * Project rectangle from image to base frame.
 * @param cam  camera parameters
 * @param img_points  points in 2D image co-ordinates (z = 0)
 * @param cam_points  points in 3D camera co-ordinates, projected to plane
 *                    parallel to groundplane at z = height
 */
void Manipulator::ProjectRectangleToBase(const Camera &cam, double height,
    const vector<Vector2D> &img_points, vector<Vector3D> &cam_points)
{
  cam_points.resize(img_points.size());
  for(unsigned i = 0; i < img_points.size(); i++)
    cam_points[i] = ProjectImagePointToPlane(cam, height, img_points[i]);
}

const char* Manipulator::GoalName(ArmGoal goal)
{
  if(goal >= 0 || goal < GOAL_UNDEF)
    return goalnames[goal];
  else
    return goalnames[GOAL_UNDEF];
}

void Manipulator::ClearGoals()
{
  nextGoals.clear();
  nextGoals[GOAL_IDLE] = GOAL_IDLE;
}


bool Manipulator::ObjectInGrasp() {
  kat_clnt->GetGripperEncoderValues(gripper_encoder);
//   std::cout << "****\n\n Gripper vals=(" << gripper_encoder.at(0) 
// 	    << ", " << gripper_encoder.at(1) << ", " << gripper_encoder.at(2)
// 	    << ")\n";
  if( gripper_encoder.at(2) < OBJECT_GRASPED_ENCODER_THRESH_HIGH &&
      gripper_encoder.at(2) > OBJECT_GRASPED_ENCODER_THRESH_LOW ) {
    return true;
  } else {
    return false;
  }
}


/*
 * Setup sequence of goals for pick and place.
 * Pick and place of object to target position consists of:
 * 1. move above object
 * 2. refine above pose using handy-cam
 * 3. move to grasp position
 * 4. grasp object
 * 5. lift object
 * 6. move above target position
 * 7. put down = move down until contact with ground plane
 * 8. release object
 * 9. move away from object (so that next movement won't knock over object)
 *    Actually collision free path planning should ensure that, but it
 *    doesn't harm to make sensible movments in the first place.
 */
bool Manipulator::InitiatePickAndPlaceBehaviour(const string &cmdId)
{
  shared_ptr<const CASTTypedData<PickAndPlaceCmd> > ccmd;
  try {
    ccmd = getWorkingMemoryEntry<PickAndPlaceCmd>(cmdId);
  }
  catch(const DoesNotExistOnWMException & e) {
    log("PickAndPlaceCmd missing, aborting : %s",e.what());
    return false;
  }

  
  // remember command id for later signalling completion
  curCmdId = cmdId;
  // get details of command
  curObjId = string(ccmd->getData()->m_objectPointer.m_address.m_id);
  string curObjSA(ccmd->getData()->m_objectPointer.m_address.m_subarchitecture);
  curPutdownPose = ccmd->getData()->m_targetPose;

  // MHN: Offset the x-axis position to account for the weird offset
  // in the coordinate frames...
  curPutdownPose.m_position.m_x += ZERO_OFFSET_X;

  log("InitiatePickAndPlaceBehaviour(): putdown (%f,%f,%f)", curPutdownPose.m_position.m_x,
      curPutdownPose.m_position.m_y, curPutdownPose.m_position.m_z);
  
  // get a local copy of the current object
  // Note that this is necesary, as movement of the robot itself will
  // trigger change and new segmentation, leading to removal of old
  // and addition of new objects in visual WM, so curObjId becomes invalid.  
  // TODO: explicit string "vision.sa" is nasty!
  shared_ptr<const CASTTypedData<SceneObject> > oobj;
  try {    
    oobj = getWorkingMemoryEntry<SceneObject>(curObjId, curObjSA);
  }
  catch(const DoesNotExistOnWMException & e) {
    log("SceneObject missing, aborting : %s",e.what());
    return false;
  }

  
  if(curObj) {
    delete curObj;
  }
  curObj = new SceneObject(*oobj->getData());

  if( curROI ) {
    delete curROI;
  }
  
  //nah: new edit, sometimes the ROI is missing?!
  curROI = NULL;

  // Get the ROI corresponding to the sceneobject being processed...
  string vis_subarch("vision.sa");
  assert(curObj->m_ROIsMemoryIDs.length() > 0);
  string roiID(curObj->m_ROIsMemoryIDs[0]);

  while(curROI == NULL) {
    try {
      shared_ptr<const CASTData<ROI> > curROIData = 
	getWorkingMemoryEntry<Vision::ROI>(roiID, vis_subarch);
      //   shared_ptr< const ROI > shareCurROI = curROIData->getData();
      //   *curROI = *shareCurROI;
      curROI = new Vision::ROI( *curROIData->getData() );
    }
    catch(const DoesNotExistOnWMException & e) {
      log("ROI was missing, trying again: %s",e.what());
      sleepProcess(100);
    }
  }

  // ignore any changes in the visual working memory due to the robots
  // own movement
  ignore_new_objects = true;


  //now tell vision not to do anything
  inhibitSceneProcessing();

  // set up sequence of goals
  nextGoals.clear();
  nextGoals[GOAL_IDLE] = GOAL_IDLE;
  nextGoals[GOAL_MOVE_ABOVE_OBJ] = GOAL_REFINE_ABOVE_OBJ;
  nextGoals[GOAL_REFINE_ABOVE_OBJ] = GOAL_APPROACH_OBJ;
  nextGoals[GOAL_APPROACH_OBJ] = GOAL_CLOSE_GRASP;
  nextGoals[GOAL_CLOSE_GRASP] = GOAL_LIFT_OBJ;
  nextGoals[GOAL_LIFT_OBJ] = GOAL_CARRY_OBJ;
  nextGoals[GOAL_CARRY_OBJ] = GOAL_PUTDOWN_OBJ;
  nextGoals[GOAL_PUTDOWN_OBJ] = GOAL_RELEASE_GRASP;
  nextGoals[GOAL_RELEASE_GRASP] = GOAL_RETREAT_OBJ;
  nextGoals[GOAL_RETREAT_OBJ] = GOAL_PNP_DONE;
  nextGoals[GOAL_PNP_DONE] = GOAL_IDLE;
  SetGoal(GOAL_MOVE_ABOVE_OBJ);
  ExecuteGoal(curGoal);
  return true;
}

void Manipulator::NewPickAndPlaceCmd(const WorkingMemoryChange &cmdId) {
  // only if we have no other goals set right now
  if(curGoal == GOAL_IDLE) {
    log("Initiating pick and place command in NewPickAndPlaceCmd()...");
    std::cout << "Initiating pick and place command in NewPickAndPlaceCmd()...\n";
  }
  if(!InitiatePickAndPlaceBehaviour(string(cmdId.m_address.m_id)))
    AbortPickAndPlace();
}


// MHN:
void Manipulator::NewAction(const WorkingMemoryChange &cmdId) {

  //store action id for signalling
  curActionId = string(cmdId.m_address.m_id);

  // only if we have no other goals set right now
  if(curGoal == GOAL_IDLE) {

    //get the action stuct
    shared_ptr<const CASTTypedData<planning::autogen::Action> > actionData =
      getWorkingMemoryEntry<planning::autogen::Action>(curActionId);
  
    //we can only do one thing
    assert(strcmp(actionData->getData()->m_action.m_type, typeName<PickAndPlaceCmd>().c_str()) == 0);

    string pnpID(actionData->getData()->m_action.m_address.m_id);
    log("Initiating pick and place command in NewAction()...");
    std::cout << "Initiating pick and place command in NewAction()...\n";

    if(!InitiatePickAndPlaceBehaviour(pnpID)) {
      AbortPickAndPlace();
    }
  }
  else {
    ExecutePnpDone(false);
  }
}



void Manipulator::StopCurrentBehaviour(const WorkingMemoryChange &cmdId)
{
  stoppedGoal = curGoal;
  nextGoals[GOAL_IDLE] = GOAL_IDLE;
  SetGoal(GOAL_IDLE);
  if(stoppedGoal != GOAL_IDLE)
    AbortPickAndPlace();
  StopArm();
  StopGripper();
}

void Manipulator::PauseCurrentBehaviour(const WorkingMemoryChange &cmdId)
{
  // note: multiple pauses are ignored
  if(curGoal != GOAL_PAUSE)
  {
    pausedGoal = curGoal;
    nextGoals[GOAL_IDLE] = GOAL_IDLE;
    nextGoals[GOAL_PAUSE] = GOAL_PAUSE;
    SetGoal(GOAL_PAUSE);
    StopArm();
    StopGripper();
  }
}

void Manipulator::ResumePausedBehaviour(const WorkingMemoryChange &cmdId)
{
  //deleteFromWorkingMemory(cmdId);  TODO: why does not work?
  // note: multiple resumes are ignored
  if(curGoal == GOAL_PAUSE)
  {
    SetGoal(pausedGoal);
    // re-execute the paused goal (probably it was interrupted in mid-execution)
    ExecuteGoal(curGoal);
  }
}

void Manipulator::AbortPickAndPlace()
{
  ExecutePnpDone(false);
  SetGoal(GOAL_IDLE);
}

void Manipulator::ExecuteGoal(ArmGoal goal)
{
  switch(goal)
    {
    case GOAL_MOVE_ABOVE_OBJ:
      ExecuteMoveAboveObj();
      break;
    case GOAL_REFINE_ABOVE_OBJ:
      ExecuteRefineAboveObj();
      break;
    case GOAL_APPROACH_OBJ:
      ExecuteApproachObj();
      break;
    case GOAL_CLOSE_GRASP:
      ExecuteGrasp();
      break;
    case GOAL_LIFT_OBJ:
      if( ObjectInGrasp() ) {
	ExecuteLiftObj();
      } else {
	std::cout << "\n Object not in grasp -- aborting without lifting... \n";
	AbortPickAndPlace();
      }
      break;
    case GOAL_CARRY_OBJ:
      if( ObjectInGrasp() ) {
	ExecuteCarryObj();
      } else {
	std::cout << "\n Object not in grasp -- aborting without carrying... \n";
	AbortPickAndPlace();
      }
      break;
    case GOAL_PUTDOWN_OBJ:
      ExecutePutdownObj();
      break;
    case GOAL_RELEASE_GRASP:
      ExecuteRelease();
      break;
    case GOAL_RETREAT_OBJ:
      ExecuteRetreatObj();
      break;
    case GOAL_PNP_DONE:
      ExecutePnpDone(true);
      break;
    case GOAL_IDLE:
      break;
    case GOAL_PAUSE:
    case GOAL_UNDEF:
    default:
      break;
    }
}

bool Manipulator::WatchGoal()
{
  bool goalReached = false;
  switch(curGoal)
  {
    case GOAL_MOVE_ABOVE_OBJ:
      goalReached = WatchMoving();
      break;
    case GOAL_REFINE_ABOVE_OBJ:
      goalReached = WatchRefineAbove();
      break;
    case GOAL_APPROACH_OBJ:
      goalReached = WatchApproach();
      break;
    case GOAL_CLOSE_GRASP:
      goalReached = WatchGrasping();
      break;
    case GOAL_LIFT_OBJ:
      goalReached = WatchLiftObj();
      break;
    case GOAL_CARRY_OBJ:
      goalReached = WatchCarryObj();
      break;
    case GOAL_PUTDOWN_OBJ:
      goalReached = WatchPutdownObj();
      break;
    case GOAL_RELEASE_GRASP:
      goalReached = WatchRelease();
      break;
    case GOAL_RETREAT_OBJ:
      goalReached = WatchRetreat();
      break;
    case GOAL_PNP_DONE:
      goalReached = WatchPnpDone();
      break;
    case GOAL_IDLE:
      goalReached = WatchIdle();
      break;
    case GOAL_PAUSE:
    case GOAL_UNDEF:
    default:
      break;
  }
  return goalReached;
}

/**
 * Go to pose safely above the object (the pre-grasp pose), taking into
 * account object size..
 */
void Manipulator::ExecuteMoveAboveObj()
{
  OpenGripper();
  FindPoseAbove(*curObj, curTargetPose);

  // MHN: Slight offset to get the arm a little above the object...
  curTargetPose.m_position.m_z += MOVE_ABOVE_OFFSET_Z;

  // MHN: Offset to account for the discrepancy in the coordinate
  // frames...
  curTargetPose.m_position.m_x += ZERO_OFFSET_X;

  if(!MoveArmTo(curTargetPose))
    AbortPickAndPlace();
}

void Manipulator::ExecuteRefineAboveObj()
{
  if(FindTouchPoseVisual(curTargetPose, true)) {
    curTargetPose.m_position.m_y += MAGIC_GRASP_OFFSET_Y;
    // MHN: Offset to get it closer to the object than move-above but
    // still allowing some leeway to prevent crashing into the
    // object...
    if(!MoveArmTo(curTargetPose)) {
      log("could not move arm to refineaboveobj...");
      AbortPickAndPlace();
    }
  }
  else
  {
    log("refine:failed to find top surface, aborting");
    AbortPickAndPlace();
  }
}

/**
 * Find a good grasp pose.
 * Currently: just try to go _into_ the object. Proximity sensors will stop
 * us anyway.
 */
void Manipulator::ExecuteApproachObj()
{
  if(FindGraspPoseVisual(curTargetPose))
  {
    curTargetPose.m_position.m_y += MAGIC_GRASP_OFFSET_Y;
    if(!MoveArmTo(curTargetPose)) {
      log("could not move arm to executeaboveobj...");
      AbortPickAndPlace();
    }
  }
  else
  {
    log("approach:failed to find top surface, aborting");
    AbortPickAndPlace();
  }
}

void Manipulator::ExecuteGrasp()
{
  CloseGripper();
}

/**
 * Lift the grasped object OFFSET above the current pose.
 */
void Manipulator::ExecuteLiftObj()
{
  curTargetPose = tcp_cur;
  // curTargetPose.m_position.m_z += PRE_GRASP_OFFSET*1.5;

  // MHN: Move lifted object specific fixed distance from ground...
  curTargetPose.m_position.m_z += POST_GRASP_LIFT_OFFSET_Z;

  if(MoveArmTo(curTargetPose))
  {
    // tighten the grip after having started the movement
    // TODO: do proper monitoring of grasp
    for(int i = 0; i < 10; i++)
    {
      CloseGripper();
      usleep(100000);
    }
    // update "hold" sensor values
    GetCurrentSensors();
    sensors_hold = sensors_cur;
  }
  else
    AbortPickAndPlace();
}

/**
 * Go to a pose safely above the put-down pose, taking into account
 * object size..
 */
void Manipulator::ExecuteCarryObj()
{
  // note that the orientation of the target pose is (for now) unspecified.
  // so we find a "good" orientation ourselves
  curTargetPose = curPutdownPose;
//   curTargetPose.m_position.m_z +=
//     (curObj->m_bbox.m_size.m_z/2. + PRE_GRASP_OFFSET*1.5);

  // MHN: Carry lifted object specific fixed distance from ground...
  curTargetPose.m_position.m_z += POST_GRASP_CARRY_OFFSET_Z;

  FindConvenientOrientation(curTargetPose.m_position,
      curTargetPose.m_orientation);

  if(!MoveArmTo(curTargetPose))
    AbortPickAndPlace();
}

/**
 * Go to put-down pose. Tactile sensors will stop us as soon as the grasped
 * object touches ground.
 */
void Manipulator::ExecutePutdownObj()
{
  // note that the orientation of the target pose is (for now) unspecified.
  // so we find a "good" orientation ourselves
  curTargetPose = curPutdownPose;
  // use the height when we first grasped the object as height to put it down
  // (assuming of course that the table is flat)
  curTargetPose.m_position.m_z = grasp_z;
  FindConvenientOrientation(curTargetPose.m_position,
      curTargetPose.m_orientation);
  if(MoveArmTo(curTargetPose))
  {
    // update "hold" sensor values
    GetCurrentSensors();
    sensors_hold = sensors_cur;
  }
  else
    AbortPickAndPlace();
}

void Manipulator::ExecuteRelease()
{
  OpenGripper();
}

void Manipulator::ExecuteRetreatObj()
{
  curTargetPose = tcp_cur;
  curTargetPose.m_position.m_z += POST_GRASP_RETREAT_OFFSET_Z;
  if(!MoveArmTo(curTargetPose))
    AbortPickAndPlace();
}

void Manipulator::SetGoal(ArmGoal newGoal)
{
  log("switching from %s to %s", goalnames[curGoal], goalnames[newGoal]);
  curGoal = newGoal;
}

// MHN:
void Manipulator::ExecutePnpDone(bool success) {
  if(success)
    log("PNP done - success");
  else
    log("PNP done - failure");


  // MHN: Adding additional stuff to get hand out of the visual field
  // before aborting the attempt at pick and place actions...
  std::vector<double> joint_pos;
  joint_pos.clear();
  joint_pos.push_back( 2.09 );   // 120 deg
  joint_pos.push_back( -1.57 );  // -90 deg
  joint_pos.push_back( 0.5235 ); // 30 deg
  joint_pos.push_back( 0.7854 );   // 45 deg
  joint_pos.push_back( -2.618 );   // -150 deg
  kat_clnt->MoveJointPositions( joint_pos );

  // nah: wait until target has been reached.
  while(!TargetReached()) {
    sleepProcess(100);
  }

  //just a sanity check in case we're not using action structs
  if(curActionId != "") {
 
    //get the original action
    shared_ptr<const CASTTypedData<planning::autogen::Action> > actionData =
      getWorkingMemoryEntry<planning::autogen::Action>(curActionId);

    // signal that we are done by overwriting our command, setting complete to
    // true
    planning::autogen::Action *response = new planning::autogen::Action();   
    *response =  *(actionData->getData());
    response->m_status = planning::autogen::COMPLETE;
    if(success) {
      response->m_succeeded = cast::cdl::triTrue;
    }
    else {
      response->m_succeeded = cast::cdl::triFalse;
    }

    overwriteWorkingMemory<planning::autogen::Action>( curActionId, response  /*, cdl::BLOCKING*/);
    log("signalled response to planner");
  }

  // Let vision, especially Segmentor, start working again...
  uninhibitSceneProcessing();

  // we are free again
  curCmdId = "";
  curActionId = "";
  
  // accept new objects again, getting changes due to robot action
  ignore_new_objects = false;
}


/**
 * While moving watch all proximity and tactile sensors and stop as soon
 * as we are about to or already have hit an obstacle.
 */
bool Manipulator::WatchMoving()
{
  // TODO: watch sensors
  if(TargetReached())
  {
    log("while %s: target reached", goalnames[curGoal]);
    return true;
  }
  else
    return false;
}

bool Manipulator::WatchRefineAbove()
{
  if(TargetReached())
  {
    if(!POINTING_HACK)
    {
      log("while %s: target reached", goalnames[curGoal]);
      return true;
    }
    else
    {
      AbortPickAndPlace();
      return false;
    }
  }
  else
    return false;
}

/**
 * While approaching for a grasp, we can want ideally the inner
 * proximity sensors to have symmetric values.
 */
bool Manipulator::WatchApproach()
{
  bool between = ObjectBetweenFingers();
  bool reached = TargetReached();
  if(between || reached)
  {
    // don't stop if target is already reached anyway as this will make the arm
    // jerk
    if(!reached)
      StopArm();
    // remember at what height we stopped, later use the same height
    // for putting down
    grasp_z = tcp_cur.m_position.m_z;
    log("while %s: object %s between fingers, target %s reached",
        goalnames[curGoal], (between?"is":"not"), (reached?"is":"not"));
    return true;
  }
  else
  {
    // TODO: if not object between fingers, but target reached, do another
    // refine grasp pose or report error
    return false;
  }
}

/**
 * While moving and holding an object, watch the tactile sensors.
 */
bool Manipulator::WatchGrasping()
{
  if(HoldingObject())
  {
    StopGripper();
    // tactile sensor values change some time after gripper closure (freeze),
    // wait some time until they settle, then read most current.
    // TODO: actually look at values, rather than wait for a specified time.
    usleep(500000);
    GetCurrentSensors();
    sensors_hold = sensors_cur;
    log("while %s: grasped object", goalnames[curGoal]);
    return true;
  }
  else
  {
    return false;
  }
}

bool Manipulator::WatchLiftObj()
{
  if(TargetReached())
  {
    log("while %s: target reached", goalnames[curGoal]);
    return true;
  }
  else
    return false;
}

bool Manipulator::WatchCarryObj()
{
  // TODO: watch proximity sensors for collision
  // TODO: collision detection turned off for now, too sensitive -> tremor
  /*if(CarriedObjectCollides())
  {
    log("Carried object collided!");
    StopArm();
    return false;
  }
  else
    return TargetReached();*/

  if(TargetReached())
  {
    log("while %s: target reached", goalnames[curGoal]);
    return true;
  }
  else
    return false;
}

bool Manipulator::WatchPutdownObj()
{
  bool touchdown = CarriedObjectTouches();
  bool reached = TargetReached();
  if(touchdown || reached)
  {
    if(!reached)
      StopArm();
    log("while %s: carried object %s touch down, target %s reached",
        goalnames[curGoal], (touchdown?"does":" does not"),
        (reached?"is":"not"));
    return true;
  }
  else
    return false;
}

bool Manipulator::WatchRelease()
{
  // HACK: always return true, as we made the call to open gripper blocking.
  // i.e. the gripper is guaranteed to be open after leaving the call to open
  // gripper.
  log("while %s: object released", goalnames[curGoal]);
  return true;
}

bool Manipulator::WatchRetreat()
{
  if(TargetReached())
  {
    log("while %s: target reached", goalnames[curGoal]);
    return true;
  }
  else
    return false;
}

bool Manipulator::WatchPnpDone()
{
  // seending the completion happens immediately in the execute function,
  // so always return true
  log("while %s: sent response command", goalnames[curGoal]);
  return true;
}

bool Manipulator::WatchIdle()
{
  // Nothing to do right now.
  // Maybe watch sensors and move if approach or contact is detected.
  // return false, i.e. never don't leave the idle state by Yourself.
  return false;
}

/**
 * Find for a given point a "convenient" orientation.
 * Basically orient the gripper so that its z axis points outwards, which
 * is a good default grasp position.
 */
void Manipulator::FindConvenientOrientation(const Vector3D &pos, Vector3D &ori)
{
  Vector3D x, y, z;  // axis vectors of target rotation matrix
  double R[3][3];    // rotation matrix

  // y points downwards
  y.m_x = 0.;
  y.m_y = 0.;
  y.m_z = -1.;
  // z points inwards, from position to origin, parallel to x-y plane
  z.m_x = katana_base.m_position.m_x - pos.m_x;
  z.m_y = katana_base.m_position.m_y - pos.m_y;
  z.m_z = 0.;
  z = Normalise(z);
  // x is normal to the above
  x = Cross(y, z);
  SetColumn3x3(R, 0, x);
  SetColumn3x3(R, 1, y);
  SetColumn3x3(R, 2, z);
  RotationMatrixToAxisAngle(R, ori);
}

void Manipulator::FindPointingOrientation(const Vector3D &pos, Vector3D &ori)
{
  Vector3D x, y, z;  // axis vectors of target rotation matrix
  double R[3][3];    // rotation matrix

  // z points downwards
  z.m_x = 0.;
  z.m_y = 0.;
  z.m_z = -1.;
  // y points outards, from position to origin, parallel to x-y plane
  y.m_x = -(katana_base.m_position.m_x - pos.m_x);
  y.m_y = -(katana_base.m_position.m_y - pos.m_y);
  y.m_z = 0.;
  y = Normalise(y);
  // x is normal to the above
  x = Cross(y, z);
  SetColumn3x3(R, 0, x);
  SetColumn3x3(R, 1, y);
  SetColumn3x3(R, 2, z);
  RotationMatrixToAxisAngle(R, ori);
}

/**
 * Finds for a given scene object a good pose above from where
 * to start the final approach.
 */
void Manipulator::FindPoseAbove(SceneObject &obj, Pose3D &pose) {
  if(!POINTING_HACK) {
    pose.m_position = obj.m_bbox.m_centroid;
    // pose.m_position.m_z += obj.m_bbox.m_size.m_z/2.;
    pose.m_position.m_z = 0.0;
    FindConvenientOrientation(pose.m_position, pose.m_orientation);

    log("findposeabove: obj=(%f,%f,%f), pose=(%f,%f,%f), orient=(%f,%f,%f)",obj.m_bbox.m_centroid.m_x,
	obj.m_bbox.m_centroid.m_y, obj.m_bbox.m_centroid.m_z, pose.m_position.m_x,
	pose.m_position.m_y, pose.m_position.m_z, pose.m_orientation.m_x, pose.m_orientation.m_y,
	pose.m_orientation.m_z);
  }
  else {
    CloseGripper();
    pose.m_position = obj.m_bbox.m_centroid;
    pose.m_position.m_x -= 0.100;  // back away
    FindPointingOrientation(pose.m_position, pose.m_orientation);
  }
}

/**
 * Find the top surface of the current object of interest in 3D world
 * co-ordinates.
 *
 * Assumption: Our object has a rectanguler top surface (i.e. is a
 * box) of unknown size. For estimating 3D position use ground plane
 * assumption: top surface is parallel to ground plane and height is
 * given by the object height.
 *
 * Returns true if a pose could be found, false otherwise.
 */
bool Manipulator::FindTopSurface3D(vector<Vector3D> &points, Vector3D &center,
    Vector3D &axis, Vector3D &norm)
{
  vector<Vector2D> img_points;
  bool ret = false;
  if(FindRectangleRobust(img_points))
  {
    Camera cam;
    if(GetCamera(cam))
    {
      // TODO: if the view frustrum of the camera does not contain the bounding
      // box of the object, abort with failure. Note that this can happen if the
      // path planner happens to find a "bad" optimal pose.

      // MHN: We do not have a 3D object representation -- so we use
      // the constant object height to determine the proper projection
      // to the ground plane...
      // double height = (curObj->m_bbox.m_centroid.m_z + curObj->m_bbox.m_size.m_z/2.);
      double height = 0.04; // 4cm...

      // find position/orientation of top surface in 3D (arm base coordinates)
      ProjectRectangleToBase(cam, height, img_points, points);
      RectangleMainAxis(points, center, axis);
      // from the ground plane assumption we know that the surface normal of
      // the top surface points up
      norm.m_x = 0.;
      norm.m_y = 0.;
      norm.m_z = 1.;

      ret = true;
    }
  }
  stat_info.have_rect3 = ret;
  if(stat_info.have_rect3)
  {
    // store current status:
    stat_info.rect3_points = points;
    stat_info.rect3_center = center;
    stat_info.rect3_axis = axis;
  }
  return ret;
}

bool TCPPointsDownwards(const Pose3D &tcp)
{
  const double DOWN = 0.7;  // threshold: downwards component of vector must be
  // at least this big, i.e. 45 degrees
  double R[3][3];
  Vector3D y;

  RotationAxisAngleToMatrix(tcp.m_orientation, R);
  y = GetColumn3x3(R, 1);
  return y.m_z < -DOWN;
}


/**
 * Find (visually) a pose wich is aligned with the top surface of the current
 * object: TCP lies in center (i.e. would just about touch the object) and is
 * aligned with the surface main axis (i.e. ready to grasp).
 * Then moving downwards will bring the object right between the fingers
 * (and the proximity and tactile sensors in the fingers can take over from
 * there).
 *
 * @param offset  to get a pose where the hand mounted camera has a better view
 *                on the _entire_ object, move pose a bit in TCP z-direction
 *                (i.e.  "downwards" in the camera image)
 */
bool Manipulator::FindTouchPoseVisual(Pose3D &pose, bool offset)
{
  if(POINTING_HACK)
  {
    FindPoseAbove(*curObj, pose);
    // pose.m_position.m_x += 0.100;
    return true;
  }

  if(!TCPPointsDownwards(tcp_cur))
  {
    log("Arm does not point downwards, aborting top surface detection");
    return false;
  }

  vector<Vector3D> points;
  Vector3D center, axis, norm;
  if(FindTopSurface3D(points, center, axis, norm)) {
    double R[3][3];
    Vector3D x, y, z;

    // we want to grasp the object at its center (or whenever the proximity
    // sensors tell us to stop)
    pose.m_position = center;
    // work out rotation matrix of new TCP from orientation of top surface
    // TCP y into surface, i.e. in negative surface normal
    y = -norm;
    
    // make the axis point inwards rather
    Vector3D inwards;
    inwards.m_x = katana_base.m_position.m_x - pose.m_position.m_x;
    inwards.m_y = katana_base.m_position.m_y - pose.m_position.m_y;
    inwards.m_z = 0.;
    inwards = Normalise(inwards);
    if(Dot(inwards, axis) < 0.) {
      axis = -axis;
    }
    // TCP z points along top surface main axis
    z = axis;
    x = Cross(y, z);
    SetColumn3x3(R, 0, x);
    SetColumn3x3(R, 1, y);
    SetColumn3x3(R, 2, z);
    RotationMatrixToAxisAngle(R, pose.m_orientation);

    const double CORR_LENGTH = 0.04;  // 0.04...
    // The offset is used to set things up for refined object pose...
    if(offset) {
      pose.m_position = pose.m_position + CORR_LENGTH*z;
    }
    // The non-offset case is used for the approach...
    else {
      pose.m_position = pose.m_position - 1.0*CORR_LENGTH*z;
    }

    std::cout << "MNP: found top surface in touchposevisual...\n";
    return true;
  }
  else {
    std::cout << "MNP: NOT found top surface in touchposevisual...\n";   
    return false;
  }
}

/**
 * Same as FindTouchPoseVisual, except that now the TCP lies in the
 * center of the object.
 */
bool Manipulator::FindGraspPoseVisual(Pose3D &pose)
{
  if(FindTouchPoseVisual(pose, false))
  {
    // we want to grasp the object at its center (or whenever the
    // proximity sensors tell us to stop)

    // MHN:: commented out the hack for the flat objects...
    // pose.m_position.m_x += 0.02;
    pose.m_position.m_z -= OBSERVE_GRASP_OFFSET_Z; // Grasp N metres above ground?...
    std::cout << "MNP: graspposevisual: grasp pose: " << pose.m_position.m_x
	      << "," << pose.m_position.m_y << "," << pose.m_position.m_z << endl;
    return true;
  }
  else {
    std::cout << "MNP: graspposevisual: touchposevisual failed...\n";
    return false;
  }
}


void Manipulator::redrawGraphics2D()
{
  stat_info.Draw2D();
}

void Manipulator::redrawGraphics3D()
{
  // draw katana base pose
  drawFrame3D(
      katana_base.m_position.m_x, katana_base.m_position.m_y,
      katana_base.m_position.m_z,
      katana_base.m_orientation.m_x, katana_base.m_orientation.m_y,
      katana_base.m_orientation.m_z,
      255, 255, 255,  0);
  // draw current pose in white
  drawFrame3D(
      tcp_cur.m_position.m_x, tcp_cur.m_position.m_y, tcp_cur.m_position.m_z,
      tcp_cur.m_orientation.m_x, tcp_cur.m_orientation.m_y, tcp_cur.m_orientation.m_z,
      255, 255, 255,  0);
  // draw goal history in blue
  for(list<Pose3D>::iterator i = tcp_goals.begin(); i != tcp_goals.end(); ++i)
  {
    drawFrame3D(
        i->m_position.m_x, i->m_position.m_y, i->m_position.m_z,
        i->m_orientation.m_x, i->m_orientation.m_y, i->m_orientation.m_z,
        50, 50, 255,  0);
  }
  stat_info.Draw3D();
}

void Manipulator::redrawGraphicsText()
{
  printText("goal: %s  (next goal : %s)\n", GoalName(curGoal),
            GoalName(nextGoals[curGoal]));

  printText("TCP: t R\n%s\n", ToCString(tcp_cur, true));

  printText("sensors: ");
  for(unsigned i = 0; i < sensors_cur.size(); i++)
    printText(" %3d", sensors_cur[i]);
  printText("\n");

  printText("current object: \n");
  if(!curObjId.empty())
    printText("ID '%s'  ", curObjId.c_str());
  else
    printText("ID N/A  ");
  if(curObj != 0)
    printText("no detail display yet\n");
  else
    printText("details N/A\n");

  // what was the last thing we attempted to do
  printText("currently doing:\n[%s]\n", stat_info.doing.c_str());
}

void Manipulator::StateChanged()
{
  // HACK: only request redraw every 10th state change - otherwise the
  // GUI might hang
  static int cnt = 0;
  if(++cnt == 10)
  {
    // make sure the display gets updated
    redrawGraphicsNow();
    cnt = 0;
  }
}

void Manipulator::RegisterAction(const string & _action) {
  // MHN:
//   planning::autogen::ActionRegistration * reg = new planning::autogen::ActionRegistration();
//   reg->m_component = CORBA::string_dup(getProcessIdentifier().c_str());
//   reg->m_subarchitecture = CORBA::string_dup(m_subarchitectureID.c_str());
//   reg->m_action = CORBA::string_dup(_action.c_str());
//   addToWorkingMemory<ActionRegistration>( newDataID(), reg  /*, cdl::BLOCKING*/);
//   log("registering action: %s", _action.c_str());
}

void Manipulator::UpdateWorkingMemory()
{
  Head *head = new Head();
  head->m_time = BALTTimer::getBALTTime();
  head->m_num = MANIPULATOR_HEAD_NUM;
  head->m_pose = tcp_cur;
  if(hand_wm_id.empty()) {
    hand_wm_id = newDataID();
    addToWorkingMemory<Vision::Head>( hand_wm_id, head  /*, cdl::BLOCKING*/);
  }
  else {
    overwriteWorkingMemory<Vision::Head>( hand_wm_id, head  /*, cdl::BLOCKING*/);
  }
}


void Manipulator::StatusInfo::Draw2D()
{
  // HACK: taking the address of a CORBA sequence is dirty, but seems to work
  if(have_img)
    man->drawRGBImage(img.m_width, img.m_height,
        (const char*)&(img.m_image[0]), 0);
  else
    man->drawText2D(10, 10, "nothing to display",  255, 255, 255,  0);
  if(have_rect2)
  {
    for(unsigned i = 0; i < rect2_points.size(); i++)
    {
      unsigned j = (i < rect2_points.size() - 1 ? i + 1 : 0);
      man->drawLine2D(
          rect2_points[i].m_x, rect2_points[i].m_y,
          rect2_points[j].m_x, rect2_points[j].m_y,
          50, 50, 255,  0);
    }
  }
}

void Manipulator::StatusInfo::Draw3D()
{
  if((man->curGoal == GOAL_REFINE_ABOVE_OBJ ||
      man->curGoal == GOAL_APPROACH_OBJ || man->curGoal == GOAL_CLOSE_GRASP) &&
     have_rect3)
  {
    for(unsigned i = 0; i < rect3_points.size(); i++)
    {
      unsigned j = (i < rect3_points.size() - 1 ? i + 1 : 0);
      man->drawLine3D(
          rect3_points[i].m_x, rect3_points[i].m_y, rect3_points[i].m_z,
          rect3_points[j].m_x, rect3_points[j].m_y, rect3_points[j].m_z,
          50, 50, 255,  0);
      man->drawLine3D(
          rect3_points[i].m_x, rect3_points[i].m_y, rect3_points[i].m_z,
          rect3_points[i].m_x, rect3_points[i].m_y, GROUND_HEIGHT,
          50, 50, 255,  0);
      man->drawLine3D(
          rect3_points[i].m_x, rect3_points[i].m_y, GROUND_HEIGHT,
          rect3_points[j].m_x, rect3_points[j].m_y, GROUND_HEIGHT,
          50, 50, 255,  0);
    }
    man->drawPoint3D(rect3_center.m_x, rect3_center.m_y, rect3_center.m_z,
        50, 50, 255,  FAT);
    Vector3D p = rect3_center + rect3_axis*0.05;
    man->drawLine3D(rect3_center.m_x, rect3_center.m_y, rect3_center.m_z,
        p.m_x, p.m_y, p.m_z,  50, 50, 255,  0);
  }
}


void Manipulator::inhibitSceneProcessing() {
  InhibitSceneProcessing * isp = new InhibitSceneProcessing();
  isp->m_component = CORBA::string_dup(getProcessIdentifier().c_str());
  //HACK: seems like everyone else is doing it
  m_inibitStructAddress.m_id = CORBA::string_dup(newDataID().c_str());
  m_inibitStructAddress.m_subarchitecture = CORBA::string_dup("vision.sa");
  addToWorkingMemory(m_inibitStructAddress, isp, cdl::BLOCKING);
  //lockEntry(m_inibitStructAddress, cdl::LOCKED_ODR);
  log("Manipulator::inhibitSceneProcessing");
}

void Manipulator::uninhibitSceneProcessing() {
  try {
    //this will unlock too
    deleteFromWorkingMemory(m_inibitStructAddress);
    log("Manipulator::uninhibitSceneProcessing");
  }
  catch(const SubarchitectureProcessException & e) {
    log("Manipulator::uninhibitSceneProcessing: %s", e.what());
  }

}
