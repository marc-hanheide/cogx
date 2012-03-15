/**
 * Provides 3D poses of cameras.
 * Poses can change if cameras are mounted on pan-tilt unit or arm.
 * NOTE: right now only works with pan-tilt unit (PTZ server).
 *
 * Michael Zillich
 * Oct 2009
 */

#ifndef CAMERA_MOUNT_HPP
#define CAMERA_MOUNT_HPP

#include <cast/architecture/ManagedComponent.hpp>
#include <castutils/Timers.hpp>
#include <Math.hpp>
#include <PTZ.hpp>

#include <IceUtil/IceUtil.h>

#include <stdexcept>
#include <vector>

class CameraMount : public cast::ManagedComponent
{
private:
  /**
   * Tolerance for deciding whther we reached a fixed position, in rad.
   * 0.015 rad = a bit less than 1 degree
   */
  static double FIXED_POSITION_TOLERANCE;
  /**
   * ICE inteface to PTU server
   */
  ptz::PTZInterfacePrx m_PTUServer;
  /**
   * IDs of cameras (0, 1 etc.)
   */
  std::vector<int> camIds;

  // @author: mmarko
  struct MotionInfo
  {
     CameraMount* pOwner;
     int mCamId;
     std::string mWmId;
     Video::CameraMotionStatePtr mpState;
     bool mbSaved;
     unsigned int mForcedMoveCount;
     double mMinDelta;
     castutils::CMilliTimer mEndMoveTimeout;

     // Last rotation vector for the detection of camera movements
     cogx::Math::Vector3 mPrevRot;

     MotionInfo(CameraMount* pComponent, int camid, std::string wmid, double minDelta)
        : pOwner(pComponent), mCamId(camid), mWmId(wmid), mMinDelta(minDelta),
          mbSaved(false), mForcedMoveCount(0)
     {
        mPrevRot = cogx::Math::vector3(0, 0, 0);
        mpState = new Video::CameraMotionState();
        mpState->camid = mCamId;
        mpState->bMoving = false;
     }

     bool isMoving()
     {
        return mpState.get() && mpState->bMoving;
     }
     void startMoving(unsigned int count=2)
     {
       mForcedMoveCount = (count > 20) ? 20 : count;
     } 
     bool detectStatusChange(cogx::Math::Pose3& newPose);
     void writeWm();
  };

  std::vector<CameraMount::MotionInfo> mCamMotion;
  IceUtil::Mutex mCamMotionMutex;

  // For the --motion_tollerance parameter.
  // The minimum change in the sum of ptz angle deltas that is classified as
  // movement. The default is 1e-4. For Gazebo set to at least 2e-3.
  // (the value for Gazebo was tested when CameraMount was running at approx. 2Hz)
  double mMotionTollerance;

  /**
   * WM IDs of camera poses.
   */
  std::vector<std::string> camWMIds;
  /**
   * Pose of PTU base w.r.t. to robot ego.
   */
  cogx::Math::Pose3 ptBasePose;
  /**
   * Pose of PTU pan joint w.r.t. base.
   */
  cogx::Math::Pose3 ptPanPose;
  /**
   * Pose of PTU tilt joint w.r.t. pan joint.
   */
  cogx::Math::Pose3 ptTiltPose;
  /**
   * Poses of cameras w.r.t. tilt joint.
   */
  std::vector<cogx::Math::Pose3> camPoses;
  /**
   * Poses of cameras w.r.t. robot ego, for fixed pan/tilt angle.
   */
  std::vector<cogx::Math::Pose3> camFixedPoses;
  /**
   * If true, ask the PTZ server regularly for pan/tilt angles to update camera poses.
   */
  bool usePTZ;
  /**
   * CAST component name of PTZ server
   */
  std::string ptzServerComponent;

  /**
   * Fixed pan and tilt angle, for which we have precise calibration.
   */
  ptz::PTZReading fixedPanTilt;
  /**
   * ID of the reference camera.
   * For a fixed camera setup (such as stereo, or stereo plus kinect)
   * one camera (e.g. the left) is typically the reference to which the other
   * cameras are calibrated, i.e. this is what a typical stereo calibration
   * procedure such as the one in tools/syscalb will output.
   * So the pose of a camera is its pose relative to the reference cam, which
   * is in turn relative to the head or fixed.
   * If the ID is set to -1 this means there is no reference camera and all
   * camera poses are relative to the head or fixed.
   */
  int ref_cam_id;

  /**
   * for given pan and tilt angles, calculate poses of cameras w.r.t. robot ego
   * system
   */
  void calculatePoses(ptz::PTZReading &ptz, std::vector<cogx::Math::Pose3> &camPosesToEgo);
  /**
   * obtain fixed poses of cameras, taking into account the reference camera
   */
  void calculateFixedPoses(std::vector<cogx::Math::Pose3> &camPosesToEgo);

  // @author: mmarko
  // Used in motion state management.
  void onAdd_PtzPoseCommand(const cast::cdl::WorkingMemoryChange & _wmc);
protected:
  virtual void configure(const std::map<std::string, std::string>& _config) throw(std::runtime_error);
  virtual void start();
  virtual void runComponent();

public:
  CameraMount();
  virtual ~CameraMount() {}
  using CASTComponent::sleepComponent; // used by paceMaker in runComponent
};

#endif

