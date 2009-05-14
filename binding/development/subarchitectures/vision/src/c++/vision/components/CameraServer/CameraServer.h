/**
 * Camera
 * Provides internal and external (= 3D pose) camera parameters.
 * Setting 3D pose is a job for either the StereoHead or the Manipulator,
 * whoever carries a camera.
 *
 * @author Michael Zillich
 * @date October 2006
 */

#ifndef CAST_CAMERA_SERVER_H
#define CAST_CAMERA_SERVER_H

#include <cast/architecture/ManagedProcess.hpp>
#include <vision/idl/Vision.hh>
#include <vision/utils/CamPars.h>

using namespace cast; using namespace std; using namespace boost; //default useful namespaces, fix to reflect your own code

class CameraServer : public ManagedProcess
{
private:
  // default values
  static const int DEFAULT_CAMERA = 0;
  static const int DEFAULT_HEAD = 0;
  static const int DEFAULT_DOWNSAMPLE = 1;

  CamPars cam_pars;
  int cam_num;
  int head_num;
  Math::Pose3D eye_pose;   // pose of eye w.r.t. head
  Math::Pose3D head_pose;  // pose of head w.r.t. world
  Math::Pose3D ground_pose;  // HACK
  int downsample;          // image downsample factor
  std::string cam_wm_id;   // working memory address of camera

  void UpdateWorkingMemory();
  void UpdateHeadPose(const Math::Pose3D &new_head_pose);
  void CopyParameters(const CamPars &cp, Vision::Camera &cam);
  void handleUpdateHead(const cdl::WorkingMemoryChange &_wmc);

protected:
  virtual void taskAdopted(const string &_taskID);
  virtual void taskRejected(const string &_taskID);
  virtual void runComponent();
  virtual void configure(map<string,string> & _config);
  virtual void redrawGraphics3D();
  virtual void redrawGraphicsText();

public:
  CameraServer(const string &_id);
  virtual ~CameraServer();
  virtual void start();
};

#endif

