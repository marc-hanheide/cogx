/**
 * Camera
 * Provides internal and external (= 3D pose) camera parameters.
 * Setting 3D pose is a job for either the StereoHead or the Manipulator,
 * whoever carries a camera.
 *
 * @author Michael Zillich
 * @date October 2006
 */

#include <fstream>
#include <vision/VisionGoals.h>
#include <vision/utils/VisionUtils.h>
#include "CameraServer.h"
#include <cast/architecture/ChangeFilterFactory.hpp>
using namespace Vision;

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new CameraServer(_id);
  }
}


CameraServer::CameraServer(const string &_id)
  : WorkingMemoryAttachedComponent(_id), 
    ManagedProcess(_id)
{
  setReceiveXarchChangeNotifications(true);

  // default values
  cam_num = DEFAULT_CAMERA;
  head_num = DEFAULT_HEAD;
  downsample = DEFAULT_DOWNSAMPLE;
  SetIdentity(eye_pose);
  SetIdentity(head_pose);
  SetIdentity(ground_pose);  // HACK
}

void CameraServer::start()
{
  ManagedProcess::start();
  
  MemberFunctionChangeReceiver<CameraServer> * headChangeReceiver = 
      new MemberFunctionChangeReceiver<CameraServer>(this,
						     &CameraServer::handleUpdateHead);
  

  addChangeFilter(createGlobalTypeFilter<Head>(cdl::OVERWRITE),
		  headChangeReceiver);  
  
}

CameraServer::~CameraServer()
{
}

void CameraServer::taskAdopted(const string &_taskID)
{
}

void CameraServer::taskRejected(const string &_taskID)
{
}

void CameraServer::handleUpdateHead(const cdl::WorkingMemoryChange &_wmc)
{
  shared_ptr<const CASTTypedData<Head> > blorb =
    getWorkingMemoryEntry<Head>(_wmc.m_address);

  // if our head was updated, get new pose
  if(blorb->getData()->m_num == head_num)
  {
    UpdateHeadPose(blorb->getData()->m_pose);
    UpdateWorkingMemory();
  }
}

void CameraServer::runComponent()
{
  lockProcess();
  // for a start, write current camera parameters into working memory
  UpdateWorkingMemory();
  unlockProcess();
}

/**
 * Options are:
 * --calib file .. camera calibration file (internal camera parameters).
 *                 Mandatory.
 * --camnum num .. camera number (e.g. 0 = left, 1 = right). Defaults to 0.
 * --eyepose file .. pose calibration file: camera pose w.r.t. "head", which can 
 *                   be a stereo head or a manipulator hand.
 *                   If the camera is fixed (not mounted on any head) this pose
 *                   is just the camera pose w.r.t. the world origin.
 *                   Defaults to identity.
 * --headnum num .. head number (e.g. 0 = stereo head, 1 = hand)
 * --headpose file .. pose calibration file: head pose w.r.t. world origin.
 *                    In case the head never moves, its pose can be specified
 *                    here. Defaults to identity.
 * --groundpose .. pose of ground (table) w.r.t. camera (HACK)
 * --downsample .. downsample factor, optional, default 1
 */
void CameraServer::configure(map<string,string> & _config)
{
  ManagedProcess::configure(_config);

  if(_config["--calib"] != "")
    cam_pars.Load(_config["--calib"].c_str());
  else
    throw BALTException(__HERE__, "no camera calibration file specified");
  if(_config["--camnum"] != "")
    cam_num = atoi(_config["--camnum"].c_str());
  if(_config["--headnum"] != "")
    head_num = atoi(_config["--headnum"].c_str());
  if(_config["--eyepose"] != "")
    ReadPose(_config["--eyepose"].c_str(), eye_pose);
  if(_config["--headpose"] != "")
    ReadPose(_config["--headpose"].c_str(), head_pose);
  if(_config["--downsample"] != "")
  {
    downsample = atoi(_config["--downsample"].c_str());
    if(downsample < 1)
    {
      println("invalid downsample factor, defaulting to 1");
      downsample = DEFAULT_DOWNSAMPLE;
    }
  }
  // HACK
  if(_config["--groundpose"] != "")
  {
    // saved pose from calibration routine is the pose of the camera w.r.t.
    // ground object (= calibration object)
    ReadPose(_config["--groundpose"].c_str(), ground_pose);
    // we want pose of the ground object w.r.t. camera
    ground_pose = InvertPose3D(ground_pose);
  }
  // HACK END
}

void CameraServer::CopyParameters(const CamPars &cp, Camera &cam)
{
  cam.m_fx = cp.fx/(double)downsample;
  cam.m_fy = cp.fy/(double)downsample;
  cam.m_cx = cp.cx/(double)downsample;
  cam.m_cy = cp.cy/(double)downsample;
  cam.m_width = cp.w/downsample;
  cam.m_height = cp.h/downsample;
  // note: distortion parameters are unaffected by downsampling
  cam.m_k1 = cp.k1;
  cam.m_k2 = cp.k2;
  cam.m_k3 = 0.;
  cam.m_t1 = cp.p1;
  cam.m_t2 = cp.p2;
  cam.m_pose = TransformPoseToGlobal(head_pose, eye_pose);
  cam.m_num = cam_num;
  // cam.m_time = BALTTimer::getBALTTime(); TODO
}
 
/**
 * Write current camera parameters into working memory.
 */
void CameraServer::UpdateWorkingMemory()
{
  Camera *cam = new Camera();
  CopyParameters(cam_pars, *cam);
  // cam->m_time = BALTTimer::getBALTTime();  TODO
  if(cam_wm_id.empty())
  {
    cam_wm_id = newDataID();
    addToWorkingMemory<Camera>(cam_wm_id, cam, cdl::BLOCKING); // sync=true
  }
  else
      overwriteWorkingMemory<Camera>(cam_wm_id, cam);
}

void CameraServer::UpdateHeadPose(const Pose3D &new_head_pose)
{
  head_pose = new_head_pose;
}

void CameraServer::redrawGraphics3D()
{
  const double l = 0.200; // length of view rays for view frustrum
  Camera cam;
  CopyParameters(cam_pars, cam);
  drawFrame3D(cam.m_pose.m_position.m_x, cam.m_pose.m_position.m_y,
      cam.m_pose.m_position.m_z, cam.m_pose.m_orientation.m_x,
      cam.m_pose.m_orientation.m_y, cam.m_pose.m_orientation.m_z,
      255, 255, 0,  0);

  // draw the view frustrum
  Vector3D e;  // eye point = origin of line of sight [m]
  Vector3D d;  // direction of line of sight [m]
  Vector2D c;  // point in image corner

  c.m_x = 0.; c.m_y = 0.;
  ImagePointToGlobalRay(cam, c, e, d);
  d = e + l*d;
  drawLine3D(e.m_x, e.m_y, e.m_z, d.m_x, d.m_y, d.m_z, 127, 127, 0,  0);

  c.m_x = cam.m_width; c.m_y = 0.;
  ImagePointToGlobalRay(cam, c, e, d);
  d = e + l*d;
  drawLine3D(e.m_x, e.m_y, e.m_z, d.m_x, d.m_y, d.m_z, 127, 127, 0,  0);

  c.m_x = cam.m_width; c.m_y = cam.m_height;
  ImagePointToGlobalRay(cam, c, e, d);
  d = e + l*d;
  drawLine3D(e.m_x, e.m_y, e.m_z, d.m_x, d.m_y, d.m_z, 127, 127, 0,  0);

  c.m_x = 0.; c.m_y = cam.m_height;
  ImagePointToGlobalRay(cam, c, e, d);
  d = e + l*d;
  drawLine3D(e.m_x, e.m_y, e.m_z, d.m_x, d.m_y, d.m_z, 127, 127, 0,  0);
 
  // HACK
  // the LEFT camera draws the table (ground plane)
  if(cam_num == CAM_LEFT && !IsIdentity(ground_pose))
  {
    const int NX = 5;  // number of dots
    const int NY = 5;
    const double SIZEX = 0.150;  // dot width
    const double SIZEY = 0.150;
    for(int i = 0; i < NX; i++)
    {
      // ground points in camera co-ordinates
      Vector3D p, q;
      p.m_x = q.m_x = (double)i*SIZEX;
      p.m_y = 0.000;
      q.m_y = (double)(NY - 1)*SIZEY;
      p.m_z = q.m_z = 0.000;
      // ground points in camera co-ordinates
      p = TransformPointToGlobal(ground_pose, p);
      q = TransformPointToGlobal(ground_pose, q);
      // ground points in world co-ordinates
      p = TransformPointToGlobal(cam.m_pose, p);
      q = TransformPointToGlobal(cam.m_pose, q);
      drawLine3D(p.m_x, p.m_y, p.m_z,  q.m_x, q.m_y, q.m_z,  255, 0, 0,  0);
    }
    for(int i = 0; i < NX; i++)
    {
      // ground points in camera co-ordinates
      Vector3D p, q;
      p.m_x = 0.000;
      q.m_x = (double)(NX - 1)*SIZEX;
      p.m_y = q.m_y = (double)i*SIZEY;
      p.m_z = q.m_z = 0.000;
      // ground points in camera co-ordinates
      p = TransformPointToGlobal(ground_pose, p);
      q = TransformPointToGlobal(ground_pose, q);
      // ground points in world co-ordinates
      p = TransformPointToGlobal(cam.m_pose, p);
      q = TransformPointToGlobal(cam.m_pose, q);
      drawLine3D(p.m_x, p.m_y, p.m_z,  q.m_x, q.m_y, q.m_z,  255, 0, 0,  0);
    }
    Pose3D pose = TransformPoseToGlobal(cam.m_pose, ground_pose);
    drawFrame3D(pose.m_position.m_x, pose.m_position.m_y,
        pose.m_position.m_z, pose.m_orientation.m_x,
        pose.m_orientation.m_y, pose.m_orientation.m_z,
        255, 0, 0,  0);
  }
  // HACK END
}

void CameraServer::redrawGraphicsText()
{
  Pose3D pose = TransformPoseToGlobal(head_pose, eye_pose);

  printText("head %d, camera %d:\n", head_num, cam_num);
  printText("image size: %d x %d\n", cam_pars.w, cam_pars.h);
  printText("camera matrix: fx = %.3f, fy = %.3f, cx = %.3f, cy = %.3f\n",
         cam_pars.fx, cam_pars.fy, cam_pars.cx, cam_pars.cy);
  printText("distortion: radial %.3f, %.3f  tangential %.3f, %.3f\n",
         cam_pars.k1, cam_pars.k2, cam_pars.p1, cam_pars.p2);
  printText("pose: %s\n", ToCString(pose));
}

