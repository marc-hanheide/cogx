/**
 * @file KinectPCServer.cpp
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the kinect sensor.
 */

#define EIGEN2_SUPPORT

#include "KinectPCServer.h"
#include <cast/core/CASTUtils.hpp>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <Eigen/LeastSquares>
#include <Eigen/Geometry>
#include <highgui.h>

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr newComponent()
  {
    return new cast::KinectPCServer();
  }
}

namespace cast {

using namespace std;
using namespace cogx;
using namespace cogx::Math;
using namespace cast::cdl;

KinectPCServer::KinectPCServer()
{
  for (int i = 0; i < N_PLANES; i++) {
    fovPlanes[i] = NULL;
    senses[i] = 0;
  }

#ifdef KINECT_USER_DETECTOR
  personDetectServer=new PersonDetectServerI(this);
#endif

  m_createViewCone = false;
  m_viewConeNeedsUpdate = false;
  m_tmUpdateViewCone.setTimeout(500); // update the view cone at most at 2Hz
  m_tmUpdateViewCone.restart();
}

KinectPCServer::~KinectPCServer()
{
  if (kinect)
    delete kinect;

  /* Delete the view cone planes */
  deleteViewConePlanes();
}

/**
 * @brief Get video resolution for given camera.
 * @param camIdx which camera
 * @param size video resolution
 */
void KinectPCServer::getResolution(int camIdx, CvSize &size)
{
  kinect->GetColorVideoSize(size);
}

/**
 * @brief Set resolution for a given camera
 * @param camIdx which camera
 * @param size requested video resolution, on exit contains the actually
 *        set resolution, which might differ dependig on the cameras capabilities
 * @return true if the requested resolution could be set, false if another
 *        reslution was chosen
 */
bool KinectPCServer::setResolution(int camIdx, CvSize &size)
{
  log("setResolution: warning: Not yet implemented! Defined by kinect camera calibration file!\n");
  return false;
}

/**
 * @brief Configure the component
 */
void KinectPCServer::configure(const map<string, string> & _config)
  throw (runtime_error)
{
  // configure the point cloud server
  PointCloudServer::configure(_config);

  map<string, string>::const_iterator it;

  m_createViewCone = false;
  if (_config.find("--create-viewcone") != _config.end()) {
    m_createViewCone = true;
  }

#ifdef KINECT_USER_DETECTOR
  m_detectPersons = false;
  if (_config.find("--detect-persons") != _config.end()) {
    m_detectPersons = true;
  }
#endif
  if ((it = _config.find("--kconfig")) != _config.end()) {
    istringstream str(it->second);
    str >> kinectConfig;
  } else{
    throw runtime_error(exceptionMessage(__HERE__, "no kinect config file (kconfig) specified."));
  }
  // init kinect hardware driver
  CvSize size;
  const char* name = kinectConfig.c_str();
  kinect = new Kinect::Kinect(this, name);
  kinect->GetColorVideoSize(size);
  captureSize.width = size.width;
  captureSize.height = size.height;
  kinect->StartCapture(0); // start capturing
  depthGenerator = kinect::getDepthGenerator();
  imageGenerator = kinect::getImageGenerator();
#ifdef KINECT_USER_DETECTOR
  userGenerator = kinect::getUserGenerator();
#endif
  m_saveToFile = false;
  if ((it = _config.find("--save-to-file")) != _config.end()) {
    m_saveToFile = true;

    if ((it = _config.find("--save-directory")) != _config.end()) {
      std::istringstream str(it->second);
      str >> m_saveDirectory;
      //check if the last char is / if yes remove
      if (m_saveDirectory[m_saveDirectory.size()-1] == '/'){
        m_saveDirectory.erase(m_saveDirectory.size()-1);
      }
      log("Will be saving scans to %s", m_saveDirectory.c_str());
    }
    else
    {
      log("You haven't specified a save directory! (usage: --save-directory)");
      abort();
    }
  }

  m_displayImage = false;
  if ((it = _config.find("--display-rgb")) != _config.end()) {
    log("Will display Kinect RGB image \n");
    m_displayImage= true;
  }
  m_lastframe = -1;
  log("Capturing from kinect sensor started.");


#ifdef KINECT_USER_DETECTOR
  registerIceServer<kinect::slice::PersonDetectorInterface, PersonDetectServerI>(personDetectServer);
  log("PersonDetectServer registered");
#endif

#ifdef FEAT_VISUALIZATION
  m_bUseV11n = false;
  if ((it = _config.find("--displayserver")) != _config.end()) {
    m_bUseV11n = true;
  }
  m_display.configureDisplayClient(_config);
#endif
}

#ifdef KINECT_USER_DETECTOR

kinect::slice::PersonsDict PersonDetectServerI::getPersons(const Ice::Current& cur)
{
  kinect::slice::PersonsDict persons;
  return pcSrv->detectPersons();
}

#endif

/**
* @brief Configure the component
*/
void KinectPCServer::start()
{
  PointCloudServer::start();

#ifdef FEAT_VISUALIZATION
  m_display.connectIceClient(*this);
#endif
}

void KinectPCServer::runComponent()
{
  log("I am running ");
  kinect->NextFrame();
  if(m_displayImage){
    log("Displaying window");
    cvNamedWindow("Kinect RGB",CV_WINDOW_AUTOSIZE);
  }
  //cvWaitKey(100);

  castutils::CCastPaceMaker<KinectPCServer> paceMaker(*this, 1000/20, 1); // run at 20Hz
#ifdef FEAT_VISUALIZATION
  castutils::CMilliTimer tmV11nUpdate;
  tmV11nUpdate.setTimeout(2000);
#endif

  while(isRunning()) {
    paceMaker.sync();

#ifdef KINECT_USER_DETECTOR
    if (m_detectPersons) {
      detectPersons();
    }
#endif

    if (m_saveToFile) {
      saveNextFrameToFile();
    }

#ifdef FEAT_VISUALIZATION
    if (m_bUseV11n && tmV11nUpdate.isTimeoutReached()) {
      tmV11nUpdate.restart();
      if (kinect) {
        kinect->NextFrame();
        IplImage* pimg = 0; // = new IplImage(kinect->rgbImage);
        kinect->GetDepthImageRgb(&pimg);
        m_display.setImage("kinect.depth.rgb", pimg);
        cvReleaseImage(&pimg);
      }
    }
#endif
  }
}

#ifdef KINECT_USER_DETECTOR
::kinect::slice::PersonsDict KinectPCServer::detectPersons()
{
  ::kinect::slice::PersonsDict persons;
  if (userGenerator==NULL || !userGenerator->IsValid()) {
    println("we don't have a valid userGenerator");
    return persons;
  }
  lockComponent();
  if (!suspendReading) {
    kinect->NextFrame();
  }
  int count=userGenerator->GetNumberOfUsers();
  XnUserID aUsers[count];
  XnUInt16 nUsers=count;

  userGenerator->GetUsers(aUsers, nUsers);
  for (int i=0; i<count; i++) {
    xn::SceneMetaData smd;
    userGenerator->GetUserPixels(aUsers[i], smd);

    int xRes=smd.LabelMap().XRes();
    int yRes=smd.LabelMap().YRes();
    long pixelCount=0;
    for (int y=0;y<yRes; y++) {
      for (int x=0;x<xRes; x++) {
        int v=smd.LabelMap()(x,y);
        if (v==aUsers[i])
          pixelCount++;
      }
    }
    double pixelRatio = ((double) pixelCount)/(xRes*yRes);
    debug("user %d xRes=%d, yRes=%d pixelCount=%f", aUsers[i], xRes, yRes, pixelRatio);
    if (pixelRatio > RELATIVE_MINIMUM_PERSON_AREA) {
      kinect::slice::KinectPersonPtr person = new kinect::slice::KinectPerson;
      person->size=pixelCount;
      persons[aUsers[i]] = person;

    }
  }
  debug("number of users in image: %d", persons.size());
  unlockComponent();
  return persons;
}

#endif

void KinectPCServer::saveNextFrameToFile()
{
  kinect->NextFrame();
  if(kinect->frameNumber == m_lastframe){
    return;
  }
  IplImage* rgb_data = 0; // = new IplImage(kinect->rgbImage);
  kinect->GetColorImage(&rgb_data);
  if(m_displayImage){
    cvShowImage("Kinect RGB",rgb_data);
    cvWaitKey(5);
  }
  char buf[256];
  CASTTime timeNow = getCASTTime();
  sprintf(buf, "%s/frame_%04d_rgb_%ld_%ld.bmp", m_saveDirectory.c_str(), kinect->frameNumber,
      (long int)timeNow.s, (long int)timeNow.us);

  cvSaveImage(buf, rgb_data);
  cvReleaseImage(&rgb_data);

  IplImage* depth_data = 0;
  kinect->GetDepthImageRgb(&depth_data, /*use-hsv=*/false);
  sprintf(buf,"%s/frame_%04d_depth_%ld_%ld.bmp", m_saveDirectory.c_str(), kinect->frameNumber,
      (long int)timeNow.s, (long int)timeNow.us);
  debug("Saving Kinect frame # %d",kinect->frameNumber);
  cvSaveImage(buf, depth_data);
  cvReleaseImage(&depth_data);

  m_lastframe = kinect->frameNumber;
}

void KinectPCServer::deleteViewConePlanes()
{
  for (int i = 0; i < N_PLANES; i++) {
    senses[i] = 0;
    if (fovPlanes[i]) {
      delete fovPlanes[i];
      fovPlanes[i] = NULL;
    }
  }
}

bool KinectPCServer::createViewCone()
{
  //debug("Updating View Cone");

  kinect::changeRegistration(0);
  if (!suspendReading) {
    kinect->NextFrame();
  }

  cv::Mat_<cv::Point3f> cloud;
  cv::Mat_<cv::Point3f> colCloud;
  kinect->Get3dWorldPointCloud(cloud, colCloud);

  Pose3 global_kinect_pose;
  global_kinect_pose = lastValidCamPose;

  for (int i = 0; i < N_PLANES; i++) {
    if (fovPlanes[i])
      continue;
    cv::Mat_<cv::Point3f> colOrRow;
    Eigen::Vector3d axis(0, 0, 0);
    switch (i) {
      case PLANE_LEFT:
        /* Grab left column */
        colOrRow = cloud.col(0);
        axis = Eigen::Vector3d(0, 1, 0);
        break;
      case PLANE_TOP:
        /* Grab top row */
        colOrRow = cloud.row(0);
        axis = Eigen::Vector3d(0, 0, 1);
        break;
      case PLANE_RIGHT:
        /* Grab right column (NOTE: ignore deadzone of 8 pixels) */
        colOrRow = cloud.col(cloud.size().width - 9);
        axis = Eigen::Vector3d(0, 1, 0);
        break;
      case PLANE_BOTTOM:
        /* Grab bottom row */
        colOrRow = cloud.row(cloud.size().height - 1);
        axis = Eigen::Vector3d(0, 0, 1);
        break;
    }
    std::vector<cv::Point3f> cvPoints(colOrRow.begin(), colOrRow.end());
    fovPlanes[i] = createPlane(cvPoints, global_kinect_pose);
    if (fovPlanes[i] != NULL) {
      /* Compensate for the variable direction of the plane normals */
      senses[i] = 2 * (std::acos(axis.dot(fovPlanes[i]->normal())) > M_PI/2) - 1;
    }
  }

  return fovPlanes[0] && fovPlanes[1] && fovPlanes[2] && fovPlanes[3];
}

// Check if the view cone has to be updated and update it.
void KinectPCServer::checkUpdateViewCone()
{
  if (m_viewConeNeedsUpdate && m_tmUpdateViewCone.isTimeoutReached()) {
    m_viewConeNeedsUpdate = false;
    m_tmUpdateViewCone.restart();

    /* Delete old view cone planes */
    deleteViewConePlanes();
    bool hasAllPlanes = createViewCone();
    if (!hasAllPlanes)
      log("Failed to get a complete view cone!");
  }

}

bool KinectPCServer::isPointInViewCone(const Vector3& point)
{
  lockComponent();

  checkUpdateViewCone();

  /* Make sure we have a complete view cone */
  for (int i = 0; i < N_PLANES; i++) {
    if (fovPlanes[i] == NULL) {
      unlockComponent();
      return false;
    }
  }

  Eigen::Vector3d p(point.x, point.y, point.z);

  bool inView = senses[PLANE_LEFT] * fovPlanes[PLANE_LEFT]->signedDistance(p) >= 0 &&
    senses[PLANE_TOP] * fovPlanes[PLANE_TOP]->signedDistance(p) >= 0 &&
    senses[PLANE_RIGHT] * fovPlanes[PLANE_RIGHT]->signedDistance(p) <= 0 &&
    senses[PLANE_BOTTOM] * fovPlanes[PLANE_BOTTOM]->signedDistance(p) <= 0;

  unlockComponent();

  return inView;
}

// ########################## Point Cloud Server Implementations ########################## //
void KinectPCServer::getPoints(bool transformToGlobal, int imgWidth,
    vector<PointCloud::SurfacePoint> &points, bool complete)
{
  lockComponent();

  cv::Mat_<cv::Point3f> cloud;
  cv::Mat_<cv::Point3f> colCloud;
  if (!suspendReading) {
    kinect->NextFrame();
    lastValidCamPose=camPars[0].pose;
  }

  kinect->Get3dWorldPointCloud(cloud, colCloud);

  Pose3 global_kinect_pose;
  if(transformToGlobal)
    global_kinect_pose = lastValidCamPose;

  // copy clouds to points-vector (dense!)
  PointCloud::SurfacePoint pt;
  int scale = imgWidth == 0 ? 1 : cloud.size().width / imgWidth;
  if (scale < 1) {
     scale = 1;
  }
  int ncol = cloud.size().width;
  int nrow = cloud.size().height;
  if (complete) {
     points.reserve(ncol * nrow);  
  }
  for (int row=0; row < nrow; row += scale)   /// SLOW conversion
  {
    for (int col=0; col < ncol; col += scale)
    {
      const cv::Point3f& cvpt = cloud.at<cv::Point3f>(row, col); 

      /* Check point for validity */
      if (cvpt.x == FLT_MAX || cvpt.y == FLT_MAX || cvpt.z == FLT_MAX)
      {
        /* If no data is available add (0,0,0) as specified in PointCloudServer */
        if (complete) {
          pt.p.x = 0;
          pt.p.y = 0;
          pt.p.z = 0;
        }
        /* Or if we don't care about missing data ignore the point */
        else
          continue;
      }
      else {
         pt.p.x = cvpt.x;
         pt.p.y = cvpt.y;
         pt.p.z = cvpt.z;
      }

      const cv::Point3f& cvco = colCloud.at<cv::Point3f>(row, col);
      pt.c.r = cvco.z;
      pt.c.g = cvco.y;
      pt.c.b = cvco.x;

      if(transformToGlobal)
        pt.p = transform(global_kinect_pose, pt.p);   // now get from kinect cam coord sys to global coord sys

      points.push_back(pt);
    }
  }
  unlockComponent();
}


void KinectPCServer::getRectImage(int side, int imgWidth, Video::Image& image)
{
  lockComponent();

  if (!suspendReading) {
    kinect->NextFrame();
    lastValidCamPose=camPars[0].pose;
  }

  double scaleFactor = 1.;
  IplImage *rgbImage;
  if (side < 0) {
     kinect->GetDepthImageRgb(&rgbImage);
     debug("got depth color image from Kinect");
  }
  else {
     kinect->GetColorImage(&rgbImage);
     debug("got color image from Kinect");
  }

  if(imgWidth != rgbImage->width)
  {
    debug("needs to be resized");
    IplImage *resized;
    scaleFactor = (double) rgbImage->width / (double) imgWidth;
    resized = cvCreateImage(cvSize(imgWidth, (int) (rgbImage->height/scaleFactor)), IPL_DEPTH_8U, 3);
    cvResize(rgbImage, resized);
    convertImageFromIpl(resized, image);
    cvReleaseImage(&rgbImage);
    cvReleaseImage(&resized);
  }
  else
  {
    debug("no need to be resized");
    convertImageFromIpl(rgbImage, image);
    cvReleaseImage(&rgbImage);
  }

  initCameraParameters(image.camPars);
  image.camPars = camPars[0];
  changeImageSize(image.camPars, imgWidth, imgWidth*3/4);

  unlockComponent();
}

bool KinectPCServer::getCameraParameters(Ice::Int side, Video::CameraParameters& _camPars)
{
  lockComponent(); // TODO: CASTComponent::Lock lock(this);

  int imgWidth = kinect->GetRgbImageWidth(); //rgbImage->width;
  initCameraParameters(_camPars);
  _camPars = camPars[0];
  changeImageSize(_camPars, imgWidth, imgWidth*3/4);

  unlockComponent(); // TODO: remove

  return true;
}

// @author: mmarko
// 2011-10-12 NOTE: we override this because we are not sure how many cameras
// there are in PointCloudServer::camPars for KinectPCServer. We only need
// the parameters of the LEFT (= color) camera.
bool KinectPCServer::isPointVisible(const cogx::Math::Vector3& point)
{
  Video::CameraParameters camPars;

  if (!getCameraParameters(LEFT, camPars) || ! Video::isPointVisible(camPars, point)) {
      //log("Point is NOT visible in camera %d", camPars.id);
      return false;
  }
  //log("Point is visible in camera %d", camPars.id);


  return true;
}

void KinectPCServer::getDisparityImage(int imgWidth, Video::Image& image)
{
  printf("KinectPCServer::getDisparityImage: Warning: Not yet implemented!\n");
  /*
     lockComponent();
     convertImageFromIpl(disparityImg, image);
     unlockComponent();
   */
}

void KinectPCServer::getRangePoints(Laser::Scan2d &KRdata)
{
  printf("KinectPCServer::getRangePoints: Warning: Not yet implemented!\n");
}

void KinectPCServer::getDepthMap(cast::cdl::CASTTime &time, vector<int>& depth)
{
  lockComponent();
  //kinect->NextFrame();
  //cv::Mat RGB;
  //cv::Mat DEPTH;
  //kinect->GetImages(RGB, DEPTH);
  //kinect::readFrame();  // read next frame
  kinect::changeRegistration(0);
  const DepthMetaData* pDepthMD = kinect->getNextDepthMD();
  time = getCASTTime();

  //for(int row=0;row < DEPTH.rows;row++)
  depth.reserve(pDepthMD->YRes() * pDepthMD->XRes());
  for(size_t row=0;row < pDepthMD->YRes();row++)
  {
    for(size_t col=0;col < pDepthMD->XRes();col++)
    {
      //depth.push_back( int ( DEPTH.ptr<short>(row)[(640-col-1)] ) );
      //depth.push_back( int ( DEPTH.ptr<short>(row)[col] ) );
      depth.push_back( (*pDepthMD)(col,row) );
    }
  }
  unlockComponent();
}

void KinectPCServer::receiveImages(const vector<Video::Image>& images)
{
  //   lockComponent();
  printf("KinectPCServer::receiveImages: Warning: Not yet implemented!\n");
  //stereoProcessing();
  //   unlockComponent();
}

void KinectPCServer::receiveCameraParameters(const cdl::WorkingMemoryChange & _wmc)
{
  PointCloudServer::receiveCameraParameters(_wmc);

  if (m_createViewCone)
  {
    m_viewConeNeedsUpdate = true;
  }
}

class CvToGlobal
{
  Pose3& global_pose;
public:
  CvToGlobal(Pose3& pose) : global_pose(pose) {}
  Vector3 operator()(const cv::Point3f& p) {
    Vector3 v;
    v.x = p.x;
    v.y = p.y;
    v.z = p.z;
    v = transform(global_pose, v);
    return v;
  }
};
class Vector3ToVector3d
{
public:
  Eigen::Vector3d operator()(const Vector3& p) {
    return Eigen::Vector3d(p.x, p.y, p.z);
  }
};
class InvalidPointFilter
{
public:
  bool operator()(const cv::Point3f& p) const {
    return p.x == FLT_MAX || p.y == FLT_MAX || p.z == FLT_MAX;
  }
};

Eigen::Hyperplane<double, 3>* KinectPCServer::createPlane(std::vector<cv::Point3f>& cvPoints,
    Pose3& global_kinect_pose)
{
  Eigen::Hyperplane<double, 3>* plane = NULL;

  /* Filter invalid points */
  cvPoints.erase(std::remove_if(cvPoints.begin(), cvPoints.end(), InvalidPointFilter()), cvPoints.end());
  /* Transform to global coordinates */
  std::vector<Vector3> globalPoints(cvPoints.size());
  std::transform(cvPoints.begin(), cvPoints.end(), globalPoints.begin(), CvToGlobal(global_kinect_pose));
  /* Include the kinect position */
  cvPoints.push_back(cv::Point3f(global_kinect_pose.pos.x, global_kinect_pose.pos.y, global_kinect_pose.pos.z));
  /* Transform to Eigen::Vector3d */
  std::vector<Eigen::Vector3d> points(globalPoints.size());
  std::transform(globalPoints.begin(), globalPoints.end(), points.begin(), Vector3ToVector3d());

  if (points.size() >= 3) {
    plane = new Eigen::Hyperplane<double, 3>(3);
    // create a vector of pointers to the points (weird Eigen API...)
    std::vector<Eigen::Vector3d*> points_ptrs(points.size());
    for(unsigned int k=0; k<points.size(); ++k) points_ptrs[k] = &points[k];

    /* Fit a plane to the points */
    double soundness;
    Eigen::fitHyperplane(points.size(), &(points_ptrs[0]), plane, &soundness);
    /* FIXME: Check soundness! */
  }
  return plane;
}

}

