/**
 * @author Michael Zillich
 * @date June 2009
 */

#include <sstream>
#include <opencv/highgui.h>
#include <cast/core/CASTUtils.hpp>
#include <VideoUtils.h>
#include <VisionUtils.h>
#include "StereoServer.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::StereoServer();
  }
}

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;

// stereo works better with smaller images, so this is the resolution we will
// work with
static const int STEREO_WIDTH = 160;
static const int STEREO_HEIGHT = 120;

static const int LEFT_CLEAR_BORDER_WIDTH = 35;
static const int RIGHT_CLEAR_BORDER_WIDTH = 5;

void StereoServerI::getPoints(VisionData::SurfacePointSeq& points, const Ice::Current&)
{
  stereoSrv->getPoints(points);
}

void StereoServerI::getPointsInSOI(const VisionData::SOIPtr &soi,
    VisionData::SurfacePointSeq& points, const Ice::Current&)
{
  stereoSrv->getPointsInSOI(*soi, points);
}

StereoServer::StereoServer()
{
  iceStereoName = "";
  iceStereoPort = cdl::CPPSERVERPORT;
  doDisplay = false;
  disparityImg = cvCreateImage(cvSize(STEREO_WIDTH, STEREO_HEIGHT), IPL_DEPTH_8U, 1);
  cvSet(disparityImg, cvScalar(0));
}

StereoServer::~StereoServer()
{
  cvReleaseImage(&disparityImg);
  if(doDisplay)
  {
    cvDestroyWindow("left");
    cvDestroyWindow("right");
    cvDestroyWindow("disparity");
  }
}

void StereoServer::setupMyIceCommunication()
{
  Ice::Identity id;
  id.name = iceStereoName;
  id.category = "StereoServer";
  getObjectAdapter()->add(new StereoServerI(this), id);
}

void StereoServer::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  map<string,string>::const_iterator it;
  bool haveStereoConfig = false;

  // first let the base classes configure themselves
  configureVideoCommunication(_config);

  if((it = _config.find("--camids")) != _config.end())
  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  }

  // note: it is ok to not specify these, defaults will be chosen in that case
  if((it = _config.find("--stereoconfig")) != _config.end())
  {
    string file = it->second;
    stereoCam.ReadSVSCalib(file);
    stereoCam.SetInputImageSize(cvSize(STEREO_WIDTH, STEREO_HEIGHT));
    stereoCam.SetupImageRectification();
    haveStereoConfig = true;
  }

  if((it = _config.find("--stereoname")) != _config.end())
  {
    iceStereoName = it->second;
  }

  if((it = _config.find("--display")) != _config.end())
  {
    doDisplay = true;
    cvNamedWindow("left", 1);
    cvNamedWindow("right", 1);
    cvNamedWindow("disparity", 1);
  }

  // sanity checks: Have all important things be configured? Is the
  // configuration consistent?
  if(camIds.size() != 2)
    throw runtime_error(exceptionMessage(__HERE__, "need exactly 2 camera IDs"));
  if(iceStereoName.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no stereo server name given"));
  if(!haveStereoConfig)
    throw runtime_error(exceptionMessage(__HERE__, "no stereo config file given"));
}

void StereoServer::start()
{
  startVideoCommunication(*this);
  setupMyIceCommunication();
}

void StereoServer::getPoints(vector<VisionData::SurfacePoint> &points)
{
  lockComponent();

  // first count how many points we will have
  int cnt = 0;
  for(int y = 0; y < disparityImg->height; y += 1)
    for(int x = 0; x < disparityImg->width; x += 1)
    {
      unsigned char d = *Video::cvAccessImageData(disparityImg, x, y);
      if(d != 0)
        cnt++;
    }

  Pose3 global_left_pose;
  // get from relative left pose to global left pose
  transform(stereoCam.pose, stereoCam.cam[LEFT].pose, global_left_pose);

  points.resize(cnt);
  cnt = 0;
  for(int y = 0; y < disparityImg->height; y += 1)
    for(int x = 0; x < disparityImg->width; x += 1)
    {
      unsigned char d = *Video::cvAccessImageData(disparityImg, x, y);
      if(d != 0)
      {
        stereoCam.ReconstructPoint((double)x, (double)y, (double)d,
           points[cnt].p.x, points[cnt].p.y, points[cnt].p.z);
        // now get from left cam coord sys to global coord sys
        points[cnt].p = transform(global_left_pose, points[cnt].p);
        cnt++;
      }
    }
/*
  // HACK: return poinst of calibration pattern
  points.push_back(vector3(0.000, 0.000, 0.000));
  points.push_back(vector3(0.240, 0.000, 0.000));
  points.push_back(vector3(0.240, 0.120, 0.000));
  points.push_back(vector3(0.200, 0.160, 0.000));
  points.push_back(vector3(0.000, 0.160, 0.000));
  // tea box
  points.push_back(vector3(0.000, 0.000, 0.073));
  points.push_back(vector3(0.160, 0.000, 0.073));
  points.push_back(vector3(0.160, 0.065, 0.073));
  points.push_back(vector3(0.000, 0.065, 0.073));
  points.push_back(vector3(0.000, 0.000, 0.000));
  points.push_back(vector3(0.160, 0.000, 0.000));
  points.push_back(vector3(0.160, 0.065, 0.000));
  points.push_back(vector3(0.000, 0.065, 0.000));
  // HACK END
*/
  unlockComponent();
}

void StereoServer::getPointsInSOI(const VisionData::SOI &soi,
    std::vector<VisionData::SurfacePoint> &points)
{
  lockComponent();

  Pose3 global_left_pose;
  // get from relative left pose to global left pose
  transform(stereoCam.pose, stereoCam.cam[LEFT].pose, global_left_pose);

  points.resize(0);
  for(int y = 0; y < disparityImg->height; y += 1)
    for(int x = 0; x < disparityImg->width; x += 1)
    {
      unsigned char d = *Video::cvAccessImageData(disparityImg, x, y);
      if(d != 0)
      {
        Vector3 p;
        stereoCam.ReconstructPoint((double)x, (double)y, (double)d,
           p.x, p.y, p.z);
        // now get from left cam coord sys to global coord sys
        p = transform(global_left_pose, p);
        if(pointInsideSOI(soi, p))
        {
          VisionData::SurfacePoint sp;
          sp.p = p;
          points.push_back(sp);
        }
      }
    }

  unlockComponent();
}

void StereoServer::runComponent()
{
  vector<Video::Image> images;
  IplImage *grey[2] = {0, 0}, *rect[2] = {0, 0};
  IplImage *rawDisp;

  for(int i = LEFT; i <= RIGHT; i++)
    rect[i] = cvCreateImage(cvSize(STEREO_WIDTH, STEREO_HEIGHT), IPL_DEPTH_8U, 1);
  rawDisp = cvCreateImage(cvSize(STEREO_WIDTH, STEREO_HEIGHT), IPL_DEPTH_8U, 1);

  while(isRunning())
  {
    getScaledImages(STEREO_WIDTH, STEREO_HEIGHT, images);
    assert(images.size() == 2);
    for(int i = LEFT; i <= RIGHT; i++)
    {
      grey[i] = convertImageToIplGray(images[i]);
      stereoCam.RectifyImage(grey[i], rect[i], i);
    }

    cvSet(rawDisp, cvScalar(0));
    census.setImages(rect[LEFT], rect[RIGHT]);
    census.match();
    // in case we are interested how blazingly fast the matching is :)
    // census.printTiming();
    census.getDisparityMap(rawDisp);
    // HACK: clear the borders of the disparity image to get rid of odd values
    cvRectangle(rawDisp, cvPoint(0, 0),
        cvPoint(LEFT_CLEAR_BORDER_WIDTH -1, STEREO_HEIGHT - 1), cvScalarAll(0),
        CV_FILLED);
    cvRectangle(rawDisp, cvPoint(STEREO_WIDTH - RIGHT_CLEAR_BORDER_WIDTH, 0),
        cvPoint(STEREO_WIDTH - 1, STEREO_HEIGHT - 1), cvScalarAll(0),
        CV_FILLED);

    lockComponent();
    //cvSmooth(rawDisp, disparityImg, CV_MEDIAN, 5);
    cvCopy(rawDisp, disparityImg);
    unlockComponent();

    // use OpenCV stereo match
    //lockComponent();
    //stereoCam.DisparityImage(rect[LEFT], rect[RIGHT], disparityImg);
    //unlockComponent();
   
    if(doDisplay)
    {
      cvShowImage("left", rect[LEFT]);
      cvShowImage("right", rect[RIGHT]);
      cvShowImage("disparity", disparityImg);
      cvWaitKey(10);
    }

    for(int i = LEFT; i <= RIGHT; i++)
      cvReleaseImage(&grey[i]);

    sleepComponent(50);
  }

  for(int i = LEFT; i <= RIGHT; i++)
    cvReleaseImage(&rect[i]);
  cvReleaseImage(&rawDisp);
}

}

