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
  for(int i = LEFT; i <= RIGHT; i++)
  {
    colorImg[i] = cvCreateImage(cvSize(STEREO_WIDTH, STEREO_HEIGHT), IPL_DEPTH_8U, 3);
    rectColorImg[i] = cvCreateImage(cvSize(STEREO_WIDTH, STEREO_HEIGHT), IPL_DEPTH_8U, 3);
    rectGreyImg[i] = cvCreateImage(cvSize(STEREO_WIDTH, STEREO_HEIGHT), IPL_DEPTH_8U, 1);
  }
  disparityImg = cvCreateImage(cvSize(STEREO_WIDTH, STEREO_HEIGHT), IPL_DEPTH_8U, 1);
  cvSet(disparityImg, cvScalar(0));
  // normally it's a good idea to do median filering on the disparity image
  medianSize = 5;
}

StereoServer::~StereoServer()
{
  for(int i = LEFT; i <= RIGHT; i++)
  {
    cvReleaseImage(&colorImg[i]);
    cvReleaseImage(&rectColorImg[i]);
    cvReleaseImage(&rectGreyImg[i]);
  }
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

  if((it = _config.find("--median")) != _config.end())
  {
    istringstream str(it->second);
    str >> medianSize;
    if(medianSize < 0)
      medianSize = 0;
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

  Pose3 global_left_pose;
  // get from relative left pose to global left pose
  transform(stereoCam.pose, stereoCam.cam[LEFT].pose, global_left_pose);

  points.resize(0);
  for(int y = 0; y < disparityImg->height; y++)
    for(int x = 0; x < disparityImg->width; x++)
    {
      unsigned char d = *Video::cvAccessImageData(disparityImg, x, y);
      if(d != 0)
      {
        VisionData::SurfacePoint p;
        stereoCam.ReconstructPoint((double)x, (double)y, (double)d,
           p.p.x, p.p.y, p.p.z);
        // now get from left cam coord sys to global coord sys
        p.p = transform(global_left_pose, p.p);
        VisionData::ColorRGB *c = (VisionData::ColorRGB*)Video::cvAccessImageData(rectColorImg[LEFT], x, y);
        p.c = *c;
        points.push_back(p);
      }
    }

  /*// HACK: project the world origin (e.g. the bottom legt corner of a tey box)
  // int left and right recifiec image
  Vector3 P = vector3(0.0, 0.0, 0.0), Q;
  Vector2 p[2];
  cout << "P left right Q ";
  cout << P << " ";
  P = transformInverse(global_left_pose, P);
  for(int i = LEFT; i <= RIGHT; i++)
    stereoCam.ProjectPoint(P.x, P.y, P.z, p[i].x, p[i].y, i);
  // note: gpustereo scales disparities by a fixed factor of 4, so we have to do
  // the same
  stereoCam.ReconstructPoint(p[LEFT].x, p[LEFT].y, 4.*(p[LEFT].x - p[RIGHT].x),
      Q.x, Q.y, Q.z);
  Q = transform(global_left_pose, Q);
  cout << p[LEFT] << " " << p[RIGHT] << " " << Q << endl;
  
  //double x = 100, y = 100, u, v;
  //stereoCam.RectifyPoint(x, y, u, v, LEFT);
  //cout << "[" << x << " " << y << "] rect [" << u << " " << v << "]" << endl;

  Vector2 a = vector2(stereoCam.cam[LEFT].fx*P.x/P.z + stereoCam.cam[LEFT].cx,
                      stereoCam.cam[LEFT].fy*P.y/P.z + stereoCam.cam[LEFT].cy);
  Vector2 b;
  stereoCam.RectifyPoint(a.x, a.y, b.x, b.y, LEFT);
  cout << "project to original left, then rectify:\n" << a << " " << b << endl;
  cout << "project to rectified left:\n" << p[LEFT] << endl;
  */
  /*
P left right Q [0 0 0] [65.8446 63.4946] [6.51125 63.4946] [-1.11022e-16 -1.11022e-16 1.11022e-16]
project to original left, then rectify:
[75.6809 61.9228] [75.3184 62.3205]
project to rectified left:
[65.8446 63.4946]

with SVS library:
x = 75.6809;
y = 61.9228;
sourceObject->RectImagePoint(&u, &v, x, y, svsLEFT);
cout << "[" << x << " " << y << "] rect [" << u << " " << v << "]" << endl;
[75.6809 61.9228] rect [75.3098 62.3134]

   */
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
  for(int y = 0; y < disparityImg->height; y++)
    for(int x = 0; x < disparityImg->width; x++)
    {
      unsigned char d = *Video::cvAccessImageData(disparityImg, x, y);
      if(d != 0)
      {
        VisionData::SurfacePoint p;
        stereoCam.ReconstructPoint((double)x, (double)y, (double)d,
           p.p.x, p.p.y, p.p.z);
        // now get from left cam coord sys to global coord sys
        p.p = transform(global_left_pose, p.p);
        if(pointInsideSOI(soi, p.p))
        {
          VisionData::ColorRGB *c = (VisionData::ColorRGB*)Video::cvAccessImageData(rectColorImg[LEFT], x, y);
          p.c = *c;
          points.push_back(p);
        }
      }
    }

  unlockComponent();
}

void StereoServer::runComponent()
{
  vector<Video::Image> images;

  while(isRunning())
  {
    getScaledImages(STEREO_WIDTH, STEREO_HEIGHT, images);
    assert(images.size() == 2);

    lockComponent();
    for(int i = LEFT; i <= RIGHT; i++)
    {
      convertImageToIpl(images[i], &colorImg[i]);
      stereoCam.RectifyImage(colorImg[i], rectColorImg[i], i);
      cvCvtColor(rectColorImg[i], rectGreyImg[i], CV_RGB2GRAY);
      // HACK
      //cvSaveImage(i == LEFT ? "orig-L.jpg" : "orig-R.jpg", colorImg[i]);
      //cvSaveImage(i == LEFT ? "rect-L.jpg" : "rect-R.jpg", rectColorImg[i]);
    }

    cvSet(disparityImg, cvScalar(0));
    census.setImages(rectGreyImg[LEFT], rectGreyImg[RIGHT]);
    census.match();
    // in case we are interested how blazingly fast the matching is :)
    // census.printTiming();
    census.getDisparityMap(disparityImg);
    // HACK: clear the borders of the disparity image to get rid of odd values
    cvRectangle(disparityImg, cvPoint(0, 0),
        cvPoint(LEFT_CLEAR_BORDER_WIDTH -1, STEREO_HEIGHT - 1), cvScalarAll(0),
        CV_FILLED);
    cvRectangle(disparityImg, cvPoint(STEREO_WIDTH - RIGHT_CLEAR_BORDER_WIDTH, 0),
        cvPoint(STEREO_WIDTH - 1, STEREO_HEIGHT - 1), cvScalarAll(0),
        CV_FILLED);

    if(medianSize > 0)
    {
      IplImage *tmp = cvCloneImage(disparityImg);
      cvSmooth(disparityImg, tmp, CV_MEDIAN, medianSize);
      swap(disparityImg, tmp);
      cvReleaseImage(&tmp);
    }
    unlockComponent();

    // use OpenCV stereo match
    //lockComponent();
    //stereoCam.DisparityImage(greyImg[LEFT], greyImg[RIGHT], disparityImg);
    //unlockComponent();
 
    if(doDisplay)
    {
      cvShowImage("left", rectColorImg[LEFT]);
      cvShowImage("right", rectColorImg[RIGHT]);
      cvShowImage("disparity", disparityImg);
      cvWaitKey(10);
    }

    sleepComponent(200);
  }
}

}

