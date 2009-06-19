/**
 * @author Michael Zillich
 * @date June 2009
 */

#include <sstream>
#include <opencv/highgui.h>
#include <cast/core/CASTUtils.hpp>
#include <VideoUtils.h>
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

void StereoServerI::getPoints(Stereo::Vector3Seq& points, const Ice::Current&)
{
  stereoSrv->getPoints(points);
}

StereoServer::StereoServer()
{
  iceStereoName = "";
  iceStereoPort = cdl::CPPSERVERPORT;
  doDisplay = false;
  disparityImg = 0;
  disparityImg = cvCreateImage(cvSize(STEREO_WIDTH, STEREO_HEIGHT), IPL_DEPTH_8U, 1);
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

void StereoServer::getPoints(vector<cogx::Math::Vector3> &points)
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
  points.resize(cnt);
  cnt = 0;
  for(int y = 0; y < disparityImg->height; y += 1)
    for(int x = 0; x < disparityImg->width; x += 1)
    {
      unsigned char d = *Video::cvAccessImageData(disparityImg, x, y);
      if(d != 0)
      {
        stereoCam.ReconstructPoint((double)x, (double)y, (double)d,
           points[cnt].x, points[cnt].y, points[cnt].z);
        cnt++;
      }
    }
  unlockComponent();
}

void StereoServer::runComponent()
{
  vector<Video::Image> images;
  IplImage *grey[2] = {0, 0}, *rect[2] = {0, 0};
  int i;

  for(i = LEFT; i <= RIGHT; i++)
    rect[i] = cvCreateImage(cvSize(STEREO_WIDTH, STEREO_HEIGHT), IPL_DEPTH_8U, 1);

  while(isRunning())
  {
    getScaledImages(STEREO_WIDTH, STEREO_HEIGHT, images);
    assert(images.size() == 2);
    for(i = LEFT; i <= RIGHT; i++)
    {
      grey[i] = convertImageToIplGray(images[i]);
      stereoCam.RectifyImage(grey[i], rect[i], i);
    }

    cvSet(disparityImg, cvScalar(0));
    census.setImages(rect[LEFT], rect[RIGHT]);
    census.match();
    // in case we are interested how blazingly fast the matching is :)
    // census.printTiming();
    lockComponent();
    census.getDisparityMap(disparityImg);
    unlockComponent();

    // use OpenCV stereo match
    //stereoCam.DisparityImage(rect[LEFT], rect[RIGHT], disparityImg);
   
    if(doDisplay)
    {
      cvShowImage("left", rect[LEFT]);
      cvShowImage("right", rect[RIGHT]);
      cvShowImage("disparity", disparityImg);
      cvWaitKey(10);
    }

    for(i = LEFT; i <= RIGHT; i++)
      cvReleaseImage(&grey[i]);
  }
  cvReleaseImage(&rect[i]);
}

}

