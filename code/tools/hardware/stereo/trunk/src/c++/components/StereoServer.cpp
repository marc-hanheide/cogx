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

void StereoServerI::getPoints(bool transformToGlobal, VisionData::SurfacePointSeq& points, const Ice::Current&)
{
  stereoSrv->getPoints(transformToGlobal, points);
}

void StereoServerI::getPointsInSOI(bool transformToGlobal, const VisionData::SOIPtr &soi,
    VisionData::SurfacePointSeq& points, const Ice::Current&)
{
  stereoSrv->getPointsInSOI(transformToGlobal, *soi, points);
}

void StereoServerI::getRectImage(Ice::Int side, Video::Image& image, const Ice::Current&)
{
  stereoSrv->getRectImage(side, image);
}

void StereoServerI::getDisparityImage(Video::Image& image, const Ice::Current&)
{
  stereoSrv->getDisparityImage(image);
}

StereoServer::StereoServer()
{
  census = 0;
  iceStereoName = "";
  iceStereoPort = cdl::CPPSERVERPORT;
  // these are quite small, but work nice'n'fast
  stereoWidth = 160;
  stereoHeight = 120;
  doDisplay = false;
  logImages = false;
  // normally it's a good idea to do median filering on the disparity image
  medianSize = 5;
  // 50 is a good start for not-too-strong disparity
  maxDisp = 50;
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

  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

  if((it = _config.find("--camids")) != _config.end())
  {
    istringstream str(it->second);
    int id;
    while(str >> id)
      camIds.push_back(id);
  }

  if((it = _config.find("--stereoconfig")) != _config.end())
  {
    string file = it->second;
    stereoCam.ReadSVSCalib(file);
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

  if((it = _config.find("--imgsize")) != _config.end())
  {
    istringstream str(it->second);
    str >> stereoWidth >> stereoHeight;
  }

  if((it = _config.find("--median")) != _config.end())
  {
    istringstream str(it->second);
    str >> medianSize;
    if(medianSize < 0)
      medianSize = 0;
  }

  if((it = _config.find("--logimages")) != _config.end())
  {
    logImages = true;
  }

  if((it = _config.find("--maxdisp")) != _config.end())
  {
    istringstream str(it->second);
    str >> maxDisp;
  }

  // if no extra image size was given, use the original grabbed image size
  if(stereoWidth == 0 || stereoHeight == 0)
  {
    stereoWidth = stereoCam.cam[LEFT].width;
    stereoHeight = stereoCam.cam[LEFT].height;
  }
  // now set the input image size to be used by stereo matching
  stereoCam.SetInputImageSize(cvSize(stereoWidth, stereoHeight));
  stereoCam.SetupImageRectification();

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
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

  setupMyIceCommunication();

  /* note: this is not needed as we get camera parameters with every image from
   * the video server.
   * However in the long run, it might be good
  // get informed about camera movements
  addChangeFilter(createLocalTypeFilter<CameraParametersWrapper>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoServer>(this,
        &StereoServer::receiveCameraParameters));

  addChangeFilter(createLocalTypeFilter<CameraParametersWrapper>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<StereoServer>(this,
        &StereoServer::receiveCameraParameters));*/

  // allocate all our various images
  for(int i = LEFT; i <= RIGHT; i++)
  {
    colorImg[i] = cvCreateImage(cvSize(stereoWidth, stereoHeight), IPL_DEPTH_8U, 3);
    rectColorImg[i] = cvCreateImage(cvSize(stereoWidth, stereoHeight), IPL_DEPTH_8U, 3);
    rectGreyImg[i] = cvCreateImage(cvSize(stereoWidth, stereoHeight), IPL_DEPTH_8U, 1);
  }
  disparityImg = cvCreateImage(cvSize(stereoWidth, stereoHeight), IPL_DEPTH_8U, 1);
  cvSet(disparityImg, cvScalar(0));

  // NOTE: stupid polling runloop is still necessary
  // push interface does not work for stereo server, probably some threading
  // issue with cuda or whatever
  // start receiving images pushed by the video server
  //videoServer->startReceiveImages(getComponentID().c_str(), camIds, stereoWidth,
  //    stereoHeight);
}

void StereoServer::getPoints(bool transformToGlobal, vector<VisionData::SurfacePoint> &points)
{
  lockComponent();

  Pose3 global_left_pose;
  if(transformToGlobal)
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
        if(transformToGlobal)
          // now get from left cam coord sys to global coord sys
          p.p = transform(global_left_pose, p.p);
        VisionData::ColorRGB *c = (VisionData::ColorRGB*)Video::cvAccessImageData(rectColorImg[LEFT], x, y);
        p.c = *c;
        points.push_back(p);
      }
    }

  unlockComponent();
}

void StereoServer::getPointsInSOI(bool transformToGlobal, const VisionData::SOI &soi,
    vector<VisionData::SurfacePoint> &points)
{
  lockComponent();

  Pose3 global_left_pose;
  if(transformToGlobal)
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
        if(transformToGlobal)
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

void StereoServer::getRectImage(int side, Video::Image& image)
{
  assert(side == LEFT || side == RIGHT);
  lockComponent();
  convertImageFromIpl(rectColorImg[side], image);
  initCameraParameters(image.camPars);
  image.camPars.id = side;
  image.camPars.width = stereoCam.cam[side].width;
  image.camPars.height = stereoCam.cam[side].height;
  image.camPars.fx = stereoCam.cam[side].proj[0][0];
  image.camPars.fy = stereoCam.cam[side].proj[1][1];
  image.camPars.cx = stereoCam.cam[side].proj[0][2];
  image.camPars.cy = stereoCam.cam[side].proj[1][2];
  changeImageSize(image.camPars, stereoCam.inImgSize.width, stereoCam.inImgSize.height);
  // get from relative left pose to global left pose
  Pose3 global_pose;
  transform(stereoCam.pose, stereoCam.cam[side].pose, global_pose);
  image.camPars.pose = global_pose;
  image.camPars.time = getCASTTime();
  unlockComponent();
}

void StereoServer::getDisparityImage(Video::Image& image)
{
	lockComponent();
	convertImageFromIpl(disparityImg, image);
	unlockComponent();
}

void StereoServer::receiveImages(const vector<Video::Image>& images)
{
  lockComponent();

  // assuming that the left camera pose is equal to stereo head pose,
  // save the head pose for this pair of images
  stereoCam.pose = images[LEFT].camPars.pose;

  for(int i = LEFT; i <= RIGHT; i++)
  {
    convertImageToIpl(images[i], &colorImg[i]);
    stereoCam.RectifyImage(colorImg[i], rectColorImg[i], i);
    cvCvtColor(rectColorImg[i], rectGreyImg[i], CV_RGB2GRAY);
  }

  cvSet(disparityImg, cvScalar(0));
  census->setImages(rectGreyImg[LEFT], rectGreyImg[RIGHT]);
  census->match();
  // in case we are interested how blazingly fast the matching is :)
  // census->printTiming();
  census->getDisparityMap(disparityImg);

  if(medianSize > 0)
  {
    IplImage *tmp = cvCloneImage(disparityImg);
    cvSmooth(disparityImg, tmp, CV_MEDIAN, medianSize);
    swap(disparityImg, tmp);
    cvReleaseImage(&tmp);
  }

  // use OpenCV stereo match (which is pretty bad)
  //stereoCam.DisparityImage(greyImg[LEFT], greyImg[RIGHT], disparityImg);

  unlockComponent();

  if(logImages)
  {
    for(int i = LEFT; i <= RIGHT; i++)
    {
      cvSaveImage(i == LEFT ? "stereoserver-orig-L.jpg" : "stereoserver-orig-R.jpg", colorImg[i]);
      cvSaveImage(i == LEFT ? "stereoserver-rect-L.jpg" : "stereoserver-rect-R.jpg", rectColorImg[i]);
    }
    cvSaveImage("stereoserver-disp.png", disparityImg);
  }

  if(doDisplay)
  {
    cvShowImage("left", rectColorImg[LEFT]);
    cvShowImage("right", rectColorImg[RIGHT]);
    cvShowImage("disparity", disparityImg);
    cvWaitKey(10);
  }
}

void StereoServer::runComponent()
{
  // allocate the actual stereo matcher with given max disparity
  census = new CensusGPU(maxDisp);

  // NOTE: stupid polling runloop is still necessary
  // push interface does not work for stereo server, probably some threading
  // issue with cuda or whatever
  vector<Video::Image> images;
  while(isRunning())
  {
    // HACK: we should actually sent the list of cam ids and not just assume
    // that the video server has precisely two cameras in the right order
    videoServer->getScaledImages(stereoWidth, stereoHeight, images);
    receiveImages(images);
    sleepComponent(100);
  }
  delete census;
}

}

