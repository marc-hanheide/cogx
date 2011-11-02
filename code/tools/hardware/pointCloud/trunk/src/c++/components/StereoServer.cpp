/**
 * @file StereoServer.cpp
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the cast-framework implemented as stereo server.
 */

#include <cast/core/CASTUtils.hpp>
#include <limits.h>
#include <sstream>
#include <opencv/highgui.h>
#include <VideoUtils.h>
#include "StereoServer.h"

// a useful default max disparity
#define DEF_MAX_DISP 64

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

#ifdef __APPLE__ //nah: OS X doesn't have CLOCK_MONOTONIC

static double gethrtime_d()
{
	timeval tv;
	int ret = gettimeofday(&tv, NULL);  
	if(ret != 0) {
    	return 0;
	}
	return (double)tv.tv_sec + 1e-6*(double)tv.tv_usec;
}


#else

static double gethrtime_d()
{
  struct timespec ts;
  int ret;
#ifdef CLOCK_MONOTONIC_HR
  ret = clock_gettime(CLOCK_MONOTONIC_HR, &ts);
#else
  ret = clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
  if(ret != 0)
    return 0;
  return (double)ts.tv_sec + 1e-9*(double)ts.tv_nsec;
}

#endif


StereoServer::StereoServer()
{
#ifdef HAVE_GPU_STEREO
  census = 0;
  // normally it's a good idea to do median filering on the disparity image
  medianSize = 5;
#endif
  doDisplay = false;
  logImages = false;
}

StereoServer::~StereoServer()
{
#ifdef HAVE_GPU_STEREO
  delete census;
#endif
  for(size_t i = 0; i < stereoCams.size(); i++)
    delete stereoCams[i];
  if(doDisplay)
  {
    cvDestroyWindow("left");
    cvDestroyWindow("right");
    cvDestroyWindow("disparity");
  }
}

void StereoServer::configure(const map<string,string> & _config) throw(runtime_error)
{
  PointCloudServer::configure(_config);     // configure base class

  map<string,string>::const_iterator it;
  string stereoCalibFile;

  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }
  else throw runtime_error(exceptionMessage(__HERE__, "no video config file (videoname) given"));

  if((it = _config.find("--imgsize")) != _config.end())							/// TODO nicht notwendige mit log oder debug Meldungen ausstatten
  {
    istringstream str(it->second);
    int w, h;
    while(str >> w >> h)
      stereoSizes.push_back(cvSize(w, h));
  } else printf("StereoServer::configure: Warning: No imagesize given.\n");

  if((it = _config.find("--maxdisp")) != _config.end())
  {
    istringstream str(it->second);
    int disp;
    while(str >> disp)
      maxDisps.push_back(disp);
  } else printf("StereoServer::configure: Warning: No maximum disparity given.\n");

#ifdef HAVE_GPU_STEREO
  if((it = _config.find("--median")) != _config.end())
  {
    istringstream str(it->second);
    str >> medianSize;
    if(medianSize <= 1)
      medianSize = 0;
    // NOTE: OpenCV can do median filtering of float images only for filter sizes 3 and 5
    if(medianSize > 5)
    {
      log("median filtering size must be <= 5; setting to 5");
      medianSize = 5;
    }
    if(medianSize%2 != 1)
    {
      log("median filtering size must be odd number, disabling median filtering");
      medianSize = 0;
    }
  }
#endif

  if((it = _config.find("--display")) != _config.end())
  {
    doDisplay = true;
  }

  if((it = _config.find("--logimages")) != _config.end())
  {
    logImages = true;
  }
  
  if((it = _config.find("--stereoconfig_xml")) != _config.end())
  {
    istringstream str(it->second);
    string fileLeft, fileRight;
    str >> fileLeft;
    str >> fileRight;
      
    // if no image size was specified, use the original image size as given by the stereo config
    if(stereoSizes.empty())
    {
      StereoCamera *sc = new StereoCamera();
      sc->ReadFromXML(fileLeft, 0, false);    // 0 = left
      sc->ReadFromXML(fileRight, 1, false);   // 1 = right
      sc->SetupImageRectification();
      stereoCams.push_back(sc);
      stereoSizes.push_back(cvSize(sc->cam[0].width, sc->cam[0].height));
    }
    else  // else: we have a set of resolutions, create a stereo camera for each
    {
      for(size_t i = 0; i < stereoSizes.size(); i++)
      {
        StereoCamera *sc = new StereoCamera();
        sc->ReadFromXML(fileLeft, 0, true);    // 0 = left
        sc->ReadFromXML(fileRight, 1, true);   // 1 = right
        sc->SetInputImageSize(stereoSizes[i]);
        sc->SetupImageRectification();
        stereoCams.push_back(sc);
      }
    }
  } else throw runtime_error(exceptionMessage(__HERE__, "no stereo config files given."));
  
  if(!maxDisps.empty())
  {
    if(maxDisps.size() != stereoCams.size())
      throw runtime_error(exceptionMessage(__HERE__, "need max disparity for each stereo resolution"));
    for(size_t i = 0; i < stereoCams.size(); i++)
      stereoCams[i]->SetDisparityRange(0, maxDisps[i]);
  }

  imgSets.resize(stereoSizes.size());
  for(size_t i = 0; i < stereoSizes.size(); i++)
  {
    // TODO: disp format depends on OpenCV stereo algorithm in stereo camera
    imgSets[i].init(stereoSizes[i], IPL_DEPTH_32F);
  }

  if(camIds.size() != 2)
    throw runtime_error(exceptionMessage(__HERE__, "need exactly 2 camera IDs"));

#ifdef HAVE_GPU_STEREO
  // allocate the actual stereo matcher with given max disparity
  census = new CensusGPU();
#endif
}


void StereoServer::start()
{
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

  // NOTE: stupid polling runloop is still necessary
  // push interface does not work for stereo server, probably some threading
  // issue with cuda or whatever
  // start receiving images pushed by the video server
  //videoServer->startReceiveImages(getComponentID().c_str(), camIds, stereoSizes[0].width,
  //    stereoSizes[0].height);

  if(doDisplay)
  {
    cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
    //cvWaitKey(10);
  }
  
  PointCloudServer::start();
}

int StereoServer::findClosestResolution(int imgWidth)
{
  assert(stereoSizes.size() > 0);
  int d, d_min = INT_MAX;
  int idx = 0;
  for(size_t i = 0; i < stereoSizes.size(); i++)
  {
    d = abs(imgWidth - stereoSizes[i].width);
    if(d < d_min)
    {
      d_min = d;
      idx = (int)i;
    }
  }
  return idx;
}

void StereoServer::getPoints(bool transformToGlobal, int imgWidth, vector<PointCloud::SurfacePoint> &points, bool complete)
{
  lockComponent();

  int res = findClosestResolution(imgWidth);
  StereoCamera *stereoCam = stereoCams[res];
  ImageSet &imgSet = imgSets[res];

  vector<Video::Image> images;
  double t1 = gethrtime_d();
  // HACK: we should actually send the list of cam ids and not just assume
  // that the video server has precisely two cameras in the right order
  videoServer->getScaledImages(stereoSizes[res].width, stereoSizes[res].height, images);
  double t2 = gethrtime_d();
  stereoProcessing(stereoCam, imgSet, images);
  double t3 = gethrtime_d();

  Pose3 global_left_pose;
  if(transformToGlobal)
  {
    Pose3 ideal_pose, rel_pose;
    setIdentity(ideal_pose);
    // pose of ideal left camera w.r.t. to real left camera
    // the pose is a rotation given by the rectification matrix
    setRow33(ideal_pose.rot, (double*)stereoCam->cam[LEFT].rect);
    // get from ideal left pose to real left pose
    transform(stereoCam->cam[LEFT].pose, ideal_pose, rel_pose);
    // get from relative left pose to global left pose
    transform(stereoCam->pose, rel_pose, global_left_pose);
  }

  points.resize(0);
  for(int y = 0; y < imgSet.disparityImg->height; y++)
    for(int x = 0; x < imgSet.disparityImg->width; x++)
    {
      float d = *((float*)Video::cvAccessImageData(imgSet.disparityImg, x, y));
      PointCloud::SurfacePoint p;
      if(stereoCam->ReconstructPoint((double)x, (double)y, (double)d, p.p.x, p.p.y, p.p.z))
      {
        if(transformToGlobal)
          // now get from left cam coord sys to global coord sys
          p.p = transform(global_left_pose, p.p);
        
        cogx::Math::ColorRGB *c = (cogx::Math::ColorRGB*)Video::cvAccessImageData(imgSet.rectColorImg[LEFT], x, y);
        p.c = *c;
        points.push_back(p);
      }
      else if(complete)
      {
        /*stereoCam->ReconstructPoint((double)x, (double)y, 64,
           p.p.x, p.p.y, p.p.z);
        if(transformToGlobal)
          // now get from left cam coord sys to global coord sys
          p.p = transform(global_left_pose, p.p);*/
        p.p = vector3(0., 0., 0.);
        cogx::Math::ColorRGB *c =
          (cogx::Math::ColorRGB*)Video::cvAccessImageData(imgSet.rectColorImg[LEFT], x, y);
        p.c = *c;
        points.push_back(p);
      }
    }
  double t4 = gethrtime_d();
  /*log("run time: get images: %lf, stereo: %lf, reconstruct: %lf - total: %lf / frame rate: %lf",
    t2 - t1, t3 - t2, t4 - t3, t4 - t1, 1./(t4 - t1));*/

  unlockComponent();
}

void StereoServer::getRectImage(int side, int imgWidth, Video::Image& image)
{
  assert(side == LEFT || side == RIGHT);

  lockComponent();

  int res = findClosestResolution(imgWidth);
  StereoCamera *stereoCam = stereoCams[res];
  ImageSet &imgSet = imgSets[res];

  vector<Video::Image> images;
  // HACK: we should actually send the list of cam ids and not just assume
  // that the video server has precisely two cameras in the right order
  videoServer->getScaledImages(stereoSizes[res].width, stereoSizes[res].height, images);
  convertImageToIpl(images[side], &imgSet.colorImg[side]);
  stereoCam->RectifyImage(imgSet.colorImg[side], imgSet.rectColorImg[side], side);

  convertImageFromIpl(imgSet.rectColorImg[side], image);
  initCameraParameters(image.camPars);
  image.camPars.id = camIds[side];
  image.camPars.width = stereoCam->cam[side].width;
  image.camPars.height = stereoCam->cam[side].height;
  image.camPars.fx = stereoCam->cam[side].proj[0][0];
  image.camPars.fy = stereoCam->cam[side].proj[1][1];
  image.camPars.cx = stereoCam->cam[side].proj[0][2];
  image.camPars.cy = stereoCam->cam[side].proj[1][2];
  changeImageSize(image.camPars, stereoCam->inImgSize.width, stereoCam->inImgSize.height);

  Pose3 ideal_pose, rel_pose, global_pose;
  setIdentity(ideal_pose);
  // pose of ideal left/right camera w.r.t. to actual left/right camera
  // the pose is a rotation given by the rectification matrix
  setRow33(ideal_pose.rot, (double*)stereoCam->cam[side].rect);
  // get from ideal left/right pose to real left/right pose
  transform(stereoCam->cam[side].pose, ideal_pose, rel_pose);
  // get from relative left/right pose to global left/right pose
  transform(stereoCam->pose, rel_pose, global_pose);
  image.camPars.pose = global_pose;

  image.camPars.time = getCASTTime();

  unlockComponent();
}

bool StereoServer::getCameraParameters(int side, Video::CameraParameters& camPars)
{
  if (side != LEFT && side != RIGHT)
    return false;

  lockComponent(); // TODO: CASTComponent::Lock lock(this);

  StereoCamera *stereoCam = stereoCams[0];
  initCameraParameters(camPars);
  camPars.id = camIds[side];
  camPars.width = stereoCam->cam[side].width;
  camPars.height = stereoCam->cam[side].height;
  camPars.fx = stereoCam->cam[side].proj[0][0];
  camPars.fy = stereoCam->cam[side].proj[1][1];
  camPars.cx = stereoCam->cam[side].proj[0][2];
  camPars.cy = stereoCam->cam[side].proj[1][2];

  Pose3 ideal_pose, rel_pose, global_pose;
  setIdentity(ideal_pose);
  // pose of ideal left/right camera w.r.t. to actual left/right camera
  // the pose is a rotation given by the rectification matrix
  setRow33(ideal_pose.rot, (double*)stereoCam->cam[side].rect);
  // get from ideal left/right pose to real left/right pose
  transform(stereoCam->cam[side].pose, ideal_pose, rel_pose);
  // get from relative left/right pose to global left/right pose
  transform(stereoCam->pose, rel_pose, global_pose);
  camPars.pose = global_pose;
  camPars.time = getCASTTime();

  unlockComponent(); // TODO: remove

  return true;
}

// @author: mmarko
// 2011-10-12 NOTE: We override the default function because isPointVisible
// didn't work with the default PointCloudServer::isPointVisible. The point was
// correctly visible in camera with camPars[0], but invisible in camera with
// camPars[1].  CameraParameters returned by getCameraParameters() seem to work
// better.
bool StereoServer::isPointVisible(const cogx::Math::Vector3& point)
{
  Video::CameraParameters camPars;

  if (!getCameraParameters(LEFT, camPars) || ! Video::isPointVisible(camPars, point)) {
      //log("Point is NOT visible in camera %d", camPars.id);
      return false;
  }
  //log("Point is visible in camera %d", camPars.id);

  if (!getCameraParameters(RIGHT, camPars) || ! Video::isPointVisible(camPars, point)) {
      //log("Point is NOT visible in camera %d", camPars.id);
      return false;
  }
  //log("Point is visible in camera %d", camPars.id);

  return true;
}

void StereoServer::getDisparityImage(int imgWidth, Video::Image& image)        /// TODO Reimplement?
{
  throw runtime_error(exceptionMessage(__HERE__, "not implemented"));
	/*lockComponent();
	convertImageFromIpl(disparityImg, image);
	unlockComponent();*/
}

void StereoServer::stereoProcessing(StereoCamera *stereoCam, ImageSet &imgSet, const vector<Video::Image>& images)
{
  // assuming that the left camera pose is equal to stereo head pose,
  // save the head pose for this pair of images
  stereoCam->pose = images[LEFT].camPars.pose;

  for(int i = LEFT; i <= RIGHT; i++)
  {
    convertImageToIpl(images[i], &imgSet.colorImg[i]);
    stereoCam->RectifyImage(imgSet.colorImg[i], imgSet.rectColorImg[i], i);
    cvCvtColor(imgSet.rectColorImg[i], imgSet.rectGreyImg[i], CV_RGB2GRAY);
  }
  cvSet(imgSet.disparityImg, cvScalar(0));

#ifdef HAVE_GPU_STEREO
  int res = findClosestResolution(imgSet.disparityImg->width);
  census->setOptions(0,               // min disp
                     maxDisps[res]);  // max disp
  census->setImages(imgSet.rectGreyImg[LEFT], imgSet.rectGreyImg[RIGHT]);
  census->match();
  // in case we are interested how blazingly fast the matching is :)
  // census->printTiming();
  // census->printTiming();
  census->getDisparityMap(imgSet.disparityImg);
  if(medianSize > 0)
  {
    IplImage *tmp = cvCloneImage(imgSet.disparityImg);
    cvSmooth(imgSet.disparityImg, tmp, CV_MEDIAN, medianSize);
    swap(imgSet.disparityImg, tmp);
    cvReleaseImage(&tmp);
  }
#else
  // use OpenCV stereo matching provided inside the stereo camera
  stereoCam->SetMatchingAlgoritm(StereoCamera::SEMI_GLOBAL_BLOCK_MATCH);
  //stereoCam->SetMatchingAlgoritm(StereoCamera::BLOCK_MATCH);
  stereoCam->CalculateDisparity(imgSet.rectGreyImg[LEFT], imgSet.rectGreyImg[RIGHT], imgSet.disparityImg);
#endif

  if(logImages)
  {
    for(int i = LEFT; i <= RIGHT; i++)
    {
      cvSaveImage(i == LEFT ? "stereoserver-orig-L.jpg" : "stereoserver-orig-R.jpg", imgSet.colorImg[i]);
      cvSaveImage(i == LEFT ? "stereoserver-rect-L.jpg" : "stereoserver-rect-R.jpg", imgSet.rectColorImg[i]);
    }
    cvSaveImage("stereoserver-disp.jpg", imgSet.disparityImg);
  }

  if(doDisplay)
  {
    cvShowImage("left", imgSet.rectColorImg[LEFT]);
    cvShowImage("right", imgSet.rectColorImg[RIGHT]);
    cvShowImage("disparity", imgSet.disparityImg);
    cvWaitKey(10);
  }

/*
  double t4 = gethrtime_d();

  log("%s at %d x %d: copy/rectify/convert images: %lf, stereo: %lf, log/display images: %lf - total: %lf / frame rate: %lf",
#ifdef HAVE_GPU_STEREO
      "gpustereo",
#else
      "OpenCV stereo",
#endif
    imgSet.disparityImg->width, imgSet.disparityImg->height,
    t2 - t1, t3 - t2, t4 - t3, t4 - t1, 1./(t4 - t1));*/
}

void StereoServer::receiveImages(const vector<Video::Image>& images)        /// TODO Reimplement?
{
//   lockComponent();
  printf("StereoServer::receiveImages: Warning: Not yet implemented!\n");
//   stereoProcessing(stereoCams[0], imgSets[0], images);
//   unlockComponent();
}

void StereoServer::runComponent()
{}

/*void StereoServer::redraw3D(vector<VisionData::SurfacePoint> &points)
{
  std::ostringstream str;
  str << "function render()\n";
  str << "glDisable(GL_LIGHTING)\n";
  str << "glPointSize(2);\n";
  str << "glBegin(GL_POINTS)\n";
  for(size_t i = 0; i < points.size(); i++)
  {
    str << "glColor(" << (double)((unsigned char)points[i].c.r)/255. << ", "
        << (double)((unsigned char)points[i].c.g)/255. << ", "
        << (double)((unsigned char)points[i].c.b)/255. << ")\n";
    str << "glVertex(" << points[i].p.x << ", " << points[i].p.y << ", " << points[i].p.z << ")\n";
  }
  str << "glEnd()\n";
  str << "end\n";
  m_display.setLuaGlObject(ID_OBJECT_3D, "stereo", str.str());
}*/

}

