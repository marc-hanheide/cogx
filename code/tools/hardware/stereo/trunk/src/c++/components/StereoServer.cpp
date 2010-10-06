/**
 * @author Michael Zillich
 * @date June 2009
 */

#include <limits.h>
#include <sstream>
#include <opencv/highgui.h>
#include <cast/core/CASTUtils.hpp>
#include <VideoUtils.h>
#include <VisionUtils.h>
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

/**
 * Returns timespecs x - y as double.
 */
static double timespec_diff(struct timespec *x, struct timespec *y)
{
  /*timespec res = {x->tv_sec - y->tv_sec, x->tv_nsec - y->tv_nsec};
  if(res.tv_nsec >= 1000000000)
  {
    res.tv_nsec -= 1000000000;
    res.tv_sec++;
  }
  if(res.tv_nsec < 0)
  {
    res.tv_nsec += 1000000000;
  }
  return (double)res.tv_sec + 1e-9*(double)res.tv_nsec;*/
  /* TODO: make the above clearer version handle nsec overflows of more than one
   * sec, see below*/
  if(x->tv_nsec < y->tv_nsec)
  {
    int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
    y->tv_nsec -= 1000000000 * nsec;
    y->tv_sec += nsec;
  }
  if(x->tv_nsec - y->tv_nsec > 1000000000)
  {
    int nsec = (x->tv_nsec - y->tv_nsec) / 1000000000;
    y->tv_nsec += 1000000000 * nsec;
    y->tv_sec -= nsec;
  }
  return (double)(x->tv_sec - y->tv_sec) +
    (double)(x->tv_nsec - y->tv_nsec)/1000000000.;
}

void StereoServerI::getPoints(bool transformToGlobal, int imgWidth, VisionData::SurfacePointSeq& points, const Ice::Current&)
{
  stereoSrv->getPoints(transformToGlobal, imgWidth, points, false);
}

void StereoServerI::getCompletePoints(bool transformToGlobal, int imgWidth, VisionData::SurfacePointSeq& points, const Ice::Current&)
{
  stereoSrv->getPoints(transformToGlobal, imgWidth, points, true);
}

void StereoServerI::getRectImage(Ice::Int side, int imgWidth, Video::Image& image, const Ice::Current&)
{
  stereoSrv->getRectImage(side, imgWidth, image);
}

void StereoServerI::getDisparityImage(int imgWidth, Video::Image& image, const Ice::Current&)
{
  stereoSrv->getDisparityImage(imgWidth, image);
}

StereoServer::StereoServer()
{
#ifdef HAVE_GPU_STEREO
  census = 0;
  // normally it's a good idea to do median filering on the disparity image
  medianSize = 5;
  maxDisp = 64;
#endif
  doDisplay = false;
  logImages = false;
}

StereoServer::~StereoServer()
{
  for(size_t i = 0; i < stereoCams.size(); i++)
    delete stereoCams[i];
  if(doDisplay)
  {
    cvDestroyWindow("left");
    cvDestroyWindow("right");
    cvDestroyWindow("disparity");
  }
}

void StereoServer::setupMyIceCommunication()
{
  hStereoServer = new StereoServerI(this);
  registerIceServer<Stereo::StereoInterface, StereoServerI>(hStereoServer);
}

void StereoServer::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  map<string,string>::const_iterator it;
  string stereoCalibFile;

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
    stereoCalibFile = it->second;
  }

  if((it = _config.find("--imgsize")) != _config.end())
  {
    istringstream str(it->second);
    int w, h;
    while(str >> w >> h)
      stereoSizes.push_back(cvSize(w, h));
  }

  if((it = _config.find("--maxdisp")) != _config.end())
  {
    istringstream str(it->second);
#ifdef HAVE_GPU_STEREO
    str >> maxDisp;
#else
    int disp;
    while(str >> disp)
      maxDisps.push_back(disp);
#endif
  }

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

  if(stereoCalibFile.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no stereo config file given"));

  // if no image size was specified, use the original image size as given by the stereo config
  if(stereoSizes.empty())
  {
    StereoCamera *sc = new StereoCamera();
    sc->ReadSVSCalib(stereoCalibFile);
    sc->SetupImageRectification();
    stereoCams.push_back(sc);
    stereoSizes.push_back(cvSize(sc->cam[LEFT].width,
                                 sc->cam[LEFT].height));
  }
  // else: we have a set of resolutions, create a stereo camera for each
  else
  {
    for(size_t i = 0; i < stereoSizes.size(); i++)
    {
      StereoCamera *sc = new StereoCamera();
      sc->ReadSVSCalib(stereoCalibFile);
      // now set the input image size to be used by stereo matching
      sc->SetInputImageSize(stereoSizes[i]);
      sc->SetupImageRectification();
      stereoCams.push_back(sc);
    }
  }

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

  // sanity checks: Have all important things be configured? Is the
  // configuration consistent?
  if(camIds.size() != 2)
    throw runtime_error(exceptionMessage(__HERE__, "need exactly 2 camera IDs"));

  setupMyIceCommunication();
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
    cvNamedWindow("left", 1);
    cvNamedWindow("right", 1);
    cvNamedWindow("disparity", 1);
  }
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

void StereoServer::getPoints(bool transformToGlobal, int imgWidth, vector<VisionData::SurfacePoint> &points,
  bool complete)
{
  lockComponent();

  int res = findClosestResolution(imgWidth);
  StereoCamera *stereoCam = stereoCams[res];
  ImageSet &imgSet = imgSets[res];

  vector<Video::Image> images;
  // HACK: we should actually send the list of cam ids and not just assume
  // that the video server has precisely two cameras in the right order
  videoServer->getScaledImages(stereoSizes[res].width, stereoSizes[res].height, images);
  stereoProcessing(stereoCam, imgSet, images);

  Pose3 global_left_pose;
  if(transformToGlobal)
    // get from relative left pose to global left pose
    transform(stereoCam->pose, stereoCam->cam[LEFT].pose, global_left_pose);

  points.resize(0);
  for(int y = 0; y < imgSet.disparityImg->height; y++)
    for(int x = 0; x < imgSet.disparityImg->width; x++)
    {
      float d = *((float*)Video::cvAccessImageData(imgSet.disparityImg, x, y));
      VisionData::SurfacePoint p;
      if(stereoCam->ReconstructPoint((double)x, (double)y, (double)d,
           p.p.x, p.p.y, p.p.z))
      {
        if(transformToGlobal)
          // now get from left cam coord sys to global coord sys
          p.p = transform(global_left_pose, p.p);
        VisionData::ColorRGB *c =
          (VisionData::ColorRGB*)Video::cvAccessImageData(imgSet.rectColorImg[LEFT], x, y);
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
        VisionData::ColorRGB *c =
          (VisionData::ColorRGB*)Video::cvAccessImageData(imgSet.rectColorImg[LEFT], x, y);
        p.c = *c;
        points.push_back(p);
      }
    }

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
  image.camPars.id = side;
  image.camPars.width = stereoCam->cam[side].width;
  image.camPars.height = stereoCam->cam[side].height;
  image.camPars.fx = stereoCam->cam[side].proj[0][0];
  image.camPars.fy = stereoCam->cam[side].proj[1][1];
  image.camPars.cx = stereoCam->cam[side].proj[0][2];
  image.camPars.cy = stereoCam->cam[side].proj[1][2];
  changeImageSize(image.camPars, stereoCam->inImgSize.width, stereoCam->inImgSize.height);
  // get from relative left pose to global left pose
  Pose3 global_pose;
  transform(stereoCam->pose, stereoCam->cam[side].pose, global_pose);
  image.camPars.pose = global_pose;
  image.camPars.time = getCASTTime();

  unlockComponent();
}

void StereoServer::getDisparityImage(int imgWidth, Video::Image& image)
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
  timespec start, stop;
  clock_gettime(CLOCK_REALTIME, &start);
  census->setImages(imgSet.rectGreyImg[LEFT], imgSet.rectGreyImg[RIGHT]);
  census->match();
  // in case we are interested how blazingly fast the matching is :)
  // census->printTiming();
  // census->printTiming();
  census->getDisparityMap(imgSet.disparityImg);
  clock_gettime(CLOCK_REALTIME, &stop);
  log("gpustereo for %d x %d: runtime / framerate: %lf s / %lf",
    imgSet.disparityImg->width, imgSet.disparityImg->height,
    timespec_diff(&stop, &start), 1./timespec_diff(&stop, &start));

  if(medianSize > 0)
  {
    IplImage *tmp = cvCloneImage(imgSet.disparityImg);
    cvSmooth(imgSet.disparityImg, tmp, CV_MEDIAN, medianSize);
    swap(imgSet.disparityImg, tmp);
    cvReleaseImage(&tmp);
  }
#else
  // use OpenCV stereo matching provided inside the stereo camera
  // TODO: timing seems to be wrong
  timespec start, stop;
  clock_gettime(CLOCK_REALTIME, &start);
  stereoCam->SetMatchingAlgoritm(StereoCamera::SEMI_GLOBAL_BLOCK_MATCH);
  //stereoCam->SetMatchingAlgoritm(StereoCamera::BLOCK_MATCH);
  stereoCam->CalculateDisparity(imgSet.rectGreyImg[LEFT], imgSet.rectGreyImg[RIGHT], imgSet.disparityImg);
  clock_gettime(CLOCK_REALTIME, &stop);
  log("OpenCV for %d x %d: runtime / framerate: %lf s / %lf",
    imgSet.disparityImg->width, imgSet.disparityImg->height,
    timespec_diff(&stop, &start), 1./timespec_diff(&stop, &start));
#endif

  // TODO: this does not work nicely with
  if(logImages)
  {
    for(int i = LEFT; i <= RIGHT; i++)
    {
      cvSaveImage(i == LEFT ? "stereoserver-orig-L.jpg" : "stereoserver-orig-R.jpg", imgSet.colorImg[i]);
      cvSaveImage(i == LEFT ? "stereoserver-rect-L.jpg" : "stereoserver-rect-R.jpg", imgSet.rectColorImg[i]);
    }
    cvSaveImage("stereoserver-disp.png", imgSet.disparityImg);
  }

  if(doDisplay)
  {
    cvShowImage("left", imgSet.rectColorImg[LEFT]);
    cvShowImage("right", imgSet.rectColorImg[RIGHT]);
    cvShowImage("disparity", imgSet.disparityImg);
    cvWaitKey(10);
  }
}

void StereoServer::receiveImages(const vector<Video::Image>& images)
{
  lockComponent();
  //stereoProcessing();
  unlockComponent();
}

void StereoServer::runComponent()
{
/*#ifdef HAVE_GPU_STEREO
  // allocate the actual stereo matcher with given max disparity
  census = new CensusGPU(maxDisp);
#endif

  // NOTE: stupid polling runloop is still necessary
  // push interface does not work for stereo server, probably some threading
  // issue with cuda or whatever
  vector<Video::Image> images;
  while(isRunning())
  {
    // HACK: we should actually sent the list of cam ids and not just assume
    // that the video server has precisely two cameras in the right order
    videoServer->getScaledImages(stereoSizes[0].width, stereoSizes[0].height, images);
    receiveImages(images);
    sleepComponent(100);
  }

#ifdef HAVE_GPU_STEREO
  delete census;
#endif*/
}

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

