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


void StereoServerI::getPoints(bool transformToGlobal, double resolution, VisionData::SurfacePointSeq& points, const Ice::Current&)
{
  stereoSrv->getPoints(transformToGlobal, resolution, points);
}

void StereoServerI::getPointsInSOI(bool transformToGlobal, const VisionData::SOIPtr &soi, double resolution,
    VisionData::SurfacePointSeq& points, const Ice::Current&)
{
  stereoSrv->getPointsInSOI(transformToGlobal, *soi, resolution, points);
}

void StereoServerI::getRectImage(Ice::Int side, double resolution, Video::Image& image, const Ice::Current&)
{
  stereoSrv->getRectImage(side, resolution, image);
}

void StereoServerI::getDisparityImage(double resolution, Video::Image& image, const Ice::Current&)
{
  stereoSrv->getDisparityImage(resolution, image);
}

StereoServer::StereoServer()
{
#ifdef HAVE_GPU_STEREO
  census = 0;
  // normally it's a good idea to do median filering on the disparity image
  medianSize = 5;
#endif
  iceStereoName = "";
  iceStereoPort = cdl::CPPSERVERPORT;
  doDisplay = false;
  logImages = false;
  maxDisp = 64;
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
  hStereoServer = new StereoServerI(this);
  registerIceServer<Stereo::StereoInterface, StereoServerI>(hStereoServer);
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
    int w, h;
    str >> w >> h;
    stereoSizes.push_back(cvSize(w, h));
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
  if(stereoSizes.empty())
  {
    stereoSizes.push_back(cvSize(stereoCam.cam[LEFT].width,
                                 stereoCam.cam[LEFT].height));
  }
  // now set the input image size to be used by stereo matching
  stereoCam.SetInputImageSize(stereoSizes[0]);
  stereoCam.SetupImageRectification();
  stereoCam.SetDisparityRange(0, maxDisp);

  // sanity checks: Have all important things be configured? Is the
  // configuration consistent?
  if(camIds.size() != 2)
    throw runtime_error(exceptionMessage(__HERE__, "need exactly 2 camera IDs"));
  //if(iceStereoName.empty())
  //  throw runtime_error(exceptionMessage(__HERE__, "no stereo server name given"));
  if(!haveStereoConfig)
    throw runtime_error(exceptionMessage(__HERE__, "no stereo config file given"));

  setupMyIceCommunication();
}

void StereoServer::start()
{
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

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
    colorImg[i] = cvCreateImage(stereoSizes[0], IPL_DEPTH_8U, 3);
    rectColorImg[i] = cvCreateImage(stereoSizes[0], IPL_DEPTH_8U, 3);
    rectGreyImg[i] = cvCreateImage(stereoSizes[0], IPL_DEPTH_8U, 1);
  }
  disparityImg = cvCreateImage(stereoSizes[0], IPL_DEPTH_32F, 1);
  cvSet(disparityImg, cvScalar(0));

  // NOTE: stupid polling runloop is still necessary
  // push interface does not work for stereo server, probably some threading
  // issue with cuda or whatever
  // start receiving images pushed by the video server
  //videoServer->startReceiveImages(getComponentID().c_str(), camIds, stereoSizes[0].width,
  //    stereoSizes[0].height);
}

void StereoServer::getPoints(bool transformToGlobal, double resolution, vector<VisionData::SurfacePoint> &points)
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
      float d = *((float*)Video::cvAccessImageData(disparityImg, x, y));
      VisionData::SurfacePoint p;
      if(stereoCam.ReconstructPoint((double)x, (double)y, (double)d,
           p.p.x, p.p.y, p.p.z))
      {
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

void StereoServer::getPointsInSOI(bool transformToGlobal, const VisionData::SOI &soi, double resolution,
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
      float d = *((float*)Video::cvAccessImageData(disparityImg, x, y));
      VisionData::SurfacePoint p;
      if(stereoCam.ReconstructPoint((double)x, (double)y, (double)d,
           p.p.x, p.p.y, p.p.z))
      {
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

void StereoServer::getRectImage(int side, double resolution, Video::Image& image)
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

void StereoServer::getDisparityImage(double resolution, Video::Image& image)
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

#ifdef HAVE_GPU_STEREO
  timespec start, stop;
  clock_gettime(CLOCK_REALTIME, &start);
  census->setImages(rectGreyImg[LEFT], rectGreyImg[RIGHT]);
  census->match();
  // in case we are interested how blazingly fast the matching is :)
  // census->printTiming();
  census->getDisparityMap(disparityImg);
  clock_gettime(CLOCK_REALTIME, &stop);
  log("gpustereo runtime / framerate: %lf s / %lf", timespec_diff(&stop, &start), 1./timespec_diff(&stop, &start));

  if(medianSize > 0)
  {
    IplImage *tmp = cvCloneImage(disparityImg);
    cvSmooth(disparityImg, tmp, CV_MEDIAN, medianSize);
    swap(disparityImg, tmp);
    cvReleaseImage(&tmp);
  }
#else
  // use OpenCV stereo matching provided inside the stereo camera
  // TODO: timing seems to be wrong
  timespec start, stop;
  clock_gettime(CLOCK_REALTIME, &start);
  stereoCam.CalculateDisparity(rectGreyImg[LEFT], rectGreyImg[RIGHT], disparityImg);
  clock_gettime(CLOCK_REALTIME, &stop);
  log("OpenCV runtime / framerate: %lf s / %lf", timespec_diff(&stop, &start), 1./timespec_diff(&stop, &start));

  //clock_gettime(CLOCK_REALTIME, &start);
  //usleep(1000);
  //clock_gettime(CLOCK_REALTIME, &stop);
  //log("usleep 1000: %lf s / %lf", timespec_diff(&stop, &start), 1./timespec_diff(&stop, &start));
#endif

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
#ifdef HAVE_GPU_STEREO
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
#endif
}

}

