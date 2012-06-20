/**
 * @file KinectPCServer.cpp
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the kinect sensor and the stereo setup.
 */

#include <cmath>
#include <cast/core/CASTUtils.hpp>
#include <opencv/highgui.h>
#include <VideoUtils.h>
#include "KinectStereoServer.h"


/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::KinectStereoServer();
  }
}

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;

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


KinectStereoServer::KinectStereoServer()
{
  framerateMillis = 0;
  #ifdef HAVE_GPU_STEREO
  census = 0;
  // normally it's a good idea to do median filering on the disparity image
  medianSize = 5;
#endif
  doDisplay = false;
  logImages = false;
}

KinectStereoServer::~KinectStereoServer()
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
  delete kinect;
}

/**
 * @brief Get video resolution for given camera.
 * @param camIdx which camera					/// TODO unused
 * @param size video resolution
 */
void KinectStereoServer::getResolution(int camIdx, CvSize &size)
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
bool KinectStereoServer::setResolution(int camIdx, CvSize &size)
{
  printf("KinectPCServer::setResolution: Warning: Not yet implemented! Defined by kinect camera calibration file!\n");
//   int i = camIdx;
//   int w = 0, h = 0;
//   bool gotRequestedSize = true;
// 
//   // first try the simple set property
//   cvSetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH, size.width);
//   cvSetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT, size.height);
// 
//   // check if that worked
//   w = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH);
//   h = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT);
//   // if not, use firewire specific set mode
//   if(w != size.width || h != size.height)
//   {
//     log("cvSetCaptureProperty(WIDTH/HEIGHT) didn't work, trying cvSetCaptureProperty(MODE)");
//     if(size.width == 320)
//       cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_320x240_YUV422);
//     else if(size.width == 640)
//       cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_640x480_YUV411);
//     else if(size.width == 800)
//       cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_800x600_MONO8);
//     else if(size.width == 1024)
//       cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_1024x768_RGB8);
//     else if(size.width == 1280)
//       cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_1280x960_RGB8);
//     else if(size.width == 1600)
//       cvSetCaptureProperty(captures[i], CV_CAP_PROP_MODE, DC1394_VIDEO_MODE_1600x1200_RGB8);
//     else
//       println("Warning: video resolution %d x %d not supported!\n",
//           size.width, size.height);
// 
//     // now check again
//     w = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_WIDTH);
//     h = (int)cvGetCaptureProperty(captures[i], CV_CAP_PROP_FRAME_HEIGHT);
// 
//     // if setting still did not work, use whatever the cameras are set to
//     if(w != size.width || h != size.height)
//     {
//       log("Warning: failed to set resolution to %d x %d, using %d x %d!\n",
//           size.width, size.height, w, h);
//       size.width = w;
//       size.height = h;
//       gotRequestedSize = false;
//     }
//   }
//   return gotRequestedSize;
  return false; /// TODO remove
}

/**
 * @param ids  list of camera IDs
 * @param dev_nums  list of device numbers (typically 0, 1) corresponding to camera IDs
 */
// void KinectServer::init(int dev_class, const vector<int> &dev_nums, const string &bayer) throw(runtime_error)
void KinectStereoServer::init() throw(runtime_error)
{
//   if(dev_nums.size() == 0)
//     throw runtime_error(exceptionMessage(__HERE__,
//           "must specify at least one camera"));
//   if(dev_nums.size() != camIds.size())
//     throw runtime_error(exceptionMessage(__HERE__,
//           "number of devices %d does not match number of camera IDs %d",
//           (int)dev_nums.size(), (int)camIds.size()));

//   grabTimes.resize(dev_nums.size());
//   retrievedImages.resize(dev_nums.size());
//   for(size_t i = 0; i < dev_nums.size(); i++)
//     retrievedImages[i] = 0;

//   captures.resize(dev_nums.size());
//   for(size_t i = 0; i < dev_nums.size(); i++)
//   {
//     captures[i] = cvCreateCameraCapture(dev_class + dev_nums[i]);
//     if(captures[i] == 0)
//       throw runtime_error(exceptionMessage(__HERE__,
//         "failed to create capture for video device %d", dev_nums[i]));
//     if(bayer.empty())
//     {
//       bayerCvt = CV_COLORCVT_MAX;
//     }
//     else
//     {
//       if(bayer == "BGGR")
//         bayerCvt = CV_BayerBG2RGB;
//       else if(bayer == "GBBR")
//         bayerCvt = CV_BayerGB2RGB;
//       else if(bayer == "RGGB")
//         bayerCvt = CV_BayerRG2RGB;
//       else if(bayer == "GRRB")
//         bayerCvt = CV_BayerGR2RGB;
//       else
//         throw runtime_error(exceptionMessage(__HERE__,
//             "invalid bayer order '%s', must be one of 'BGGR' 'GBBR' 'RGGB' 'GRRB'",
//             bayer.c_str()));
//       // the default in opencv is CV_CAP_PROP_CONVERT_RGB=1 which causes
//       // cameras with bayer encoding to be converted from mono to rgb
//       // without using the bayer functions. CV_CAP_PROP_CONVERT_RGB=0
//       // keeps the original format.
//       cvSetCaptureProperty(captures[i], CV_CAP_PROP_CONVERT_RGB, 0.0);
//     }
//   }

  // if capture size was not set by config, use what is currently set in the cameras
//   if(captureSize.width == 0 || captureSize.width == 0)
//   {
//     // get currently set resolution of first camera and set it to all other
//     // cameras so all cameras will have same resolution
//     getResolution(0, captureSize);
//     for(size_t i = 1; i < captures.size(); i++)
//       if(!setResolution(i, captureSize))
//         throw runtime_error(exceptionMessage(__HERE__,
//               "failed to set all cameras to identical resolutions"));
//     log("using currently set video resolution %d x %d\n",
//         captureSize.width, captureSize.height);
//   }
//   else
//   {
//     log("setting video resolution to %d x %d\n", captureSize.width, captureSize.height);
//     // set resolution for first camera, where we don't care whether our
//     // requested resolution was actually set
//     setResolution(0, captureSize);
//     // set for all other cameras to the same resolution (whatever it was)
//     // and insist they are the same
//     for(size_t i = 1; i < captures.size(); i++)
//       if(!setResolution(i, captureSize))
//         throw runtime_error(exceptionMessage(__HERE__,
//               "failed to set all cameras to identical resolutions"));
//     log("using available video resolution %d x %d\n", captureSize.width, captureSize.height);
//   }

  // frames per second
  double fps = 100000000.; //cvGetCaptureProperty(captures[0], CV_CAP_PROP_FPS);					/// TODO fps not correct

  if(fps > 0.)
    framerateMillis = (int)(1000./fps);  // milliseconds per frame
  else
    // just some huge value (better than 0. as that might result in divides by zero somewhere)
    framerateMillis = 1000000;

  // to make sure we have images in the capture's buffer
//   printf("KinectPCServer::init: end!\n");
}

void KinectStereoServer::configure(const map<string,string> & _config) throw(runtime_error)
{
  // configure the point cloud server
  PointCloudServer::configure(_config);
  string stereoCalibFile;
  map<string,string>::const_iterator it;
    
  if((it = _config.find("--kconfig")) != _config.end())
  {  
    istringstream str(it->second);
    str >> kinectConfig;
  }
  else throw runtime_error(exceptionMessage(__HERE__, "no kinect config file (kconfig) specified."));

  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }
  else throw runtime_error(exceptionMessage(__HERE__, "no video config file (videoname) specified."));

//   if((it = _config.find("--camids")) != _config.end()) /// shifted to point cloud server
//   {
//     istringstream str(it->second);
//     int id;
//     while(str >> id)
//       camIds.push_back(id);
//   } 
//   else throw runtime_error(exceptionMessage(__HERE__, "no camIDs specified."));

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

/// TODO old --stereoconfig things: delete it later
//   if((it = _config.find("--stereoconfig")) != _config.end())
//   {
//     stereoCalibFile = it->second;
//   }
//   else throw runtime_error(exceptionMessage(__HERE__, "no stereo config file specified"));
  
// if no image size was specified, use the original image size as given by the stereo config
//   if(stereoSizes.empty())
//   {
//     StereoCamera *sc = new StereoCamera();
//     sc->ReadSVSCalib(stereoCalibFile);
//     sc->SetupImageRectification();
//     stereoCams.push_back(sc);
//     stereoSizes.push_back(cvSize(sc->cam[0].width, sc->cam[0].height));
//   }
//   // else: we have a set of resolutions, create a stereo camera for each
//   else
//   {
//     for(size_t i = 0; i < stereoSizes.size(); i++)
//     {
//       StereoCamera *sc = new StereoCamera();
//       sc->ReadSVSCalib(stereoCalibFile);
//       // now set the input image size to be used by stereo matching
//       sc->SetInputImageSize(stereoSizes[i]);
//       sc->SetupImageRectification();
//       stereoCams.push_back(sc);
//     }
//   }

  if((it = _config.find("--imgsize")) != _config.end())							/// TODO nicht notwendige mit log oder debug Meldungen ausstatten
  {
    istringstream str(it->second);
    int w, h;
    while(str >> w >> h)
      stereoSizes.push_back(cvSize(w, h));
  }  

  if((it = _config.find("--maxdisp")) != _config.end())
  {
    istringstream str(it->second);
    int disp;
    while(str >> disp)
      maxDisps.push_back(disp);
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

  // sanity checks: Have all important things be configured? Is the configuration consistent?
  if(camIds.size() != 3)
    throw runtime_error(exceptionMessage(__HERE__, "need exactly 3 camera IDs"));

#ifdef HAVE_GPU_STEREO
  // allocate the actual stereo matcher with given max disparity
  census = new CensusGPU();
#endif
  
  // do some initialisation based on configured items
  init();

  // init kinect hardware driver
  CvSize size;
  const char* name = kinectConfig.c_str();
  kinect = new Kinect::Kinect(name);
  kinect->GetColorVideoSize(size);
  captureSize.width = size.width;
  captureSize.height = size.height;
  
  kinect->StartCapture(0); 	// start capturing
  log("Capturing started.");
}


void KinectStereoServer::start()
{
  PointCloudServer::start();
  
  // get connection to the video server
 videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

  if(doDisplay)
  {
    cvNamedWindow("left", 1);
    cvNamedWindow("right", 1);
    cvNamedWindow("disparity", 1);
  }
}

int KinectStereoServer::findClosestResolution(int imgWidth)
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



// ########################## Point Cloud Server Implementations ########################## //
void KinectStereoServer::getPoints(bool transformToGlobal, int imgWidth, vector<PointCloud::SurfacePoint> &points, bool complete)
{
  lockComponent();
  
  // ########################### stereo processing ########################### //
  int res = findClosestResolution(imgWidth);
  StereoCamera *stereoCam = stereoCams[res];
  ImageSet &imgSet = imgSets[res];

  vector<Video::Image> images;
 
  // get next kinect frame
  cv::Mat_<cv::Point3f> cloud;
  cv::Mat_<cv::Point3f> colCloud;
  kinect->NextFrame();
  kinect->Get3dWorldPointCloud(cloud, colCloud);
  videoServer->getScaledImages(stereoSizes[res].width, stereoSizes[res].height, images);


  points.resize(0);

  // ######################## kinect procesing ######################## //
  Pose3 global_kinect_pose, zeroPose;
  if(transformToGlobal)
    global_kinect_pose = camPars[2].pose;

  // copy clouds to points-vector (dense!)
  for(unsigned row=0; row< cloud.size().height; row++)    /// TODO SLOW!!!
  {
    for(unsigned col=0; col < cloud.size().width; col++)
    {
      PointCloud::SurfacePoint pt;
      pt.p.x = cloud.at<cv::Point3f>(row, col).x;
      pt.p.y = cloud.at<cv::Point3f>(row, col).y;
      pt.p.z = cloud.at<cv::Point3f>(row, col).z;
      pt.c.r = colCloud.at<cv::Point3f>(row, col).z;
      pt.c.g = colCloud.at<cv::Point3f>(row, col).y;
      pt.c.b = colCloud.at<cv::Point3f>(row, col).x;

      if(transformToGlobal)
        pt.p = transform(global_kinect_pose, pt.p);   // now get from kinect cam coord sys to global coord sys

      points.push_back(pt);
    }
  }
  
  //######################## stereo procesing ######################## //
  stereoProcessing(stereoCam, imgSet, images);
  Pose3 global_left_pose;
  if(transformToGlobal)
  {
    Pose3 ideal_pose, rel_pose;
    setIdentity(ideal_pose);
    // pose of ideal left camera w.r.t. to real left camera
    // the pose is a rotation given by the rectification matrix
    setRow33(ideal_pose.rot, (double*)stereoCam->cam[LEFT].rect);
    // NOTE: stereo camera rect matrix is the rotation matrix of real to ideal.
    // So to get from ideal to real, we have to invert.
    inverse(ideal_pose, ideal_pose);
    // get from ideal left pose to real left pose
    transform(stereoCam->cam[LEFT].pose, ideal_pose, rel_pose);
    // get from relative left pose to global left pose
    transform(stereoCam->pose, rel_pose, global_left_pose);
  }

  for(int y = 0; y < imgSet.disparityImg->height; y++)
    for(int x = 0; x < imgSet.disparityImg->width; x++)
    {
      float d = *((float*)Video::cvAccessImageData(imgSet.disparityImg, x, y));
      PointCloud::SurfacePoint p;
      if(stereoCam->ReconstructPoint((double)x, (double)y, (double)d,
           p.p.x, p.p.y, p.p.z))
      {
        if(transformToGlobal)
          // now get from left cam coord sys to global coord sys
          p.p = transform(global_left_pose, p.p);
        cogx::Math::ColorRGB *c =
          (cogx::Math::ColorRGB*)Video::cvAccessImageData(imgSet.rectColorImg[0], x, y);
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
        cogx::Math::ColorRGB *c = (cogx::Math::ColorRGB*) Video::cvAccessImageData(imgSet.rectColorImg[0], x, y);
        p.c = *c;
        points.push_back(p);
      }  
    }
  unlockComponent();
}

void KinectStereoServer::getRectImage(int side, int imgWidth, Video::Image& image)
{
  lockComponent();

  int res = findClosestResolution(imgWidth);
  StereoCamera *stereoCam = stereoCams[res];
  ImageSet &imgSet = imgSets[res];
  IplImage *kinectIpl;
  
  vector<Video::Image> images;
  
  if(side == LEFT || side == RIGHT) // get stereo images
  {
    // HACK: we should actually send the list of cam ids and not just assume
    // that the video server has precisely two cameras in the right order
    videoServer->getScaledImages(stereoSizes[res].width, stereoSizes[res].height, images);
    convertImageToIpl(images[side], &imgSet.colorImg[side]);
    stereoCam->RectifyImage(imgSet.colorImg[side], imgSet.rectColorImg[side], side);

    convertImageFromIpl(imgSet.rectColorImg[side], image);
  }
  else if(side == 2) // get kinect image
  {
    kinect->GetColorImage(&kinectIpl);
    convertImageFromIpl(kinectIpl, image);
  }
  
  if(side == LEFT || side ==RIGHT)
  {
    initCameraParameters(image.camPars);
    image.camPars.id = side;
    image.camPars.width = imgWidth;
    image.camPars.height = imgWidth*3/4;

    image.camPars.fx = stereoCam->cam[side].proj[0][0];
    image.camPars.fy = stereoCam->cam[side].proj[1][1];
    image.camPars.cx = stereoCam->cam[side].proj[0][2];
    image.camPars.cy = stereoCam->cam[side].proj[1][2];
//       changeImageSize(image.camPars, stereoCam->inImgSize.width, stereoCam->inImgSize.height);

    // set stereo camera pose to pose of left (=0) camera
    stereoCam->pose = camPars[LEFT].pose;
    Pose3 ideal_pose, rel_pose, global_pose;
    setIdentity(global_pose);
    // pose of ideal left/right camera w.r.t. to actual left/right camera
    // the pose is a rotation given by the rectification matrix
    setRow33(ideal_pose.rot, (double*)stereoCam->cam[side].rect);
    // NOTE: stereo camera rect matrix is the rotation matrix of real to ideal.
    // So to get from ideal to real, we have to invert.
    inverse(ideal_pose, ideal_pose);
    // get from ideal left/right pose to real left/right pose
    transform(stereoCam->cam[side].pose, ideal_pose, rel_pose);
    // get from relative left/right pose to global left/right pose
    transform(stereoCam->pose, rel_pose, global_pose);
    image.camPars.pose = global_pose;

    image.camPars.time = getCASTTime();
  }
  else if(side == 2)
  {
    initCameraParameters(image.camPars);
    image.camPars.id = side;
    image.camPars.width = imgWidth;
    image.camPars.height = imgWidth*3/4;
    image.camPars = camPars[2];

    Pose3 global_pose, zeroPose;
    setIdentity(zeroPose);
    transform(camPars[side].pose, zeroPose, global_pose);
    image.camPars.pose = global_pose;
    image.camPars.time = getCASTTime();
  }

  unlockComponent();
}

bool KinectStereoServer::getCameraParameters(Ice::Int side, Video::CameraParameters& _camPars)
{
  if (side != LEFT && side != RIGHT)
    return false;

  lockComponent(); // TODO: CASTComponent::Lock lock(this);

  initCameraParameters(_camPars);
  _camPars.id = camIds[side];
  _camPars.width  = camPars[side].width;
  _camPars.height = camPars[side].height;
  _camPars.fx = camPars[side].fx; // /scaleFactor;
  _camPars.fy = camPars[side].fy; // /scaleFactor;
  _camPars.cx = camPars[side].cx; // /scaleFactor;
  _camPars.cy = camPars[side].cy; // /scaleFactor;

  Pose3 global_pose, zeroPose;
  setIdentity(zeroPose);
  transform(camPars[side].pose, zeroPose, global_pose);
  _camPars.pose = global_pose;
  _camPars.time = getCASTTime();

  unlockComponent(); // TODO: remove

  return true;
}

/**
 * @brief Get the disparity image from the stereo calculation
 * @param imgWidth TODO not implemented!
 * @param image Disparity imgage to return
 */
void KinectStereoServer::getDisparityImage(int imgWidth, Video::Image& image)
{
	lockComponent();
  
  int res = findClosestResolution(imgWidth);
  StereoCamera *stereoCam = stereoCams[res];
  ImageSet &imgSet = imgSets[res];
  
	convertImageFromIpl(imgSet.disparityImg, image);
	unlockComponent();
}

void KinectStereoServer::stereoProcessing(StereoCamera *stereoCam, ImageSet &imgSet, const vector<Video::Image>& images)
{
  // assuming that the left camera pose is equal to stereo head pose,
  // save the head pose for this pair of images
  stereoCam->pose = images[LEFT].camPars.pose;

  double t1 = gethrtime_d();

  for(int i = LEFT; i <= RIGHT; i++)
  {
    convertImageToIpl(images[i], &imgSet.colorImg[i]);
    stereoCam->RectifyImage(imgSet.colorImg[i], imgSet.rectColorImg[i], i);
    cvCvtColor(imgSet.rectColorImg[i], imgSet.rectGreyImg[i], CV_RGB2GRAY);
  }
  cvSet(imgSet.disparityImg, cvScalar(0));

  double t2 = gethrtime_d();

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

  double t3 = gethrtime_d();

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

  double t4 = gethrtime_d();

  /*log("%s at %d x %d: copy/rectify/convert images: %lf, stereo: %lf, log/display images: %lf - total: %lf / frame rate: %lf",
#ifdef HAVE_GPU_STEREO
      "gpustereo",
#else
      "OpenCV stereo",
#endif
    imgSet.disparityImg->width, imgSet.disparityImg->height,
    t2 - t1, t3 - t2, t4 - t3, t4 - t1, 1./(t4 - t1));*/
}

void KinectStereoServer::receiveImages(const vector<Video::Image>& images)
{
//   lockComponent();
  printf("KinectStereoServer::receiveImages: Warning: Not yet implemented!\n");
  //stereoProcessing();
//   unlockComponent();
}

}

