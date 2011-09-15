/**
 * @file KinectStereoSeqServer.cpp
 * @author Richtsfeld Andreas
 * @date April 2011
 * @version 0.1
 * @brief Point cloud server for the kinect sensor and the stereo setup.
 * Reads sequences of kinect and stereo data from files.
 */


#include <cast/core/CASTUtils.hpp>
#include <cmath>
#include <opencv/highgui.h>
#include <VideoUtils.h>

#include "KinectStereoSeqServer.h"

#include <iostream>
#include <sstream>
#include <algorithm>
#include <iterator>

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::KinectStereoSeqServer();
  }
}

namespace cast
{
using namespace std;
using namespace cogx;
using namespace cogx::Math;



CSequenceInfo::CSequenceInfo()
{
  clear();
}

void CSequenceInfo::clear()
{
  fileTemplates = "";
  start = 0;
  end = 0;
  step = 1;
  repeatFrame = 1;
  downsampleFactor = 1;
  loop = true;
}

void CSequenceInfo::checkLimits()
{
  if (start < 0) start = 0;
  if (end < 0) end = 0;
  if (step <= 0) step = 1;
  if (repeatFrame <= 0) repeatFrame = 1;
  if(downsampleFactor <= 0) downsampleFactor = 1;
}

void CSequenceInfo::parseConfig(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  clear();

  if((it = _config.find("--files")) != _config.end())
  {
    fileTemplates = it->second;
  }
  if((it = _config.find("--start")) != _config.end())
  {
    istringstream str(it->second);
    str >> start;
  }
  if((it = _config.find("--end")) != _config.end())
  {
    istringstream str(it->second);
    str >> end;
  }
  if((it = _config.find("--step")) != _config.end())
  {
    istringstream str(it->second);
    str >> step;
  }
  if((it = _config.find("--repeatframe")) != _config.end())
  {
    istringstream str(it->second);
    str >> repeatFrame;
  }
  if((it = _config.find("--noloop")) != _config.end())
  {
    loop = false;
  }
  if((it = _config.find("--downsample")) != _config.end())
  {
    istringstream str(it->second);
    str >> downsampleFactor;
  }
  checkLimits();
}

void CSequenceInfo::parseConfig(const std::string& _configStr)
{
  map<string,string> params;
  // TODO: split string, join quoted strings, add to map
  vector<string> tokens, parts;
  vector<string>::iterator it, itnx;
  istringstream iss(_configStr);
  copy(istream_iterator<string>(iss),
       istream_iterator<string>(),
       back_inserter<vector<string> >(tokens));

  // Fix quoted parameters (escape sequences are not handled)
  for(it = tokens.begin(); it != tokens.end(); it++) {
    if (it->substr(0, 1) == "\"") {
      string s = *it;
      while (s.substr(s.length()-1, 1) != "\"") {
        it++;
        if (it == tokens.end()) break;
        s = s + " " + *it;
      }
      parts.push_back(s.substr(1, s.length()-2));
    }
    else if (it->substr(0, 1) == "\'") {
      string s = *it;
      while (s.substr(s.length()-1, 1) != "\'") {
        it++;
        if (it == tokens.end()) break;
        s = s + " " + *it;
      }
      parts.push_back(s.substr(1, s.length()-2));
    }
    else parts.push_back(*it);
  }  

  // create a map from the list
  for(it = parts.begin(); it != parts.end(); it++) {
    string& s = *it;
    if (s.substr(0, 2) != "--") continue; // TODO: error - expecting parameter

    string ns;
    itnx = it+1;
    if (itnx == tokens.end()) ns = "";
    else {
      ns = *itnx;
      if (ns.substr(0, 2) == "--") ns = "";
    }
    params[s] = ns;
    if (itnx->substr(0, 2) != "--") it++;
  }

  parseConfig(params);
}

void CSequenceInfo::setInfo(Video::VideoSequenceInfo& info)
{
  clear();
  fileTemplates = info.fileTemplates;
  start = info.start;
  end = info.end;
  step = info.step;
  repeatFrame = info.repeatFrame;
  loop = info.loop;
  downsampleFactor = 1;
  checkLimits();
}


// ####################### KINECT STEREO SEQUENCE SERVER ####################### //
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


KinectStereoSeqServer::KinectStereoSeqServer()
{
  haveImages = false;
  framerateMillis = 1000; // default framerate = 1s
  width = 0;
  height = 0;
  
  #ifdef HAVE_GPU_STEREO
  census = 0;
  // normally it's a good idea to do median filering on the disparity image
  medianSize = 5;
  #endif

  doDisplay = false;
  logImages = false;
  
  scaledStereoImages[0] = 0;
  scaledStereoImages[1] = 0;
  rectifiedStereoImages[0] = 0;;
  rectifiedStereoImages[1] = 0;;
  rectifiedGrayStereoImages[0] = 0;;
  rectifiedGrayStereoImages[1] = 0;;
  disparityImg = 0;
}

KinectStereoSeqServer::~KinectStereoSeqServer()
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

/**
 * @brief Construct filenames from file templates.
 * Each camera has a file template, e.g. img_left_%03d.jpg img_right_%03d.jpg
 * @param fileTemplates
 * @param first First id
 * @param last Last id
 * @param inc Increment value
 */
void KinectStereoSeqServer::constructFilenames(const vector<string> &fileTemplates, int first, int last, int inc)
{
  char filename[1024];
  // normal order: first < last, positive increment
  // (note first = last is a special case: just one image)
  if(first <= last)
  {
    // if no increment given, assume default
    if(inc == 0)
      inc = 1;
    // just in case we were given a stupid increment
    else if(inc < 0)
      inc = -inc;
    for(int i = first; i <= last; i += inc)
      for(size_t c = 0; c < fileTemplates.size(); c++)
      {
        snprintf(filename, 1024, fileTemplates[c].c_str(), i);
        filenames.push_back(filename);
      }
  }
  // reverse order (to run a movie backwards): first > last, negative increment
  else
  {
    if(inc == 0)
      inc = -1;
    else if(inc > 0)
      inc = -inc;
    for(int i = first; i >= last; i += inc)
      for(size_t c = 0; c < fileTemplates.size(); c++)
      {
        snprintf(filename, 1024, fileTemplates[c].c_str(), i);
        filenames.push_back(filename);
      }
  }
}


/**
 * @brief Read the first image of the sequence and store its size. This assumes of
 * course that all images in the sequence have the same size.
 * Furthermore this requires the file list to be initialised already
 */
void KinectStereoSeqServer::obtainImageSize() throw(runtime_error)
{
  if(width == 0)  // if width and height are not set yet
  {
    IplImage *img = 0;

    if(filenames.size() == 0)
      throw runtime_error(exceptionMessage(__HERE__, "video not initialised"));
    // load the first image and have a look at its size
    // note: we are not simply using GrabFrames() here, as that would increment
    // the counter, set times etc.
    img = cvLoadImage(filenames[0].c_str(), CV_LOAD_IMAGE_COLOR);
    if(img == 0)
      throw runtime_error(exceptionMessage(__HERE__, "failed to load image '%s'", filenames[0].c_str()));
    width = img->width;
    height = img->height;
    cvReleaseImage(&img);
  }
  else
    throw runtime_error(exceptionMessage(__HERE__, "obtainImageSize() must only be called once"));
}

/**
 * @brief Get video resolution for given camera.
 * @param camIdx which camera					/// TODO unused
 * @param size video resolution
 */
void KinectStereoSeqServer::getResolution(int camIdx, CvSize &size)
{
  size = cvSize(width, height);   // image size of file sequence
}


/**
 * @brief Set resolution for a given camera
 * @param camIdx which camera
 * @param size requested video resolution, on exit contains the actually
 *        set resolution, which might differ dependig on the cameras capabilities
 * @return true if the requested resolution could be set, false if another
 *        reslution was chosen
 */
bool KinectStereoSeqServer::setResolution(int camIdx, CvSize &size)
{
  printf("KinectStereoSeqServer::setResolution: Warning: Not yet implemented.\n");
  return false;
}


/**
 * @brief Configure
 */
void KinectStereoSeqServer::configure(const map<string,string> & _config) throw(runtime_error)
{
  // configure the point cloud server
  PointCloudServer::configure(_config);
  
  map<string,string>::const_iterator it;
  
  // define number of cams and filenames // TODO we know that we have 3 cams and 4 file-templates
  numCams = 3;
  numFiles = numCams + 1;
  
  CSequenceInfo sequenceInfo;
  string firstSequence = "";
  sequenceInfo.parseConfig(_config);
  if (sequenceInfo.fileTemplates.length() > 0) {
    firstSequence = "[--files]";
    if (sequenceInfo.loop) sequenceMap[firstSequence] = sequenceInfo;
    else leadInMap[firstSequence] = sequenceInfo;
  }
  installSequence(firstSequence);

  if((it = _config.find("--framerate_ms")) != _config.end())
  {
    istringstream str(it->second);
    str >> framerateMillis;
    if(framerateMillis <= 0)
      framerateMillis = 1000;
  } else log("configure: warning: framerate_ms set to 1000ms = 1s.\n");

  if((it = _config.find("--imgsize")) != _config.end())
  {
    istringstream str(it->second);
    int w, h;
    while(str >> w >> h)
    {
      stereoSizes.push_back(cvSize(w, h));
    }
  } else log("configure: warning: No image sizes specified.\n");

  if((it = _config.find("--maxdisp")) != _config.end())
  {
    istringstream str(it->second);
    int disp;
    while(str >> disp)
      maxDisps.push_back(disp);
  } else log("configure: warning: No image sizes specified.\n");
  
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
    log("do display.");
  }

  if((it = _config.find("--noCont")) != _config.end())
  {
    noContinousGrabing = true;
    log("no continous grabbing of frames active.");
  }
  
  if((it = _config.find("--logimages")) != _config.end())
  {
    logImages = true;
    log("log images.");
  }
  
//   string stereoCalibFile;
//   if((it = _config.find("--stereoconfig")) != _config.end())
//   {
//     stereoCalibFile = it->second;
//   }
//   else throw runtime_error(exceptionMessage(__HERE__, "no stereo config file specified"));
  
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
  }

  if(!maxDisps.empty())
  {
    if(maxDisps.size() != stereoCams.size())
      throw runtime_error(exceptionMessage(__HERE__, "need max disparity for each stereo resolution"));
    for(size_t i = 0; i < stereoCams.size(); i++)
      stereoCams[i]->SetDisparityRange(0, maxDisps[i]);
  }

  if(camIds.size() != 3)
    throw runtime_error(exceptionMessage(__HERE__, "need exactly 3 camera IDs"));

#ifdef HAVE_GPU_STEREO
  // allocate the actual stereo matcher with given max disparity
  census = new CensusGPU();
#endif
}

void KinectStereoSeqServer::start()
{
  grabbedImages.resize(getNumCams());
  frameCnt = 0;
  
  if(doDisplay)
  {
    cvNamedWindow("left", 1);
    cvNamedWindow("right", 1);
    cvNamedWindow("disparity", 1);
  }
  
  PointCloudServer::start();
}

/**
 * @brief Start grabing images, when component starts!
 */
void KinectStereoSeqServer::runComponent()
{
  while(isRunning() && !noContinousGrabing)
  {
    grabFramesInternal();
    sleepComponent(framerateMillis);
  }
}

/**
 * @brief Find the next best possible resolution for the stereo processing.
 * @param findClosestResolution Image width
 */
int KinectStereoSeqServer::findClosestResolution(int imgWidth)
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

/**
 * @brief Grab frames (internal method)
 */
void KinectStereoSeqServer::grabFramesInternal()
{
  lockComponent();
  std::string ptCloudFileName;
  
  if((filenames.size() == 0) || (frameCnt >= numFrames() && !loopSequence))
    tryNextSequence();

  // tryNextSequence can cause a deadlock if it's afer this lock
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_sequenceMonitor);

  if(filenames.size() == 0)
    throw runtime_error(exceptionMessage(__HERE__, "video not initialised"));

  if(frameCnt < numFrames() || loopSequence)
  {
    int fn = frameCnt % numFrames();    // number of current frame, note that we loop
    for(size_t i = 0; i < grabbedImages.size(); i++)
    {
      ptCloudFileName = filenames[fn*getNumFiles() + grabbedImages.size()]; // we know that the last filename is for the point cloud
      cvReleaseImage(&grabbedImages[i]);
      grabbedImages[i] = cvLoadImage(filenames[fn*getNumFiles() + i].c_str(), CV_LOAD_IMAGE_COLOR);
      if(grabbedImages[i] == 0)
        throw runtime_error(exceptionMessage(__HERE__, "failed to load image '%s'", filenames[fn*getNumCams() + i].c_str()));
      if(grabbedImages[i]->width != width || grabbedImages[i]->height != height)
        throw runtime_error(exceptionMessage(__HERE__, "size of loaded image '%s': %dx%d does not match video size %dx%d",
              filenames[fn*getNumCams() + i].c_str(),
              grabbedImages[i]->width, grabbedImages[i]->height,
              width, height));
    }
  }
  else
  {
    for(size_t i = 0; i < grabbedImages.size(); i++) // return empty images
    {
      cvReleaseImage(&grabbedImages[i]);
      grabbedImages[i] = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
      cvSet(grabbedImages[i], cvScalar(0));
    }
  }
  
  cdl::CASTTime time = getCASTTime();
  for(size_t i = 0; i < grabTimes.size(); i++)
    grabTimes[i] = time;

  if (frameRepeatPos > frameRepeatCnt) frameRepeatPos = frameRepeatCnt;
  frameRepeatPos--;
  if (frameRepeatPos <= 0)
  {
    frameRepeatPos = frameRepeatCnt;
    frameCnt++;
  }
  
  // read kinect point cloud from file
  AR::readFromFile(ptCloudFileName.c_str(), cloud, colCloud);
  haveImages = true;

  unlockComponent();
}


// ######################### FILE SEQUENCE PROCESSING ######################### //
void KinectStereoSeqServer::initFileTemplates(const vector<string> &fileTemplates, int first, int last, int inc) throw(runtime_error)
{
  if(fileTemplates.size() == 0) throw runtime_error(exceptionMessage(__HERE__, "no image lists given"));
  if(fileTemplates.size() != getNumFiles())
    throw runtime_error(exceptionMessage(__HERE__, "number of file templates %d does not match number of camera IDs %d",
          (int)fileTemplates.size(), getNumCams()));
  grabTimes.resize(getNumFiles());
  grabbedImages.resize(getNumFiles());
  for(size_t i = 0; i < grabbedImages.size(); i++)
    grabbedImages[i] = 0;
  constructFilenames(fileTemplates, first, last, inc);
  obtainImageSize();
}

void KinectStereoSeqServer::switchSequence(CSequenceInfo& seq)
{
  IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_sequenceMonitor);
  seq.checkLimits();

  // XXX: filenames are not cleared by constructFilenames().
  // XXX: width & height are reset so that obtainImageSize() will work.
  // TODO: frame size should remain unchanged when switching sequences (use downsampleFactor?)
  filenames.clear();
  width = 0;
  height = 0;

  vector<string> fileTemplates;
  {
    istringstream str(seq.fileTemplates);
    string file;
    while(str >> file)
      fileTemplates.push_back(file);
  }
  // do some initialisation based on configured items
  initFileTemplates(fileTemplates, seq.start, seq.end, seq.step);

  frameRepeatCnt = seq.repeatFrame;
  if(frameRepeatCnt <= 0) frameRepeatCnt = 1;
  loopSequence = seq.loop;

  // Restart the sequence
  frameRepeatPos = frameRepeatCnt;
  frameCnt = 0;
}

void KinectStereoSeqServer::installSequence(const std::string& name)
{
  std::map<std::string,CSequenceInfo>::iterator it;
  if (m_currentSequenceName != "") {
    it = leadOutMap.find(m_currentSequenceName);
    if (it != leadOutMap.end()) {
      m_sequenceStage = stLeadOut;
      switchSequence(leadOutMap[m_currentSequenceName]);
      m_nextSequenceName = name;
      return;
    }
  }
  m_nextSequenceName = name;
  tryNextSequence();
}

void KinectStereoSeqServer::tryNextSequence()
{
  std::map<std::string,CSequenceInfo>::iterator it;

  if (m_nextSequenceName == "") {
    if (m_sequenceStage == stLeadIn) {
      it = sequenceMap.find(m_currentSequenceName);
      if (it != sequenceMap.end()) {
        switchSequence(sequenceMap[m_currentSequenceName]);
        return;
      }
    }
    return;
  }

  it = leadInMap.find(m_nextSequenceName);
  if (it != leadInMap.end())
  {
    switchSequence(leadInMap[m_nextSequenceName]);
    m_currentSequenceName = m_nextSequenceName;
    m_nextSequenceName = "";
    m_sequenceStage = stLeadIn;
    return;
  }

  it = sequenceMap.find(m_nextSequenceName);
  if (it != sequenceMap.end())
  {
    switchSequence(sequenceMap[m_nextSequenceName]);
    m_currentSequenceName = m_nextSequenceName;
    m_nextSequenceName = "";
    m_sequenceStage = stLoop;
    return;
  }

  m_nextSequenceName = "";
}


// ########################## Point Cloud Server Implementations ########################## //
/**
 * @brief Get the point clouds from stereo and kinect.
 * @param transformToGlobal Transform to global coordinates.
 * @param imageWidth The point cloud and image width
 * @param points Point cloud to return
 * @param complete TODO always complete!
 */
void KinectStereoSeqServer::getPoints(bool transformToGlobal, int imgWidth, vector<PointCloud::SurfacePoint> &points, bool complete)
{ 
  if(imgWidth >= 640)
    log("getPoints: Warning: Point clouds >= 640x480 may exceed memory limit of ice streams."); 
  
  if(noContinousGrabing) grabFramesInternal();

  while(!haveImages)
  {
    sleepComponent(100);
    log("getPoints: Warning: wait for images.");
  }
  lockComponent();
  
  int res = findClosestResolution(imgWidth);
  StereoCamera *stereoCam = stereoCams[res];

  // set stereo camera pose to pose of left (=0) camera
  stereoCam->pose = camPars[LEFT].pose;

  cvReleaseImage(&scaledStereoImages[0]);
  cvReleaseImage(&scaledStereoImages[1]);
  cvReleaseImage(&rectifiedStereoImages[0]);
  cvReleaseImage(&rectifiedStereoImages[1]);
  cvReleaseImage(&rectifiedGrayStereoImages[0]);
  cvReleaseImage(&rectifiedGrayStereoImages[1]);
  cvReleaseImage(&disparityImg);
  
  scaledStereoImages[0] = cvCreateImage(cvSize(stereoCam->inImgSize.width, stereoCam->inImgSize.height), IPL_DEPTH_8U, 3);
  scaledStereoImages[1] = cvCreateImage(cvSize(stereoCam->inImgSize.width, stereoCam->inImgSize.height), IPL_DEPTH_8U, 3);
  rectifiedStereoImages[0] = cvCreateImage(cvSize(stereoCam->inImgSize.width, stereoCam->inImgSize.height), IPL_DEPTH_8U, 3);
  rectifiedStereoImages[1] = cvCreateImage(cvSize(stereoCam->inImgSize.width, stereoCam->inImgSize.height), IPL_DEPTH_8U, 3);
  rectifiedGrayStereoImages[0] = cvCreateImage(cvSize(stereoCam->inImgSize.width, stereoCam->inImgSize.height), IPL_DEPTH_8U, 1);
  rectifiedGrayStereoImages[1] = cvCreateImage(cvSize(stereoCam->inImgSize.width, stereoCam->inImgSize.height), IPL_DEPTH_8U, 1);
  disparityImg = cvCreateImage(cvSize(stereoCam->inImgSize.width, stereoCam->inImgSize.height), IPL_DEPTH_32F, 1);

  cvResize(grabbedImages[0], scaledStereoImages[0]);
  cvResize(grabbedImages[1], scaledStereoImages[1]);

  for(int i = LEFT; i <= RIGHT; i++)
  {
    stereoCam->RectifyImage(scaledStereoImages[i], rectifiedStereoImages[i], i);
    cvCvtColor(rectifiedStereoImages[i], rectifiedGrayStereoImages[i], CV_RGB2GRAY);
  }

  cvSet(disparityImg, cvScalar(0));

  // calculate disparity image
#ifdef HAVE_GPU_STEREO
  census->setOptions(0,               // min disp
                     maxDisps[res]);  // max disp
  census->setImages(rectifiedGrayStereoImages[LEFT], rectifiedGrayStereoImages[RIGHT]);
  census->match();
  // in case we are interested how blazingly fast the matching is :)
  // census->printTiming();
  // census->printTiming();
  census->getDisparityMap(disparityImg);
  if(medianSize > 0)
  {
    IplImage *tmp = cvCloneImage(disparityImg);
    cvSmooth(disparityImg, tmp, CV_MEDIAN, medianSize);
    swap(disparityImg, tmp);
    cvReleaseImage(&tmp);
  }
#else
  // use OpenCV stereo matching provided inside the stereo camera
  stereoCam->SetMatchingAlgoritm(StereoCamera::SEMI_GLOBAL_BLOCK_MATCH);
  //stereoCam->SetMatchingAlgoritm(StereoCamera::BLOCK_MATCH);
  stereoCam->CalculateDisparity(rectifiedGrayStereoImages[LEFT], rectifiedGrayStereoImages[RIGHT], disparityImg);
#endif


  points.resize(0);
  
  // ######################## kinect procesing ######################## //
  Pose3 global_kinect_pose;
  if(transformToGlobal)
    global_kinect_pose = camPars[2].pose;

  int scale = (int) cloud.size().width / imgWidth;        // scale down to imageWidth (in full steps)
  for(unsigned row=0; row< cloud.size().height; row+=scale)                /// TODO SLOW!!!
  {
    for(unsigned col=0; col < cloud.size().width; col+=scale)
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
  
  // ######################## stereo procesing ######################## //
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

  // get stereo point cloud with the color from the left RECTIFIED image!!!       /// TODO We need the rectified image
  for(int y = 0; y < disparityImg->height; y++)
    for(int x = 0; x < disparityImg->width; x++)
    {
      float d = *((float*)Video::cvAccessImageData(disparityImg, x, y));
      PointCloud::SurfacePoint p;
      if(stereoCam->ReconstructPoint((double)x, (double)y, (double)d, p.p.x, p.p.y, p.p.z))
      {
        if(transformToGlobal)
          // now get from left cam coord sys to global coord sys
          p.p = transform(global_left_pose, p.p);
      
        cogx::Math::ColorRGB *c = (cogx::Math::ColorRGB*)Video::cvAccessImageData(rectifiedStereoImages[0], x, y);
        p.c = *c;
        points.push_back(p);
      }
      else if(complete)
      {
        p.p = vector3(0., 0., 0.);
        cogx::Math::ColorRGB *c = (cogx::Math::ColorRGB*) Video::cvAccessImageData(rectifiedStereoImages[0], x, y);
        p.c = *c;
        points.push_back(p);
      }  
    }
    
  if(logImages)
  {
    for(int i = LEFT; i <= RIGHT; i++)
    {
      cvSaveImage(i == LEFT ? "KinectStereoSeqServer-orig-L.jpg" : "KinectStereoSeqServer-orig-R.jpg", grabbedImages[i]);
      cvSaveImage(i == LEFT ? "KinectStereoSeqServer-rect-L.jpg" : "KinectStereoSeqServer-rect-R.jpg", rectifiedStereoImages[i]);
    }
    cvSaveImage("KinectStereoSeqServer-disp.png", disparityImg);
  }

  if(doDisplay)
  {
    cvShowImage("left", rectifiedStereoImages[LEFT]);
    cvShowImage("right", rectifiedStereoImages[RIGHT]);
    cvShowImage("disparity", disparityImg);
    cvWaitKey(10);
  }
  
  unlockComponent();
}

/**
 * @brief Get the rectified and resized images.
 * @param side Which side: 0 = Left stereo / 1 = right stereo / 2 = kinect
 * @param imageWidth The image width
 * @param image Image to return
 */
void KinectStereoSeqServer::getRectImage(int side, int imgWidth, Video::Image& image)
{
  while(!haveImages)
  {
    sleepComponent(100);
    log("getRectImage: Warning: wait for images.\n");
  }
  
  assert(side == LEFT || side == RIGHT || side == 2);
  lockComponent();

  int res = findClosestResolution(imgWidth);
  StereoCamera *stereoCam = stereoCams[res];
  
  if(grabbedImages[side]->width != imgWidth) //scale image
  {
    cvReleaseImage(&scaledStereoImages[side]);
    cvReleaseImage(&rectifiedStereoImages[side]);
    scaledStereoImages[side] = cvCreateImage(cvSize(imgWidth, imgWidth*3/4), IPL_DEPTH_8U, 3);
    rectifiedStereoImages[side] = cvCreateImage(cvSize(imgWidth, imgWidth*3/4), IPL_DEPTH_8U, 3);
    cvResize(grabbedImages[side], scaledStereoImages[side]);
    if(side == LEFT || side == RIGHT) // if stereo image => rectify
    {
      stereoCam->RectifyImage(scaledStereoImages[side], rectifiedStereoImages[side], side);     
      convertImageFromIpl(rectifiedStereoImages[side], image);
    }
    else convertImageFromIpl(scaledStereoImages[side], image);
  }
  else
  {
    if(side == LEFT || side == RIGHT) // if stereo image => rectify
    {      
      cvReleaseImage(&rectifiedStereoImages[side]);
      rectifiedStereoImages[side] = cvCreateImage(cvSize(imgWidth, imgWidth*3/4), IPL_DEPTH_8U, 3);
      stereoCam->RectifyImage(grabbedImages[side], rectifiedStereoImages[side], side);
      convertImageFromIpl(rectifiedStereoImages[side], image);
    }
    else convertImageFromIpl(grabbedImages[side], image);
  }
    
  if(side == LEFT || side ==RIGHT)
  {
    initCameraParameters(image.camPars);
    image.camPars.id = side;
    image.camPars.width = stereoCam->cam[side].width*stereoCam->sx;
    image.camPars.height = stereoCam->cam[side].height*stereoCam->sy;
    image.camPars.fx = stereoCam->cam[side].proj[0][0]*stereoCam->sx;
    image.camPars.fy = stereoCam->cam[side].proj[1][1]*stereoCam->sy;
    image.camPars.cx = stereoCam->cam[side].proj[0][2]*stereoCam->sx;
    image.camPars.cy = stereoCam->cam[side].proj[1][2]*stereoCam->sy;

    // set stereo camera pose to pose of left (=0) camera
    stereoCam->pose = camPars[LEFT].pose;
    Pose3 ideal_pose, rel_pose, global_pose;
    setIdentity(ideal_pose);
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
    image.camPars = camPars[side];
    image.camPars.id = side;
    changeImageSize(image.camPars, imgWidth, imgWidth*3/4);

    image.camPars.pose = camPars[2].pose;

    image.camPars.time = getCASTTime();
  }
  unlockComponent();
}

/**
 * @brief Get the disparity image.
 * @param imageWidth The image width    /// TODO not considered!
 * @param image Disparity image to return
 */
void KinectStereoSeqServer::getDisparityImage(int imgWidth, Video::Image& image)
{
  lockComponent();
  while(!haveImages)
  {
    sleepComponent(100);
    log("getDisparityImage: Warning: wait for images.\n");
  }
  convertImageFromIpl(disparityImg, image);
  unlockComponent();
}

void KinectStereoSeqServer::receiveImages(const vector<Video::Image>& images)
{
//   lockComponent();
  printf("KinectStereoSeqServer::receiveImages: Warning: Not yet implemented!\n");
  //stereoProcessing();
//   unlockComponent();
}

}

