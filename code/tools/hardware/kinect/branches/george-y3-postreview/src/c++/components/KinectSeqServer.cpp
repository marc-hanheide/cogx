/**
 * @file KinectSeqServer.cpp
 * @author Richtsfeld Andreas
 * @date July 2011
 * @version 0.1
 * @brief Point cloud server for the kinect sensor, playing pcl-recorded streams.
 */

#include <cmath>
#include <cast/core/CASTUtils.hpp>
#include "KinectSeqServer.h"
#include <highgui.h>

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
cast::CASTComponentPtr newComponent() {
  return new cast::KinectSeqServer();
}
}

namespace cast {

using namespace std;
using namespace cogx;
using namespace cogx::Math;
using namespace cast::cdl;



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


// ####################### KINECT SEQUENCE SERVER ####################### //
/**
 * @brief Constructor of class KinectSeqServer
 */
KinectSeqServer::KinectSeqServer() {
  width = 0;
  height = 0;
}

/**
 * @brief Destructor of class KinectSeqServer
 */
KinectSeqServer::~KinectSeqServer() {
}


/**
 * @brief Construct filenames from file templates.
 * Each camera has a file template, e.g. img_left_%03d.jpg img_right_%03d.jpg
 * @param fileTemplates
 * @param first First id
 * @param last Last id
 * @param inc Increment value
 */
void KinectSeqServer::constructFilenames(const vector<string> &fileTemplates, int first, int last, int inc)
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
void KinectSeqServer::obtainImageSize() throw(runtime_error)
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
 * @brief Configure the component
 */
void KinectSeqServer::configure(const map<string, string> & _config) throw (runtime_error) 
{
  // configure the point cloud server
  PointCloudServer::configure(_config);

  numFiles = 2;   // 2 files: for the kinect image and for the pcd-file

  map<string, string>::const_iterator it;
  
  CSequenceInfo sequenceInfo;
  string firstSequence = "";
  sequenceInfo.parseConfig(_config);
  if (sequenceInfo.fileTemplates.length() > 0) {
    firstSequence = "[--files]";
    if (sequenceInfo.loop) sequenceMap[firstSequence] = sequenceInfo;
    else leadInMap[firstSequence] = sequenceInfo;
  }
  installSequence(firstSequence);
  
  m_displayImage = false;
  if ((it = _config.find("--display")) != _config.end()) {
    m_displayImage= true;
  }
  
  if((it = _config.find("--noCont")) != _config.end())
  {
    noContinousGrabing = true;
    log("no continous grabbing of frames active.");
  }
  
  if((it = _config.find("--framerate_ms")) != _config.end())
  {
    istringstream str(it->second);
    str >> framerateMillis;
    if(framerateMillis <= 0)
      framerateMillis = 1000;
  } else log("configure: warning: framerate_ms set to 1000ms = 1s.\n");


  if(camIds.size() != 1)
    throw runtime_error(exceptionMessage(__HERE__, "need a camera ID"));

  log("grabbing frames from pcl-point cloud sequences started.");
}

/**
* @brief Start the component
*/
void KinectSeqServer::start()
{
  PointCloudServer::start();
  if(m_displayImage){
    cvNamedWindow(getComponentID().c_str(),1);
  }
}

void KinectSeqServer::runComponent() {
  log("I am running");
  while(isRunning() && !noContinousGrabing) {
//       if (m_saveToFile) {
//         saveNextFrameToFile();
//         usleep(50000);
//       }
    grabFramesInternal();
    sleepComponent(framerateMillis);
  }
}
  
  
/**
* @brief Start the component
*/
void KinectSeqServer::grabFramesInternal()
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
    ptCloudFileName = filenames[fn*getNumFiles()+1]; // we know that the next filename is for the point cloud
    cvReleaseImage(&grabbedImage);
    grabbedImage = cvLoadImage(filenames[fn*getNumFiles()].c_str(), CV_LOAD_IMAGE_COLOR);
    if(grabbedImage == 0)
      throw runtime_error(exceptionMessage(__HERE__, "failed to load image '%s'", filenames[fn].c_str()));
    if(grabbedImage->width != width || grabbedImage->height != height)
      throw runtime_error(exceptionMessage(__HERE__, "size of loaded image '%s': %dx%d does not match video size %dx%d",
            filenames[fn].c_str(),
            grabbedImage->width, grabbedImage->height,
            width, height));
  }
  else
  {
    cvReleaseImage(&grabbedImage);
    grabbedImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    cvSet(grabbedImage, cvScalar(0));
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
  
//   std::cout << "File to read: " << ptCloudFileName.c_str() << endl;
  // READ kinect point cloud from pcd file!!!
  pcl::io::loadPCDFile (ptCloudFileName.c_str(), pcl_cloud);
  pcl_cloud.is_dense = true;
  
  haveImages = true;
  unlockComponent();
}


void KinectSeqServer::saveNextFrameToFile() {
//   kinect->NextFrame();
//   IplImage* rgb_data = new IplImage(kinect->rgbImage);
//   // Doing new IplImage(kinect->depImage); actually causes the depth map stored as a binary image for some reason
//   if(m_displayImage){
//     cvShowImage(getComponentID().c_str(),rgb_data);
//   }
//   IplImage* depth_data = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
//   char buf[256];
//   CASTTime timeNow = getCASTTime();
//   sprintf(buf, "%s/frame_%d_rgb_%ld_%ld.bmp", m_saveDirectory.c_str(), kinect->frameNumber,
//       (long int)timeNow.s, (long int)timeNow.us);
// 
//   cvSaveImage(buf, rgb_data);
// 
//   short*d = kinect->depImage.ptr<short>(0); 
//   for(int i = 0; i < kinect->depImage.rows*kinect->depImage.cols; i++)
//   {
//     short value = d[i]/16;
//     char value_pt1 = d[i]>>8;
//     char value_pt2 = d[i]&0xFF;
//     depth_data->imageData[3*i+0]=(char)value_pt1;
//     depth_data->imageData[3*i+1]=(char)value_pt2;
//     depth_data->imageData[3*i+2]=(char)value;
//   }
//   char buf2[256];
//   sprintf(buf2,"%s/frame_%d_depth_%ld_%ld.bmp", m_saveDirectory.c_str(), kinect->frameNumber,
//       (long int)timeNow.s, (long int)timeNow.us);
//   cvSaveImage(buf2, depth_data);
}


// ######################### FILE SEQUENCE PROCESSING ######################### //
void KinectSeqServer::initFileTemplates(const vector<string> &fileTemplates, int first, int last, int inc) throw(runtime_error)
{
  if(fileTemplates.size() == 0) throw runtime_error(exceptionMessage(__HERE__, "no image lists given"));
//   if(fileTemplates.size() != getNumFiles())
//     throw runtime_error(exceptionMessage(__HERE__, "number of file templates %d does not match number of camera IDs %d",
//           (int)fileTemplates.size(), getNumCams()));
  grabTimes.resize(getNumFiles());
//   grabbedImages.resize(getNumFiles());

  grabbedImage = 0;
  constructFilenames(fileTemplates, first, last, inc);
  obtainImageSize();
}

void KinectSeqServer::switchSequence(CSequenceInfo& seq)
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

void KinectSeqServer::installSequence(const std::string& name)
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

void KinectSeqServer::tryNextSequence()
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
void KinectSeqServer::getPoints(bool transformToGlobal, int imgWidth, vector<PointCloud::SurfacePoint> &points, bool complete)
{
  if(imgWidth >= 640) log("getPoints: Warning: Point clouds >= 640x480 may exceed memory limit of ice streams."); 
  
  if(noContinousGrabing) grabFramesInternal();

  while(!haveImages)
  {
    sleepComponent(100);
    log("getPoints: Warning: wait for images.");
  }
  lockComponent();

  points.resize(0);

  Pose3 global_kinect_pose;
  if(transformToGlobal)
    global_kinect_pose = camPars[0].pose;
    
  int scale = (int) pcl_cloud.width / imgWidth;
  for(unsigned y=0; y < pcl_cloud.height; y+=scale)  /// SLOW conversion
  {
    for(unsigned x=0; x < pcl_cloud.width; x+=scale)
    {
      PointCloud::SurfacePoint pt;
      pt.p.x = pcl_cloud(x, y).x;
      pt.p.y = pcl_cloud(x, y).y;   
      pt.p.z = pcl_cloud(x, y).z;   
      
//       RGBValue color;                                  /// We copy the color from the image, because pcd-files have a problem with linked openCV
//       color.float_value = pcl_cloud(x, y).rgb;
      unsigned pos = (y*grabbedImage->width + x)*3;
      pt.c.r = grabbedImage->imageData[pos];
      pt.c.g = grabbedImage->imageData[pos + 1];
      pt.c.b = grabbedImage->imageData[pos + 2];
       if(transformToGlobal)
         pt.p = transform(global_kinect_pose, pt.p);   // now get from kinect cam coord sys to global coord sys

      points.push_back(pt);
    }
  }
  unlockComponent();
}


void KinectSeqServer::getRectImage(int side, int imgWidth, Video::Image& image)
{
  while(!haveImages)
  {
    sleepComponent(100);
    log("getRectImage: Warning: wait for images.\n");
  }

  lockComponent();

  // get grabbed image and scale it, if necessary
  if(grabbedImage->width != imgWidth)
  {
    cvReleaseImage(&scaledImage);
    scaledImage = cvCreateImage(cvSize(imgWidth, imgWidth*3/4), IPL_DEPTH_8U, 3);
    cvResize(grabbedImage, scaledImage);
  convertImageFromIpl(scaledImage, image);
  }
  else
   convertImageFromIpl(grabbedImage, image);
  
  initCameraParameters(image.camPars);
  image.camPars = camPars[0];
  image.camPars.id = side;
  image.camPars.width = imgWidth;
  image.camPars.height = imgWidth*3/4;

  Pose3 global_pose, zeroPose;
  setIdentity(zeroPose);
  transform(camPars[side].pose, zeroPose, global_pose);
  image.camPars.pose = global_pose;
  image.camPars.time = getCASTTime();
    
  unlockComponent();
}

bool KinectSeqServer::getCameraParameters(Ice::Int side /*not used*/, Video::CameraParameters& _camPars)
{
  printf("KinectSeqServer::getCameraParameters: Warning: Not yet implemented!\n");
  
//   lockComponent(); // TODO: CASTComponent::Lock lock(this);
//   // TODO: we don't need the image! This is an expensive way to obtain the width
//   IplImage *rgbImage;
//   kinect->GetColorImage(&rgbImage);
//   int imgWidth = rgbImage->width;
//   double scaleFactor = camPars[0].width / imgWidth;
//   cvReleaseImage(&rgbImage);
// 
//   initCameraParameters(_camPars);
//   _camPars.id = camIds[0];
//   _camPars.width  = imgWidth;
//   _camPars.height = imgWidth * 3/4;
//   _camPars.fx = camPars[0].fx/scaleFactor;
//   _camPars.fy = camPars[0].fy/scaleFactor;
//   _camPars.cx = camPars[0].cx/scaleFactor;
//   _camPars.cy = camPars[0].cy/scaleFactor;
// 
//   Pose3 global_pose, zeroPose;
//   setIdentity(zeroPose);
//   transform(camPars[0].pose, zeroPose, global_pose);
//   _camPars.pose = global_pose;
//   _camPars.time = getCASTTime();
// 
//   unlockComponent(); // TODO: remove

  return true;
}

void KinectSeqServer::getDisparityImage(int imgWidth, Video::Image& image)
{
  printf("KinectSeqServer::getDisparityImage: Warning: Not yet implemented!\n");
/*
  lockComponent();
  convertImageFromIpl(disparityImg, image);
  unlockComponent();
*/
}

void KinectSeqServer::getRangePoints(Laser::Scan2d &KRdata)
{
  printf("KinectSeqServer::getRangePoints: Warning: Not yet implemented!\n");
}

void KinectSeqServer::getDepthMap(cast::cdl::CASTTime &time, vector<int>& depth)
{
  printf("KinectSeqServer::getDepthMap: Warning: Not yet implemented!\n");

//   lockComponent();
// 
//   //kinect->NextFrame();
//   //cv::Mat RGB;
//   //cv::Mat DEPTH;
//   //kinect->GetImages(RGB, DEPTH);
//   //kinect::readFrame();  // read next frame
//   kinect::changeRegistration(0);
//   const DepthMetaData* pDepthMD = kinect->getNextDepthMD();
//   time = getCASTTime();
// 
//   //for(int row=0;row < DEPTH.rows;row++)
//   for(size_t row=0;row < pDepthMD->YRes();row++)
//   {
//     for(size_t col=0;col < pDepthMD->XRes();col++)
//     {
//       //depth.push_back( int ( DEPTH.ptr<short>(row)[(640-col-1)] ) );
//       //depth.push_back( int ( DEPTH.ptr<short>(row)[col] ) );
//       depth.push_back( (*pDepthMD)(col,row) );
//     }
//   }
//   unlockComponent();
}

void KinectSeqServer::receiveImages(const vector<Video::Image>& images)
{
  printf("KinectSeqServer::receiveImages: Warning: Not yet implemented!\n");
}

}

