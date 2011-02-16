/**
 * @author Michael Zillich
 * @date October 2006
 */

#include <highgui.h>
//#include <opencv/highgui.h> -- this is incorrect according to the pkg-config flags
#include <cast/core/CASTUtils.hpp>
#include <VideoUtils.h>
#include "OpenCvImgSeqServer.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::OpenCvImgSeqServer();
  }
}

namespace cast
{

using namespace std;

// default framerate if none specified is 1 s
static const int FRAMERATE_DEFAULT = 1000;

OpenCvImgSeqServer::OpenCvImgSeqServer()
{
  framerateMillis = FRAMERATE_DEFAULT;
  frameCnt = 0;
  downsampleFactor = 1;
  width = height = 0;
  frameRepeatCnt = 1;
  loopSequence = true;
}

OpenCvImgSeqServer::~OpenCvImgSeqServer()
{
  for(size_t i = 0; i < grabbedImages.size(); i++)
    cvReleaseImage(&grabbedImages[i]);
}

void OpenCvImgSeqServer::init(const vector<string> &fileTemplates,
    int first, int last, int inc) throw(runtime_error)
{
  if(fileTemplates.size() == 0)
    throw runtime_error(exceptionMessage(__HERE__, "no image lists given"));
  if(fileTemplates.size() != camIds.size())
    throw runtime_error(exceptionMessage(__HERE__,
          "number of file templates %d does not match number of camera IDs %d",
          (int)fileTemplates.size(), (int)camIds.size()));
  grabTimes.resize(getNumCameras());
  grabbedImages.resize(getNumCameras());
  for(size_t i = 0; i < grabbedImages.size(); i++)
    grabbedImages[i] = 0;
  constructFilenames(fileTemplates, first, last, inc);
  obtainImageSize();
}

/**
 * Read the first image of the sequence and store its size. This assumes of
 * course that all images in the sequence have the same size.
 * Furthermore this requires the file list to be initialised already
 */
void OpenCvImgSeqServer::obtainImageSize() throw(runtime_error)
{
  // if width and height are not set yet
  if(width == 0)
  {
    IplImage *img = 0;

    if(filenames.size() == 0)
      throw runtime_error(exceptionMessage(__HERE__, "video not initialised"));
    // load the first image and have a look at its size
    // note: we are not simply using GrabFrames() here, as that would increment
    // the counter, set times etc.
    img = cvLoadImage(filenames[0].c_str(), CV_LOAD_IMAGE_COLOR);
    if(img == 0)
      throw runtime_error(exceptionMessage(__HERE__,
          "failed to load image '%s'", filenames[0].c_str()));
    width = img->width;
    height = img->height;
    cvReleaseImage(&img);
  }
  else
    throw runtime_error(exceptionMessage(__HERE__,
        "obtainImageSize() must only be called once"));
}

/**
 * Construct filenames from file templates.
 * Each camera has a file template, e.g. img_left_%03d.jpg img_right_%03d.jpg
 */
void OpenCvImgSeqServer::constructFilenames(const vector<string> &fileTemplates,
    int first, int last, int inc)
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

void OpenCvImgSeqServer::grabFramesInternal() throw(runtime_error)
{
  if(filenames.size() == 0)
    throw runtime_error(exceptionMessage(__HERE__, "video not initialised"));
  if(frameCnt < numFrames() || loopSequence)
  {
    // number of current frame, note that we loop
    int fn = frameCnt % numFrames();
    for(size_t i = 0; i < grabbedImages.size(); i++)
    {
      cvReleaseImage(&grabbedImages[i]);
      grabbedImages[i] = cvLoadImage(filenames[fn*getNumCameras() + i].c_str(),
          CV_LOAD_IMAGE_COLOR);
      if(grabbedImages[i] == 0)
        throw runtime_error(exceptionMessage(__HERE__,
              "failed to load image '%s'",
              filenames[fn*getNumCameras() + i].c_str()));
      if(grabbedImages[i]->width != width || grabbedImages[i]->height != height)
        throw runtime_error(exceptionMessage(__HERE__,
              "size of loaded image '%s': %dx%d does not match video size %dx%d",
              filenames[fn*getNumCameras() + i].c_str(),
              grabbedImages[i]->width, grabbedImages[i]->height,
              width, height));
    }
  }
  else
  {
    // return empty images
    for(size_t i = 0; i < grabbedImages.size(); i++)
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
}

void OpenCvImgSeqServer::grabFrames()
{
  // note that by just calling sleep(framrate) we actually do not really
  // get a fixed framerate, this would require setitimer() and pause()
  sleepComponent(framerateMillis);
  grabFramesInternal();
}

void OpenCvImgSeqServer::retrieveFrameInternal(int camIdx, int width, int height,
    Video::Image &frame)
{
  // To handle the case where retrieve is called before any Grab
  if(!haveFrames())
    grabFramesInternal();

  frame.time = grabTimes[camIdx];
  frame.camId = camIds[camIdx];
  frame.camPars = camPars[camIdx];

  // no size given, use native size
  if((width == 0 || height == 0) || (width == this->width && height == this->height))
  {
    convertImageFromIpl(grabbedImages[camIdx], frame);
    // adjust to native size
    // (note that calibration image size need not be the same as currently set
    // native capture size)
    changeImageSize(frame.camPars, this->width, this->height);
  }
  else
  {
    IplImage *tmp = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    cvResize(grabbedImages[camIdx], tmp);
    convertImageFromIpl(tmp, frame);
    // TODO: avoid allocate/deallocating all the time
    cvReleaseImage(&tmp);
    // adjust to scaled image size
    changeImageSize(frame.camPars, width, height);
  }
}

void OpenCvImgSeqServer::retrieveFrames(const std::vector<int> &camIds,
    int width, int height, std::vector<Video::Image> &frames)
{
  frames.resize(camIds.size());
  for(size_t j = 0; j < camIds.size(); j++)
  {
    size_t i = getCamIndex(camIds[j]);
    retrieveFrameInternal(i, width, height, frames[j]);
  }
}

void OpenCvImgSeqServer::retrieveFrames(int width, int height,
    vector<Video::Image> &frames)
{
  frames.resize(getNumCameras());
  for(size_t i = 0; i < (size_t)getNumCameras(); i++)
    retrieveFrameInternal(i, width, height, frames[i]);
}

void OpenCvImgSeqServer::retrieveFrame(int camId, int width, int height,
    Video::Image &frame)
{
  size_t i = getCamIndex(camIds[camId]);
  retrieveFrameInternal(i, width, height, frame);
}

void OpenCvImgSeqServer::retrieveHRFrames(std::vector<Video::Image> &frames)
{
	printf("OpenCvImgSeqServer::retrieveHRFrames: not yet implemented.\n");
}

/**
 * Returns whether any frames have been grabbed yet.
 */
bool OpenCvImgSeqServer::haveFrames()
{
  for(int i = 0; i < getNumCameras(); i++)
    if(grabbedImages[i] == 0)
      return false;
  return true;
}

void OpenCvImgSeqServer::getImageSize(int &width, int &height)
{
  width = this->width;
  height = this->height;
}

int OpenCvImgSeqServer::getFramerateMilliSeconds()
{
  return framerateMillis;
}

void OpenCvImgSeqServer::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  int start = 0, end = 0, step = 1;
  vector<string> fileTemplates;
  map<string,string>::const_iterator it;

  // first let the base class configure itself
  VideoServer::configure(_config);

  if((it = _config.find("--files")) != _config.end())
  {
    istringstream str(it->second);
    string file;
    while(str >> file)
      fileTemplates.push_back(file);
  }
  if((it = _config.find("--start")) != _config.end())
  {
    istringstream str(it->second);
    str >> start;
    if(start < 0)
      start = 0;
  }
  if((it = _config.find("--end")) != _config.end())
  {
    istringstream str(it->second);
    str >> end;
    if(end < 0)
      end = 0;
  }
  if((it = _config.find("--step")) != _config.end())
  {
    istringstream str(it->second);
    str >> step;
    if(step <= 0)
      step = 1;
  }
  if((it = _config.find("--repeatframe")) != _config.end())
  {
    istringstream str(it->second);
    str >> frameRepeatCnt;
    if(frameRepeatCnt <= 0)
      frameRepeatCnt = 1;
  }
  if((it = _config.find("--framerate_ms")) != _config.end())
  {
    istringstream str(it->second);
    str >> framerateMillis;
    if(framerateMillis <= 0)
      framerateMillis = FRAMERATE_DEFAULT;
  }
  if((it = _config.find("--downsample")) != _config.end())
  {
    istringstream str(it->second);
    str >> downsampleFactor;
    if(downsampleFactor <= 0)
      downsampleFactor = 1;
  }
  if((it = _config.find("--noloop")) != _config.end())
  {
    loopSequence = false;
  }

  // do some initialisation based on configured items
  init(fileTemplates, start, end, step);
}

/**
 * @brief This function is only for the PointGrey server available: experimental mode.
 * @param width Image width
 * @param height Image height
 * @param offsetX Offset in x- direction for Format7 mode.
 * @param offsetY Offset in y- direction for Format7 mode.
 * @param mode Image grabbing mode for the Format7 mode.
 * @param fps Requested framerate [1/s]
 */
void OpenCvImgSeqServer::changeFormat7Properties(int width, int height, int offsetX, int offsetY, int mode, int paketSize)
{
	log("only for the PointGrey server available: abort.");
}

/**
 * @brief Camera is not in Format7 mode.
 * @return Returns false, when cameras not in Format7 mode.
 */
bool OpenCvImgSeqServer::inFormat7Mode()
{
	return false;
}

/**
 * @brief Get the server name
 * @param name Server name (PointGreyServer / OpenCvImgSeqServer / OpenCvLiveServer)
 */
const std::string OpenCvImgSeqServer::getServerName()
{
	const std::string str("OpenCvImgSeqServer");
	return str;
}

}
