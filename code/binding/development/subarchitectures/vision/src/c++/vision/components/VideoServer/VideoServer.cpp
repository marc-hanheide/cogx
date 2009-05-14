/**
 * TODO: move ringbuffer into VideoDevice class, only convert images if needed!
 */

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <vision/idl/Vision.hh>
#include <vision/utils/VisionUtils.h>
#include "VideoSrv.hpp"
#include "ImgSeqDevice.h"
#include "OpenCvDevice.h"
#include "VideoServer.h"

#ifndef __APPLE__
#include "V4LDevice.h"
#endif

using namespace Vision;

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new VideoServer(_id);
  }
}


VideoServer::VideoServer(const string &_id)
: 
  WorkingMemoryAttachedComponent(_id),
  UnmanagedProcess(_id)
{
  video = 0;
  curFrame = new ImageFrame();
  //ring_idx = -1;
  connHub = 0;
}

VideoServer::~VideoServer()
{
  delete connHub;
  //FreeRingBuffer();
  delete curFrame;
  delete video;
}

VideoDevice* VideoServer::ConfigureImgSeq(map<string,string> &_config)
{
  int start, end, framerate_ms, step, first_id;
  vector<int> cam_ids;
  vector<string> file_templates;

  // note that strtol returns 0 for an empty string
  start        = strtol(_config["-s"].c_str(), NULL, 10);
  end          = strtol(_config["-e"].c_str(), NULL, 10);
  framerate_ms = strtol(_config["-t"].c_str(), NULL, 10);
  step         = strtol(_config["-d"].c_str(), NULL, 10);
  first_id     = strtol(_config["-i"].c_str(), NULL, 10);

  // chose useful default values
  if(step <= 0)
    step = 1; // chose a default step of 1
  if(framerate_ms <= 0)
    framerate_ms = 1000; // chose a default framerate of 1 img/sec
  if(end == 0)
    throw CASTException(__HERE__, "need end frame of img sequence");

  cam_ids.push_back(first_id);
  file_templates.push_back(_config["-f"]);
  // if stored stereo
  if(_config["-g"] != "")
  {
    cam_ids.push_back(first_id + 1);
    file_templates.push_back(_config["-g"]);
  }
  return new ImgSeqDevice(cam_ids, file_templates, start, end, framerate_ms,
      step);
}

VideoDevice* VideoServer::ConfigureV4L(map<string,string> &_config)
{

#ifdef __APPLE__
  throw new CASTException(__HERE__, "Video4Linux device not available on Mac platforms.");
#else
  int first_id, width = 640, height = 480;  // HACK: get from config
  vector<int> cam_ids;
  vector<string> dev_names;

  // note that strtol returns 0 for an empty string
  first_id = strtol(_config["-i"].c_str(), NULL, 10);

  cam_ids.push_back(first_id);
  dev_names.push_back(_config["-v"]);
  // if stereo
  if(_config["-w"] != "")
  {
    cam_ids.push_back(first_id + 1);
    dev_names.push_back(_config["-w"]);
  }
  return new V4LDevice(cam_ids, dev_names, width, height);
#endif
}

VideoDevice* VideoServer::ConfigureOpenCv(map<string,string> &_config)
{
  int first_id;
  vector<int> cam_ids;
  vector<int> dev_nums;
  string bayer;

  // note that strtol returns 0 for an empty string
  first_id = strtol(_config["-i"].c_str(), NULL, 10);

  cam_ids.push_back(first_id);
  dev_nums.push_back(strtol(_config["-o"].c_str(), NULL, 10));
  // if stereo
  if(_config["-p"] != "")
  {
    cam_ids.push_back(first_id + 1);
    dev_nums.push_back(strtol(_config["-p"].c_str(), NULL, 10));
  }
  // if cameras return raw Bayer patterns
  if(_config["--bayer"] != "")
  {
    bayer = _config["--bayer"];

  }
  return new OpenCvDevice(cam_ids, dev_nums, bayer);
}

/**
 * Configure the server for different video sources.
 * Options are:
 * -i <num>  camera ID of the first camera, other IDs by increment, default 0
 * -f <file pattern>  use mono image sequence device (stored images). Pattern
 *     is a C printf-style pattern, e.g.  demo-%03d.jpg  to get images
 *     demo-000.jpg, demo-001.jpg etc.
 * -g <file pattern>  together with -f use stored stereo sequence, where '-f' is
 *     assigned camera id '-i', and '-g' is assigned '-i' + 1
 *     Note that -g alone is undefined.
 * -s <num>  start number for image sequence, e.g. 2 -> demo002.jpg ...
 * -e <num>  end number for images sequence
 * -d <num>  increment (delta) for image sequence numbers, default 1
 * -t <num>  frame rate (in ms per frame) for image sequence, default 1000
 * -v <device>  Video4Linux device, e.g. /dev/video0
 * -w <device>  Video4Linux device, e.g. /dev/video1, together with -v for
 *     stereo note that -w alone is undefined.
 * -o <num>  use OpenCv for video capture, use device <num> (e.g. /dev/video0)
 *     note that OpenCv finds various devices automatically.
 * -p <num>  use OpenCv for video capture, use device <num> (e.g. /dev/video0)
 *     together with -o for stereo
 *     Note that -p alone is undefined.
 * --bayer <format>  for OpenCV devce: whether raw Bayer to RGB converion
 *     should be performed (e.g. for the Point Grey Flea). Format must be
 *     one of:  BGGR  GBBR  RGGB  GRRB
 * --downsample .. downsample factor, optional, default 1
 * --videoport <num> .. the port number for video communication.
 *   When a prot is specified in VideoServer, 
 *   the same port number must also be specified for 
 *   all components that request images from it.
 */
void VideoServer::configure(map<string,string> & _config)
{
  // first let the base class configure itself
  UnmanagedProcess::configure(_config);

  if(_config["-f"] != "")
    video = ConfigureImgSeq(_config);
  else if(_config["-v"] != "")
    video = ConfigureV4L(_config);
  else if(_config["-o"] != "")
    video = ConfigureOpenCv(_config);
  // check if any video source was specified
  if(video == 0)
    throw CASTException(__HERE__, "no video source was specified");

  downsample = strtol(_config["--downsample"].c_str(), NULL, 10);
  // make sure downsample has a meaningful value (strtol might return 0)
  if(downsample == 0)
    downsample = 1;

  // port number for extra socket connection
  int port = strtol(_config["--port"].c_str(), NULL, 10);
  initSocketComm(port);

  //AllocateRingBuffer();
}

void VideoServer::initSocketComm(int listenPort)
{
  connHub = new ConnHub(listenPort);
  connHub->addConnectionFactory(
      new ConnectionFactory<VideoServer, VideoSrv>(this));
}

void VideoServer::runComponent()
{
  // add image buffer states to WM, with invalid times for a start
  vector<int> camNums;
  // bool first_capture = true;

  video->GetCameraIds(camNums);
  //stateIds.resize(camNums.size());

  while(m_status == STATUS_RUN)
  {
    // process any pending image requests
    connHub->waitAndProcessMessages(false);

    lockProcess();

    // grab frames (and just store internally in video device), blocking
    video->GrabFrames();

    // update image buffer state with current frames
    // TODO: should time come from video device?
    /*BALTTime time = BALTTimer::getBALTTime();
    // TODO: how much CPU does this cost?
    for(unsigned i = 0; i < cams.size(); i++)
    {
      ImageBufferState *state = new ImageBufferState();
      state->m_camNum = cams[i].m_num;
      state->m_time = time;
      if(!first_capture)
      {
        overwriteWorkingMemory(stateIds[i],
          VisionOntology::IMAGE_BUFFER_STATE_TYPE, state);
      }
      else
      {
        stateIds[i] = newDataID();
        addToWorkingMemory(stateIds[i], VisionOntology::IMAGE_BUFFER_STATE_TYPE,
          state);
        first_capture = false;
      }
    }*/
    unlockProcess();
  }
}

void VideoServer::retrieveCurrentFrame(int camNum)
{
  lockProcess();
  if(m_status == STATUS_RUN)
  {
    if(downsample == 1)
    {
      video->RetrieveFrame(camNum, curFrame);
    }
    else
    {
      ImageFrame *fullFrame = new ImageFrame();
      video->RetrieveFrame(camNum, fullFrame);
      DownsampleImage(fullFrame, curFrame, downsample);
      delete fullFrame;
      fullFrame = NULL;
    }
  }
  unlockProcess();
}

const ImageFrame& VideoServer::getCurrentFrame()
{
  return *curFrame;
}

void VideoServer::receivePullQuery(const FrameworkQuery & _query, 
				   FrameworkLocalData<ImageFrame> * & _pData)
{
  lockProcess();
  if(m_status == STATUS_RUN)
  {
    int camNum;
    string src = _query.getSource();
    string query = _query.getQuery();
    istringstream myStream(query);
 
    if(!(myStream >> camNum))
      throw CASTException(__HERE__, "not a valid query for image frame");

    // pull consumes memory, so we have to make copy here
    video->RetrieveFrame(camNum, curFrame);
    ImageFrame *copy = new ImageFrame();
    if(downsample == 1) {
      *copy = *curFrame;
    }
    else {
      DownsampleImage(curFrame, copy, downsample);
    }
    _pData = new FrameworkLocalData<ImageFrame>(getProcessIdentifier(), copy);
    if(m_bLogOutput) {
      // hack for simplicity
      if(src != "change.detector") {
        ostringstream outStream;
        outStream<<"GET"<<" "<<BALTTimer::getBALTTime()<<" "<<src<<" Vision::ImageFrame";
        log(outStream.str());
      }
    }
  }
  unlockProcess();
}

/**
 * Find the set frame for given camera id not older than given time.
 */
/*ImageFrame* VideoServer::FindFrame(int camNum, BALTTime time)
{
  // TODO: this is stupid! we should count backwards from the current index,
  //  not forward from the next index!
  int i = (ring_idx + 1) % ring_buf.size();
  int cnt = 0;  // makes sure we don't loop endlessly
  bool found = false;
  while(!found && cnt < ring_buf.size())
  {
    if(ring_buf[i]->m_camNum == camNum && ring_buf[i]->m_time >= time)
      found = true;
    else
      i = (i + 1) % ring_buf.size();
    cnt++;
  }
  if(found)
    return ring_buf[i];
  else
  {
    // HACK: this seems to happen from time to time, so repeat the above and
    // print the detailed steps
    int i = (ring_idx + 1) % ring_buf.size();
    int cnt = 0;  // makes sure we don't loop endlessly
    bool found = false;
    printf("****** error in VideoServer: could not find frame ******\n");
    while(!found && cnt < ring_buf.size())
    {
      printf("ring idx %d:  ", i);
      printf(" cam num %d time %d:%06d\n", ring_buf[i]->m_camNum,
         ring_buf[i]->m_time.m_s, ring_buf[i]->m_time.m_us);
      if(ring_buf[i]->m_camNum == camNum && ring_buf[i]->m_time >= time)
        found = true;
      else
        i = (i + 1) % ring_buf.size();
      cnt++;
    }
    printf("****** end error in VideoServer ******\n");
    throw CASTException(__HERE__, "found no frame for cam %d at %d:%06d", camNum,
        time.m_s, time.m_us);
  }
}

void VideoServer::AllocateRingBuffer()
{
  int width, height, num_frames;
  num_frames = max(1, BUFFERED_TIME/video->GetFramerate());

  video->GetImageSize(width, height);
  ring_buf.resize(num_frames*video->NumCameras());
  printf("%d cameras, allocating %d buffers for %d frames\n",
    video->NumCameras(), ring_buf.size(), num_frames);
  for(unsigned i = 0; i < ring_buf.size(); i++)
  {
    ring_buf[i] = new ImageFrame();
    ring_buf[i]->m_width = width;
    ring_buf[i]->m_height = height;
    ring_buf[i]->m_image.length(3*width*height);
    ring_buf[i]->m_time.m_s = -1;
    ring_buf[i]->m_time.m_us = -1;
    ring_buf[i]->m_camNum = -1;
  }
  ring_idx = 0;
}

void VideoServer::FreeRingBuffer()
{
  for(unsigned i = 0; i < ring_buf.size(); i++)
    delete ring_buf[i];
}*/

void VideoServer::redrawGraphics2D()
{
  lockProcess();
  vector<int> camIds;
  video->GetCameraIds(camIds);
  // display the current image of the first camera of the video.
  // (not that in most cases a video has only one camera anyway)
  if(camIds.size() > 0)
  {
    ImageFrame *curFrame = new ImageFrame();
    vector<char> buf;
    video->RetrieveFrame(camIds[0], curFrame);
    buf.resize(curFrame->m_image.length());
    for(unsigned p = 0; p < (unsigned)curFrame->m_image.length(); p++)
      buf[p] = curFrame->m_image[p];
    drawRGBImage(curFrame->m_width, curFrame->m_height, buf, 0);
    delete curFrame;
  }
  unlockProcess();
}

void VideoServer::redrawGraphicsText()
{
  lockProcess();
  printText("Num cameras: %d\nFramerate [ms]: %d\n", video->NumCameras(),
    video->GetFramerate());
  unlockProcess();
}
