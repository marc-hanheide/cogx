/**
 * @author Andrzej Pronobis
 * @date IROS'11 deadline
 */

#include <cmath>
#include <cast/core/CASTUtils.hpp>
#include <VideoUtils.h>
#include "ProsilicaServer.h"

#include     <signal.h>
#include     <unistd.h>


static void sig_alrm(int signo)
{    /* nothing to do, just return to wake up the pause */}





// Camera unique ids
#define PROSILICA_LEFT_CAM_ID 37107 
#define PROSILICA_RIGHT_CAM_ID 37105

/*
Prosilica_PeriodMs 100
Prosilica_TriggerId 1
Prosilica_UseLeftCam 1<><------><------># 1 - Yes, 0 - No
Prosilica_UseRightCam 1><------><------># 1 - Yes, 0 - No
Prosilica_ExposureModeAuto 1<--><------># 1 - Yes, 0 - No
Prosilica_ExposureValue 15000
Prosilica_ExposureAutoAdjustDelay 0
Prosilica_ExposureAutoAdjustTol 3
Prosilica_ExposureAutoMax 15000><------># In us. Must be less than periodms*1000
Prosilica_ExposureAutoMin 6000<><------># Min. 10
Prosilica_ExposureAutoOutliers 5
Prosilica_ExposureAutoTarget 35
Prosilica_GainValue 15
Prosilica_WhitebalModeAuto 1<--><------># 1 - Yes, 0 - No
Prosilica_WhitebalValueRed 100
Prosilica_WhitebalValueBlue 300
Prosilica_WhitebalAutoAdjustDelay 0
Prosilica_WhitebalAutoAdjustTol 5
Prosilica_WhitebalAutoOutliers 5
Prosilica_MtuAuto 0
Prosilica_MtuValue 4114
Prosilica_FrequencyDivider 2<--><------># If >1, every nth image & trigger will be used (trigger Nos will be divided by n)
Prosilica_BufferSize 20><------><------># No of frames in the buffer.
Prosilica_FileFormat 0<><------><------># 0 - Half resolution RGB JPEG, 1 - Bayer8 PGM, 2 - Half resolution RGB PPM
Prosilica_VerticalImageShift -22<------># No of pixels by which the right image will be shifted down (or up if negative)
*/

const unsigned int bufferSize = 20;
const bool mtuAuto = false;
const int mtu = 4114;
const int periodMs = 100;
const int verticalImageShift = -22;



/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ProsilicaServer();
  }
}

namespace cast
{

using namespace std;

ProsilicaServer::Timer::Timer()
: count(0),
  rate(0.),
  lastRate(0.),
  changeThresh(0.),
  sigChange(false)
{
}

void ProsilicaServer::Timer::increment()
{
  // increment
  count++;

  // if this is the first time
  if(count == 1)
  {
    // start timer
    gettimeofday(&startTime, 0);
  }
  else
  {
    // current time
    timeval now;
    gettimeofday(&now, 0);
    timeval start(startTime);

    // from http://www.delorie.com/gnu/docs/glibc/libc_428.html
    // Perform the carry for the later subtraction by updating y.
    if(now.tv_usec < start.tv_usec)
    {
      int nsec = (start.tv_usec - now.tv_usec) / 1000000 + 1;
      start.tv_usec -= 1000000 * nsec;
      start.tv_sec += nsec;
    }
    if(now.tv_usec - start.tv_usec > 1000000)
    {
      int nsec = (now.tv_usec - start.tv_usec) / 1000000;
      start.tv_usec += 1000000 * nsec;
      start.tv_sec -= nsec;
    }

    // tv_usec is certainly positive.
    double diffSeconds = (double)(now.tv_sec - start.tv_sec);
    double diffMicros = (double)(now.tv_usec - start.tv_usec);

    double totalDiffSeconds = diffSeconds  + (diffMicros / 1000000.);

    // get new rate
    rate = (double)count/totalDiffSeconds;

    // diff the old and new rates
    double diffRate = abs(lastRate - rate);

    // compare diff to a percentage of the old rate
    if(diffRate > changeThresh)
    {
      sigChange = true;
      lastRate = rate;
      // get 3% of the old rate... HACK WARNING CONSTANT
      changeThresh = lastRate * 0.03;
    }
    else
    {
      sigChange = false;
    }
  }
}



ProsilicaServer::ProsilicaServer()
{
	pthread_mutex_init(&_pvMutex, NULL);
	initDone=false;
	signal(SIGALRM, sig_alrm);// == SIG_ERR

}

ProsilicaServer::~ProsilicaServer()
{
	pthread_mutex_destroy(&_pvMutex);
}


const std::string ProsilicaServer::getServerName()
{
	const std::string str("ProsilicaServer");
	return str;
}


//-------------------------------------------------------------------
void ProsilicaServer::init()
{
  // Init API
  log("Initializing API.");
  tPvErr err;
  if((err=PvInitialize()))
  {
	if (err == ePvErrResources)
		throw cast::CASTException(cast::exceptionMessage(__HERE__, "Cannot initalize the API (RESOURCES)."));
	if (err == ePvErrInternalFault)
		throw cast::CASTException(cast::exceptionMessage(__HERE__, "Cannot initalize the API (INTERNAL FAULT)."));
	throw cast::CASTException(cast::exceptionMessage(__HERE__, "Cannot initalize the API."));
  }

  // List the cameras
  log("Detecting cameras.");
  tPvCameraInfo camList[10];
  unsigned long camNo=0;
  for(int i=0; i<10; ++i)
  {
    camNo=PvCameraList(camList, 10, NULL);
    sleepComponent(100);
  }

  if (camNo==0)
  {
    throw cast::CASTException(cast::exceptionMessage(__HERE__, "No Prosilica GigE cameras detected."));
  }

  // Find cameras
  log("Found %d camera(s)", camNo);

  bool foundLeft=false;
  bool foundRight=false;
  for (unsigned int i=0; i<camNo; ++i)
  {
//    log("Found camera of unique ID: %d"+(int)(camList[i].UniqueId));
    if (camList[i].UniqueId==PROSILICA_LEFT_CAM_ID)
    {
      foundLeft=true;
      log("Found left camera.");
    }
    if (camList[i].UniqueId==PROSILICA_RIGHT_CAM_ID)
    {
      foundRight=true;
      log("Found right camera.");
    }
  }

  if (!foundLeft)
  {
      error("Left camera not found.");
	  throw cast::CASTException(cast::exceptionMessage(__HERE__, "Left camera not found!"));
  }

  // Init left camera
    // Open camera
    log("Opening left camera");
    if (PvCameraOpen(PROSILICA_LEFT_CAM_ID, ePvAccessMaster, &_leftCamHandle))
    {
      throw cast::CASTException(cast::exceptionMessage(__HERE__, "Cannot open the left camera."));
    }
    // Adjust packet size - it is vital that this is not larger than the size set on the ethernet card
    if (PvCaptureAdjustPacketSize(_leftCamHandle, 7000))
    {
        throw cast::CASTException("Cannot adjust the packet size for the left camera.");
    }

//    if (mtuAuto)
//    {
//      if (PvCaptureAdjustPacketSize(_leftCamHandle, 10000))
//      {
//    	  throw cast::CASTException("Cannot adjust the packet size for the left camera.");
//      }
//    }
//    else
//    {
//      if (PvAttrUint32Set(_leftCamHandle, "PacketSize", mtu))
//      {
//    	  throw cast::CASTException("Cannot set the packet size for the left camera.");
//      }
//    }
    tPvUint32 ps=0;
    PvAttrUint32Get(_leftCamHandle, "PacketSize", &ps);
    log("Packet size for the left camera set to %d", ps);
    // Setting camera parameters
    log("Setting left camera parameters.");
    if (!setCameraAttributes(_leftCamHandle))
    {
        error("Cannot set left camera attributes.");
    	throw cast::CASTException(cast::exceptionMessage(__HERE__, "Cannot set left camera attributes."));
    }
    // Calculating StreamBytesPerSecond
    tPvUint32 tbpf=0;
    PvAttrUint32Get(_leftCamHandle, "TotalBytesPerFrame", &tbpf);
//    tPvUint32 sbps=static_cast<tPvUint32>(static_cast<double>(tbpf)*
//                                          (1000.0/static_cast<double>(periodMs))*
//                                          3
//                                          );
    int sbps=6493836;
    if (PvAttrUint32Set(_leftCamHandle, "StreamBytesPerSecond",  sbps))
    {
    	throw cast::CASTException("Cannot set StreamBytesPerSecond for the left camera.");
    }
    log("StreamBytesPerSecond of the left camera set to %d ", sbps);
    // Create buffers
    log("Creating buffer for the left camera.");
    _frame.ImageBuffer=new char[tbpf];
    _frame.ImageBufferSize=tbpf;
    _frame.AncillaryBufferSize=0;

    // Starting capture
    log("Starting capture for the left camera.");
    if (PvCaptureStart(_leftCamHandle))
    {
    	throw cast::CASTException(cast::exceptionMessage(__HERE__, "Error invoking CaptureStart for left camera."));
    }

    sleepComponent(1000);
    // Start acquisition
    log("Starting acquisition for the left camera.");
    if (PvCommandRun(_leftCamHandle, "AcquisitionStart"))
    {
    	throw cast::CASTException(cast::exceptionMessage(__HERE__, "Cannot start acquisition for the left camera."));
    }
    log("Finished initialization.");

}




//-------------------------------------------------------------------
bool ProsilicaServer::setCameraAttributes(tPvHandle cam)
{
  // Image mode
  if (PvAttrUint32Set(cam, "BinningX", 1))
    return false;
  if (PvAttrUint32Set(cam, "BinningY", 1))
    return false;
  // ROI
  int vertShift = (verticalImageShift/2) * 2; // only dividable by 2
  unsigned int y=0;
  unsigned int vertShiftAbs=0;
  if (vertShift>0)
  {
    vertShiftAbs=vertShift;
//    if (cam==_rightCamHandle)
//      y=vertShiftAbs;
  }
  else
  {
    vertShiftAbs=-vertShift;
    if (cam==_leftCamHandle)
     y=vertShiftAbs;
  }

  log ("Settin W&H");
  if (PvAttrUint32Set(cam, "Width", 1360))
    return false;
  if (PvAttrUint32Set(cam, "Height", 1024-vertShiftAbs))
    return false;
  if (PvAttrUint32Set(cam, "RegionX", 0))
    return false;
  if (PvAttrUint32Set(cam, "RegionY", 0+y))
    return false;
  // Image format
  if (PvAttrEnumSet(cam, "PixelFormat", "Bayer8"))
    return false;
  // Acquisition control
//  if (PvAttrEnumSet(cam, "AcquisitionMode", "Continuous"))
//    return false;
  if (PvAttrEnumSet(cam, "FrameStartTriggerMode", "Freerun"))
    return false;
//  if (PvAttrEnumSet(cam, "FrameStartTriggerEvent", "EdgeRising"))
//    return false;
//  if (PvAttrUint32Set(cam, "FrameStartTriggerDelay", 0))
//    return false;
//  if (PvAttrEnumSet(cam, "AcqEndTriggerMode", "Disabled"))
//    return false;
//  if (PvAttrEnumSet(cam, "AcqStartTriggerMode", "Disabled"))
//    return false;
//  if (PvAttrEnumSet(cam, "AcqRecTriggerMode", "Disabled"))
//    return false;
  // Feature control
  log("Setting exposure.");
  if (PvAttrEnumSet(cam, "ExposureMode", "Auto"))
    return false;
  log("xx");
  if (PvAttrUint32Set(cam, "ExposureValue", 15000 ))
    return false;
  log("xx");
//  if (PvAttrUint32Set(cam, "ExposureAutoAdjustDelay", 0 ))
//    return false;
  log("xx");
  if (PvAttrUint32Set(cam, "ExposureAutoAdjustTol", 3 ))
    return false;
  log("xx");
  if (PvAttrEnumSet(cam, "ExposureAutoAlg", "Mean"))
    return false;
  log("xx");
  if (PvAttrUint32Set(cam, "ExposureAutoMax", 15000 ))
    return false;
  if (PvAttrUint32Set(cam, "ExposureAutoMin", 6000 ))
    return false;
  if (PvAttrUint32Set(cam, "ExposureAutoOutliers", 5 ))
    return false;
  if (PvAttrUint32Set(cam, "ExposureAutoRate", 100))
    return false;
  if (PvAttrUint32Set(cam, "ExposureAutoTarget", 35 ))
    return false;
  log("Setting gain");
  if (PvAttrUint32Set(cam, "GainValue", 15 ))
    return false;
  log("Setting white balance");
  if (PvAttrEnumSet(cam, "WhitebalMode", "Auto"))
    return false;
  log("xx");
  if (PvAttrUint32Set(cam, "WhitebalValueRed",100 ))
    return false;
  log("xx");
  if (PvAttrUint32Set(cam, "WhitebalValueBlue", 300 ))
    return false;
  log("xx");
//  if (PvAttrUint32Set(cam, "WhitebalAutoAdjustDelay", 0 ))
//    return false;
  log("xx");
  if (PvAttrUint32Set(cam, "WhitebalAutoAdjustTol", 5 ))
    return false;
  log("xx");
//  if (PvAttrEnumSet(cam, "WhitebalAutoAlg", "Mean"))
//    return false;
  log("xx");
//  if (PvAttrUint32Set(cam, "WhitebalAutoOutliers", 5 ))
//    return false;
  log("xx");
  if (PvAttrUint32Set(cam, "WhitebalAutoRate", 100))
    return false;

  return true;
}



void ProsilicaServer::configure(const map<string,string> & _config)
  throw(runtime_error)
{
  // first let the base class configure itself
  VideoServer::configure(_config);

  init();
}


void ProsilicaServer::grabFrames()
{
//	if (!initDone)
//	{
//		init();
//		initDone = false;
//	}


	log("Grabbing frames");
	pthread_mutex_lock(&_pvMutex);
	if(!PvCaptureQueueFrame(_leftCamHandle,&(_frame),NULL))
	{
		debug("waiting for the frame to be done ...");
		while(PvCaptureWaitForFrameDone(_leftCamHandle,&(_frame),PVINFINITE) == ePvErrTimeout)
			debug("still waiting ...");
		grabTime = getCASTTime();
		if(_frame.Status == ePvErrSuccess)
		{
			debug("frame saved\n");
		}
		else
		{
			error("the frame failed to be captured - error code %d", _frame.Status);
		}
	}
	else
	error("failed to enqueue the frame");
	pthread_mutex_unlock(&_pvMutex);
}


void ProsilicaServer::retrieveFrameInternal(int camIdx, int w, int h,
    Video::Image &frame)
{
	log("Retrieving frame");
	pthread_mutex_lock(&_pvMutex);

	frame.time = grabTime;
  frame.camId = camIdx;
  unsigned int width = _frame.Width;
//  unsigned int height = _frame.Height;
  unsigned int halfWidth =_frame.Width/2;
  unsigned int halfHeight =_frame.Height/2;
  frame.width = halfWidth;
  frame.height = halfHeight;
  frame.data.resize(halfWidth*halfHeight*3);
  unsigned char* img = reinterpret_cast<unsigned char*>(_frame.ImageBuffer);

  for(unsigned int y=0; y<halfHeight; ++y)
  {
    for(unsigned int x=0; x<halfWidth; ++x)
    {
        unsigned char r = *(img+width*(2*y)+(2*x));
        unsigned char g = static_cast<unsigned char>
                                (
                                  (static_cast<int>(*(img+width*(2*y)+(2*x+1)))+static_cast<int>(*(img+width*(2*y+1)+(2*x))))/2
                                );
        unsigned char b = *(img+width*(2*y+1)+(2*x+1));

        frame.data[3*(y*halfWidth + x) + 0] = r;
        frame.data[3*(y*halfWidth + x) + 1] = g;
        frame.data[3*(y*halfWidth + x) + 2] = b;
    }
  }

	pthread_mutex_unlock(&_pvMutex);
}





void ProsilicaServer::retrieveFrames(const std::vector<int> &camIds,
    int width, int height, std::vector<Video::Image> &frames)
{
  frames.resize(camIds.size());
  for(size_t j = 0; j < camIds.size(); j++)
  {
    size_t i = getCamIndex(camIds[j]);
    retrieveFrameInternal(i, width, height, frames[j]);
  }
}

void ProsilicaServer::retrieveFrames(int width, int height,
    std::vector<Video::Image> &frames)
{
  frames.resize(1);
  for(size_t i = 0; i < 1; i++)
    retrieveFrameInternal(i, width, height, frames[i]);
}

void ProsilicaServer::retrieveFrame(int camId, int width, int height,
    Video::Image &frame)
{
  size_t i = getCamIndex(camIds[camId]);
  retrieveFrameInternal(i, width, height, frame);
}

void ProsilicaServer::retrieveHRFrames(std::vector<Video::Image> &frames)
{
	log("ProsilicaServer::retrieveHRFrames: not yet implemented.\n");
}

void ProsilicaServer::getImageSize(int &width, int &height)
{
  width = _frame.Width/2;
  height = _frame.Height/2;
}

int ProsilicaServer::getFramerateMilliSeconds()
{
  return 100;
}

void ProsilicaServer::changeFormat7Properties(int width, int height, int offsetX, int offsetY, int mode, int paketSize)
{
	log("only for the PointGrey server available: abort.");
}

bool ProsilicaServer::inFormat7Mode()
{
	return false;
}


}

