/**
 * @file PointGrayServer.cpp
 * @author Andreas Richtsfeld, Michael Zillich
 * @date Februrary 2010, Februar 2009
 * @version 0.1
 * @brief Video server for the PointGray stereo cameras.
 */

#include <cmath>
#include <cast/core/CASTUtils.hpp>
#include <VideoUtils.h>
#include "PointGreyServer.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr newComponent() {
    return new cast::PointGreyServer();
  }
}


namespace cast
{

using namespace std;

#define USE_WRAPIMAGE_HACK 1
#ifdef USE_WRAPIMAGE_HACK
#include <ConvertImage.h>
IplImage* wrapFlyCaptureImage(const FlyCapture2::Image &img);
#endif

/**
 * @brief Property type names from the PointGrey cameras.
 */
static const char propTypeNames [][30] =
{
  "BRIGHTNESS",									// Is per default (10010) manual and (valueA = 0)
  "AUTO_EXPOSURE",							/// Is per default (10011) (valueA)
  "SHARPNESS",									/// Is per default (10011) (valueA)
  "WHITE_BALANCE",							/// Is per default (10011) (valueA-valueB)
  "HUE",												// Is per default (10000) off and manual(valueA=2048)
  "SATURATION",									/// Is per default (10011) (valueA-absValue)
  "GAMMA",											// Is per default (10000) off and manual (valueA=1024-absValue=1.0)
  "IRIS",												// Not available
  "FOCUS",											// Not available
  "ZOOM",												// Not available
  "PAN",												// Not available
  "TILT",												// Not available
  "SHUTTER",										/// Is per default (10011) (valueA-absValue)
  "GAIN",												/// Is per default (10011) (valueA)
  "TRIGGER_MODE",								// Is per default (10000) off and manual (valueA=0-absValue=0)
  "TRIGGER_DELAY",							// Is per default (10000) off and manual (valueA=1024-absValue=1.0)
  "FRAME_RATE",									// Is per default (10011) (valueA=480-absValue=30.0)
  "TEMPERATURE",								// Not available
  "UNSPECIFIED_PROPERTY_TYPE"
};

/**
 * @brief Active properties (0 ... inactive, 1 ... active).
 */
static int propEnabled[sizeof(propTypeNames)/30] = { 0, 1, 1, 1, 0, 1, 0, 0 ,0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0};

/**
 * @brief Active properties (0 ... inactive, 1 ... active). Default values for the initialization.
 */
static int propActive[sizeof(propTypeNames)/30] = { 0, 1, 1, 1, 0, 1, 0, 0 ,0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0};

static const char bools [][10] =
{
  "FALSE",
  "TRUE"
};

/**
 * @brief Constructor of class MeanRate.
 */
PointGreyServer::MeanRate::MeanRate(int size) : mean(size)
{
  gettimeofday(&prev, 0);
}

/**
 * @brief Returns the current rate as 1/(current time - last time) or 0 if current time
 * == last time
 * @return Returns the current calculated (frame-) rate.
 */
float PointGreyServer::MeanRate::calculateCurrentRate()
{
  timeval now;

  gettimeofday(&now, 0);
  // from http://www.delorie.com/gnu/docs/glibc/libc_428.html
  // Perform the carry for the later subtraction by updating y.
  if(now.tv_usec < prev.tv_usec)
  {
    int nsec = (prev.tv_usec - now.tv_usec) / 1000000 + 1;
    prev.tv_usec -= 1000000 * nsec;
    prev.tv_sec += nsec;
  }
  if(now.tv_usec - prev.tv_usec > 1000000)
  {
    int nsec = (now.tv_usec - prev.tv_usec) / 1000000;
    prev.tv_usec += 1000000 * nsec;
    prev.tv_sec -= nsec;
  }
  // tv_usec is certainly positive.
  float diffSeconds = (float)(now.tv_sec - prev.tv_sec);
  float diffMicros = (float)(now.tv_usec - prev.tv_usec);
  float totalDiffSeconds = diffSeconds  + diffMicros / 1000000.;
  prev = now;
  //return fpclassify(totalDiffSeconds) != FP_ZERO ? 1./totalDiffSeconds : 0.;
  // return frame rate in milliseconds
  return totalDiffSeconds*1000.;
}


/**
 * @brief Constructor of class PointGreyServer.
 */
PointGreyServer::PointGreyServer()
{
  cameras = 0;
  width = height = 0;
  offsetX = offsetY = 0;
  paketSize = 0;
  useVideoMode7 = false;
  setAutomaticPropertyAdjustment = false;
}


/**
 * @brief Destructor of class PointGreyServer.
 */
PointGreyServer::~PointGreyServer()
{
  for(size_t i = 0; i < (size_t)getNumCameras(); i++)
  {
    cameras[i]->StopCapture();
    cameras[i]->Disconnect();
    delete cameras[i];
  }
  delete[] cameras;
}


/**
 * @brief Log camera information to the console.
 * @param pCamInfo FlyCapture camera info.
 */
void PointGreyServer::LogCameraInfo(FlyCapture2::CameraInfo* pCamInfo)
{
  log("\n\n*** CAMERA INFORMATION ***\n"
      "Serial number - %u\n"
      "Camera model - %s\n"
      "Camera vendor - %s\n"
      "Sensor - %s\n"
      "Resolution - %s\n"
      "Firmware version - %s\n"
      "Firmware build time - %s\n",
      pCamInfo->serialNumber,
      pCamInfo->modelName,
      pCamInfo->vendorName,
      pCamInfo->sensorInfo,
      pCamInfo->sensorResolution,
      pCamInfo->firmwareVersion,
      pCamInfo->firmwareBuildTime );
}

/**
 * @brief Log camera configuration to the console
 */
void PointGreyServer::LogCameraConfig()
{
  for(size_t i = 0; i < (size_t)getNumCameras(); i++)
  {
    FlyCapture2::FC2Config pConfig;
    cameras[i]->GetConfiguration(&pConfig);

    pConfig.numBuffers = 0;
    pConfig.grabMode = FlyCapture2::DROP_FRAMES;
    pConfig.isochBusSpeed = FlyCapture2::BUSSPEED_S800;
    pConfig.asyncBusSpeed = FlyCapture2::BUSSPEED_S800;
    cameras[i]->SetConfiguration(&pConfig);

    cameras[i]->GetConfiguration(&pConfig);

    log("\n\n*** CAMERA CONFIGURATION ***\n"
        "Number of buffers: %i\n"
        "Number of image notifications: %i\n"
        "Grab timeout: %i\n"
        "Grab mode: %i\n"
        "Isochronous bus speed: %i\n"
        "Asyncronous bus speed: %i\n"
        "Bandwidth allocation: %i\n",
        pConfig.numBuffers,
        pConfig.numImageNotifications,
        (int) pConfig.grabTimeout,
        (int) pConfig.grabMode,
        (int) pConfig.isochBusSpeed,
        (int) pConfig.asyncBusSpeed,
        (int) pConfig.bandwidthAllocation);
  }
}

/**
 * @brief Log camera property information to the console.
 * @param pPropInfo FlyCapture property info.
 */
void PointGreyServer::LogPropertyInfo(FlyCapture2::PropertyInfo* pPropInfo)
{

  log(" CAMERA PROPERTY INFORMATION\n"
      "Property Type:		%s\n"
      "Present:		%s\n"
      "Auto supported:		%s\n"
      "Manual supported:	%s\n"
      "On-Off supported:	%s\n"
      "Abs-Val supported:	%s\n"
      "ReadOut supported:	%s\n"
      "Minimum value:		%u\n"
      "Maximum value:		%u\n"
      "Absolut minimum:	%4.3f\n"
      "Absolut maximum:	%4.3f\n"
      "Units [abr.]:		%s [%s]\n",
      propTypeNames[pPropInfo->type],
      bools[pPropInfo->present],
      bools[pPropInfo->autoSupported],
      bools[pPropInfo->manualSupported],
      bools[pPropInfo->onOffSupported],
      bools[pPropInfo->absValSupported],
      bools[pPropInfo->readOutSupported],
      pPropInfo->min,
      pPropInfo->max,
      pPropInfo->absMin,
      pPropInfo->absMax,
      pPropInfo->pUnits,
      pPropInfo->pUnitAbbr);
}

/**
 * @brief Get property from camera.
 * @param propType Proporty type
 */
void PointGreyServer::GetPropertyInfo(int camId, FlyCapture2::PropertyType propType, FlyCapture2::PropertyInfo* pPropInfo)
{
  pPropInfo->type = propType;

  FlyCapture2::Error error;
  error = cameras[camId]->GetPropertyInfo(pPropInfo);
  if(error != FlyCapture2::PGRERROR_OK)
    throw runtime_error(error.GetDescription());
}

/**
 * @brief Log camera property information to the console.
 * @param pPropInfo FlyCapture property info.
 */
void PointGreyServer::LogProperty(FlyCapture2::Property* pProp)
{
  log("CAMERA PROPERTY:\n"
      "Property Type:		%s\n"
      "Present:		%s\n"
      "Absolut control:	%s\n"
      "One push:		%s\n"
      "On/Off:			%s\n"
      "Auto / Manual:		%s\n"
      "Value A:		%u\n"
      "Value B:		%u\n"
      "Abs. value:		%4.3f\n",
      propTypeNames[pProp->type],
      bools[pProp->present],
      bools[pProp->absControl],
      bools[pProp->onePush],
      bools[pProp->onOff],
      bools[pProp->autoManualMode],
      pProp->valueA,
      pProp->valueB,
      pProp->absValue);
}

/**
 * @brief Log camera property values to the console.
 * @param pPropInfo FlyCapture property info.
 */
void PointGreyServer::LogPropertyValues(FlyCapture2::Property* pProp)
{
  log("CAMERA PROPERTY:\n"
      "Property Type:		%s\n"
      "Value A:		%u\n"
      "Value B:		%u\n"
      "Abs. value:		%4.3f\n",
      propTypeNames[pProp->type],
      pProp->valueA,
      pProp->valueB,
      pProp->absValue);
}

/**
 * @brief Get property from camera.
 * @param propType Proporty type
 */
void PointGreyServer::GetProperty(int camId, FlyCapture2::PropertyType propType, FlyCapture2::Property* pProp)
{
  pProp->type = propType;

  FlyCapture2::Error error;
  error = cameras[camId]->GetProperty(pProp);
  if(error != FlyCapture2::PGRERROR_OK)
    throw runtime_error(error.GetDescription());
}

/**
 * @brief Set property control of camera manual/auto.
 * @param camId Camera ID
 * @param propType Proporty type
 * @param manual Property control manual.
 */
void PointGreyServer::SetPropertyManual(int camId, FlyCapture2::PropertyType propType, bool manual)
{
  FlyCapture2::Property pProp;
  GetProperty(camId, propType, &pProp);
  pProp.autoManualMode = !manual;

  FlyCapture2::Error error;
  error = cameras[camId]->SetProperty(&pProp);
  if(error != FlyCapture2::PGRERROR_OK)
    throw runtime_error(error.GetDescription());
}


/**
 * @brief Set property value for camera.
 * @param camId Camera ID
 * @param propType Proporty type
 * @param value Integer value of the property
 */
void PointGreyServer::SetPropertyValue(int camId, FlyCapture2::PropertyType propType, int valueA, int valueB, float absValue)
{
  FlyCapture2::Property pProp;
  GetProperty(camId, propType, &pProp);
  pProp.valueA = valueA;
  pProp.valueB = valueB;
  pProp.absValue = absValue;

  FlyCapture2::Error error;
  error = cameras[camId]->SetProperty(&pProp);
  if(error != FlyCapture2::PGRERROR_OK)
    throw runtime_error(error.GetDescription());
}


/**
 * @brief Set the properties for all cameras except the first one to manual mode. \n
 * The first camera will become the "property master".
 */
void PointGreyServer::SetAllPropertiesManual()
{
  for(size_t i = 0; i < (size_t)getNumCameras(); i++)
  {
    if (i == 0) // set the camera properties of the first camera to auto.
      for(unsigned j=0; j< sizeof(propTypeNames)/30; j++)
        if(propActive[j] == 1)
          SetPropertyManual(i, (FlyCapture2::PropertyType) j, false);
    if (i > 0) // set later camera properties auto or manual (propActive)
      for(unsigned j=0; j< sizeof(propTypeNames)/30; j++)
      {
        if(propActive[j] == 1)
          SetPropertyManual(i, (FlyCapture2::PropertyType) j, true);
        else if(propEnabled[j] == 1)
          SetPropertyManual(i, (FlyCapture2::PropertyType) j, false);
      }
  }
}

/**
 * @brief Copy all property values from the first camera to all others. \n
 * The first camera is then the "property master".
 */
void PointGreyServer::CopyAllPropertyValues()
{
  static int counter = 0;
  counter++;
  if (counter > realFps)	// copy the properties every second
  {
    counter = 0;
    FlyCapture2::Property pProp;
    for(unsigned j=0; j< sizeof(propTypeNames)/30; j++)
    {
      if(propActive[j] == 1)
      {
        for(size_t i = 0; i < (size_t)getNumCameras(); i++)
        {
          if (i == 0) GetProperty(0, (FlyCapture2::PropertyType) j, &pProp);
          if (i > 0) SetPropertyValue(i, (FlyCapture2::PropertyType) j, pProp.valueA, pProp.valueB, pProp.absValue);
        }
      }
    }
  }
}


/**
 * @brief Set for all camares the same properties. \n
 * The first camera is then the "property master". All other cams will get the same properties.
 */
void PointGreyServer::SetAutomaticPropertyAdjustment()
{
  setAutomaticPropertyAdjustment = true;

  SetAllPropertiesManual();
  CopyAllPropertyValues();
}

/**
 * @brief Save the properties for the Format7 mode. \n
 */
void PointGreyServer::SetFormat7Properties(int w, int h, int offX, int offY, int vMode, int pSize)
{
  width = w;
  height = h;
  offsetX = offX;
  offsetY = offY;
  paketSize = pSize;
  videoMode = vMode;
  useVideoMode7 = true;
}


/**
 * @brief Check, if the camera is in Format7 mode. \n
 * @NOTE Please consider, that this function deletes the current \n
 * Format7 properties: paket-size ect.
 * @param camId Id of the camera
 * @return Returns true, if camera is in Format7 mode.
 */
bool PointGreyServer::IsCurrentlyInFormat7(int camId)
{
  FlyCapture2::Error error;
  FlyCapture2::VideoMode currVideoMode;
  FlyCapture2::FrameRate currFrameRate;

  error = cameras[camId]->GetVideoModeAndFrameRate( &currVideoMode, &currFrameRate );
  if ( error != FlyCapture2::PGRERROR_OK ) throw runtime_error(error.GetDescription());
  return (currVideoMode == FlyCapture2::VIDEOMODE_FORMAT7);
}


/**
 * @brief Get the Format7 image parameters directly from the camera registers.\n
 * @NOTE This function maybe does not work right. Consider that the camId is hard-coded!
 * @param mode Camera mode
 * @param pLeft Left pixel offset
 * @param pTop Top pixel offset
 * @param pWidth Image width in pixel
 * @param pHeight Image height in pixel
 * @return Returns true for success.
 */
bool PointGreyServer::GetFormat7ImageParametersFromCamera(FlyCapture2::Mode mode, unsigned int* pLeft, unsigned int* pTop, unsigned int* pWidth, unsigned int* pHeight)
{
  int camId = 0;							/// camID
  FlyCapture2::Error error;

  // Get the proper mode offset
  unsigned int modeOffset = 0;
  unsigned int modeOffsetRegister = 0x2E0 + (4 * mode);

  error = cameras[camId]->ReadRegister( modeOffsetRegister, &modeOffset );
  if( error != FlyCapture2::PGRERROR_OK )
  {
    return false;
  }

  modeOffset *= 4;
  modeOffset &= 0x000FFFFF;

  unsigned int imageSize;
  error = cameras[camId]->ReadRegister( modeOffset + 0x008, &imageSize );
  if( error != FlyCapture2::PGRERROR_OK )
  {
    return false;
  }

  *pLeft = imageSize >> 16;
  *pTop = imageSize & 0x0000FFFF;

  unsigned int imagePosition;
  error = cameras[camId]->ReadRegister( modeOffset + 0x00C, &imagePosition );
  if( error != FlyCapture2::PGRERROR_OK )
  {
    return false;
  }

  *pWidth = imagePosition >> 16;
  *pHeight = imagePosition & 0x0000FFFF;

  return true;
}


/**
 * @brief Set properties for VideoMode7.
 * @param camId Id of the camera.
 */
void PointGreyServer::SetVideoMode7(int camId)
{
  FlyCapture2::Error error;

  // Save the current settings
  FlyCapture2::VideoMode currVideoMode;
  FlyCapture2::FrameRate currFrameRate;
  FlyCapture2::Format7ImageSettings currFmt7Settings;
  unsigned int currPacketSize;

  // Get current video mode and frame rate
  error = cameras[camId]->GetVideoModeAndFrameRate( &currVideoMode, &currFrameRate );
  if ( error != FlyCapture2::PGRERROR_OK )
    throw runtime_error(error.GetDescription());

  if ( currVideoMode == FlyCapture2::VIDEOMODE_FORMAT7 )
  {
    // Get the current Format 7 settings
    float percentage;
    error = cameras[camId]->GetFormat7Configuration( &currFmt7Settings, &currPacketSize, &percentage );
    if ( error != FlyCapture2::PGRERROR_OK )
      throw runtime_error(error.GetDescription());
  }

  // Get the image settings
  FlyCapture2::Format7ImageSettings newFmt7Settings;
  if(videoMode == 0) newFmt7Settings.mode = FlyCapture2::MODE_0;
  else if(videoMode == 1) newFmt7Settings.mode = FlyCapture2::MODE_1;
  else println("wrong video mode for format7-mode selected.");
  newFmt7Settings.offsetX = offsetX;
  newFmt7Settings.offsetY = offsetY;
  newFmt7Settings.width = width;
  newFmt7Settings.height = height;
  newFmt7Settings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RGB8;

  // Experimentell bestimmte maximale packet-size: für MODE_0: 1600 ==   ~13fps
  // 																													 3200 ==   ~20fps
  //																													 2400 == 15-22fps
  // Experimentell bestimmte maximale packet-size: für MODE_1: 1800 ==   ~15fps
  //																													 2400 == 15-22fps
  // Bei MODE_0: 3400 bzw. MODE_1: 2600 bekommt man folgende Meldung:
  // [VideoServer: ***************************************************************]
  // [VideoServer: Aborting after catching std::exception from runComponent()]
  // [VideoServer: what(): PointGreyServer: can only handle images with size 3*w*h bytes]
  // [VideoServer: ***************************************************************]
  unsigned int newPacketSize = paketSize;				// Mode_0: 7260 (52Hz) / Mode_1: 3628 (30Hz)
  if(paketSize > 1600) log("maybe paket size too big: 1600 recommended.");

  // Stop the camera: should be stopped at the start
  // error = m_pCamera->StopCapture();

  // Set the Format7 settings
  error = cameras[camId]->SetFormat7Configuration( &newFmt7Settings, newPacketSize );
  if ( error != FlyCapture2::PGRERROR_OK )
  {
    // Set the camera back to original state, if we got an error.
    println("could not set the format7 configuration.\n");
    cameras[camId]->SetVideoModeAndFrameRate( currVideoMode, currFrameRate );
  }

  // Get and print Format7 info
  // 		FlyCapture2::Format7Info pInfo;
  // 		bool pSupported;
  // 		error = cameras[camId]->GetFormat7Info(&pInfo, &pSupported);
  // 		if(error != FlyCapture2::PGRERROR_OK) throw runtime_error(error.GetDescription());
  // 		printf("GetFormat7Info():\n");
  // 		if(pInfo.mode == FlyCapture2::MODE_0) printf("    pInfo.mode = FlyCapture2::MODE_0\n");
  // 		if(pInfo.mode == FlyCapture2::MODE_1) printf("    pInfo.mode = FlyCapture2::MODE_1\n");
  // 		printf("    GOT: Packetsize: %u	percentage: %4.3f\n", pInfo.packetSize, pInfo.percentage);
  // 		printf("    GOT: max. Packetsize: %u	pixelFormatBitField: %u\n", pInfo.maxPacketSize, pInfo.pixelFormatBitField);
}


/**
 * @brief Select the video mode in respect to the image width.\n
 * Choose only between 640x480 or 1280x960. All other resolutions are \n
 * pruned from this two resolutions (and are therefore senseless). \n
 * The heigt will be set automatically to get a 4:3 image. We choose the highest \n
 * colour resolution.
 * NOTE: we use only width to select the video mode and assume height to be 3/4 of width \n
 * If we have several colour formats to choose from, we always select colour \n
 * over greyscale and select the one with hightest colour resolution, e.g. RGB \n
 * over YUV422.
 * @param width Image width
 * @param heigt Image height
 */
FlyCapture2::VideoMode PointGreyServer::selectVideoMode(int &_width, int &_height)
{
  if(_width == 640)
  {
    _height = 480;
    if(fps > 15) log("Maybe too high framerate requested: <= 15fps recommended.");
    return FlyCapture2::VIDEOMODE_640x480RGB;
  }
  if(_width == 1280)
  {
    _height = 960;
    if(fps > 3) log("Maybe too high framerate requested: 3,75fps (3) recommended, <7,5 mandatory.");
    if(fps > 7)
    {
      log("Changed requested framerate to 7,5fps.");
      fps = 7;
    }
    return FlyCapture2::VIDEOMODE_1280x960YUV422;
  }
  if(_width <= 640)
  {
    // the default value
    log("unknown video mode: video mode set to: 640x480RGB");
    _width = 640;
    _height = 480;
    return FlyCapture2::VIDEOMODE_640x480RGB;
  }
  if(_width > 640)
  {
    // the default value
    log("unknown video mode: video mode set to: 1280x960YUV422");
    _width = 640;
    _height = 480;
    if(fps > 3) log("Maybe too high framerate requested: 3,75fps (3) recommended, <7,5 mandatory.");
    if(fps > 7)
    {
      log("Changed requested framerate to 7,5fps.");
      fps = 7;
    }
    return FlyCapture2::VIDEOMODE_1280x960YUV422;
  }

  return FlyCapture2::VIDEOMODE_640x480RGB; // must return something to make the compiler happy
}


/**
 * @brief Select the frame rate. Default value is 30fps.
 * @param _fps Frames per second
 */
FlyCapture2::FrameRate PointGreyServer::selectFrameRate(int &_fps)
{
  switch(_fps)
  {
    case 1:
      return FlyCapture2::FRAMERATE_1_875;
      break;
    case 3:
      return FlyCapture2::FRAMERATE_3_75;
      break;
    case 7:
      return FlyCapture2::FRAMERATE_7_5;
      break;
    case 15:
      return FlyCapture2::FRAMERATE_15;
      break;
    case 30:
      return FlyCapture2::FRAMERATE_30;
      break;
    case 60:
      return FlyCapture2::FRAMERATE_60;
      break;
    case 120:
      return FlyCapture2::FRAMERATE_120;
      break;
    default:
      log("unknown framerate: set to default value: 15fps\n");
      return FlyCapture2::FRAMERATE_15;
      break;
  }
}


/**
 * @brief Initialise the PointGreyServer.
 */
void PointGreyServer::init() throw(runtime_error)
{
  unsigned int numAvailableCameras;
  FlyCapture2::Error error;

  error = busMgr.GetNumOfCameras(&numAvailableCameras);
  if(error != FlyCapture2::PGRERROR_OK)
    throw runtime_error(error.GetDescription());

  if(numAvailableCameras < (size_t)getNumCameras())
    throw runtime_error("PointGreyServer: insufficient number of cameras detected");
  cameras = new FlyCapture2::Camera*[(size_t)getNumCameras()];
  retrievedImages.resize((size_t)getNumCameras());
  grabTimes.resize((size_t)getNumCameras());

  // Connect to all detected cameras and attempt to set them to
  // a common video mode and frame rate
  for(size_t i = 0; i < (size_t)getNumCameras(); i++)
  {
    cameras[i] = new FlyCapture2::Camera();

    FlyCapture2::PGRGuid guid;
    error = busMgr.GetCameraFromIndex(i, &guid);
    if(error != FlyCapture2::PGRERROR_OK)
      throw runtime_error(error.GetDescription());

    // Connect to a camera
    error = cameras[i]->Connect(&guid);
    if(error != FlyCapture2::PGRERROR_OK)
      throw runtime_error(error.GetDescription());

    // Get the camera information and log it
    // FlyCapture2::CameraInfo camInfo;
    // error = cameras[i]->GetCameraInfo(&camInfo);
    // if(error != FlyCapture2::PGRERROR_OK)
    //   throw runtime_error(error.GetDescription());
    // LogCameraInfo(&camInfo);

    log("setting video mode %d x %d", width, height);

    FlyCapture2::VideoMode mode = selectVideoMode(width, height);
    FlyCapture2::FrameRate rate = selectFrameRate(fps);
    if(useVideoMode7)
      SetVideoMode7(i);
    else
    {
      error = cameras[i]->SetVideoModeAndFrameRate(mode, rate);
      if(error != FlyCapture2::PGRERROR_OK)
        throw runtime_error(error.GetDescription());
    }
  }

  // Camera configuration
  // LogCameraConfig();

  // start syncronized capturing of images from all cameras
  error = FlyCapture2::Camera::StartSyncCapture(getNumCameras(), (const FlyCapture2::Camera**)cameras);
  if(error != FlyCapture2::PGRERROR_OK)
    throw runtime_error(error.GetDescription());

  // 	// start unsyncronized capturing of images
  // 	for(size_t i=0; i<(size_t)getNumCameras(); i++)
  // 	{
  // 		error = cameras[i]->StartCapture();
  // 		if(error != FlyCapture2::PGRERROR_OK)
  // 			throw runtime_error(error.GetDescription());
  // 	}
}


/**
 * @brief Configure the PointGreyServer component for cast.
 * @param width Configuration
 */
void PointGreyServer::configure(const map<string,string> & _config) throw(runtime_error)
{
  map<string,string>::const_iterator it;

  // first let the base class configure itself
  VideoServer::configure(_config);

  if((it = _config.find("--imgsize")) != _config.end())
  {
    istringstream str(it->second);
    str >> width >> height;
  }

  if((it = _config.find("--framerate_fps")) != _config.end())
  {
    istringstream str(it->second);
    str >> fps;
  }

  // set the properties before the initialisation
  if((it = _config.find("--format7")) != _config.end())								/// TODO Check, if we have 5 properties!
  {
    useVideoMode7 = true;
    int prop[6];
    istringstream str(it->second);
    str >> prop[0] >> prop[1] >> prop[2] >> prop[3] >> prop[4] >> prop[5];
    SetFormat7Properties(prop[0], prop[1], prop[2], prop[3], prop[4], prop[5]);
  }

  // do some initialisation based on configured items
  init();

  // set same properties AFTER initialisation
  if((it = _config.find("--setAutPropAdj")) != _config.end())					/// TODO Check, if we have 6 properties!
  {
    istringstream str(it->second);
    str >> propActive[1] >> propActive[2] >> propActive[3] >> propActive[5] >> propActive[12] >> propActive[13];
    SetAutomaticPropertyAdjustment();
  }
}


/**
 * @brief Grab frames internal
 */
void PointGreyServer::grabFramesInternal()
{
  for(size_t i = 0; i < (size_t)getNumCameras(); i++)
  {
    FlyCapture2::Error error;
    error = cameras[i]->RetrieveBuffer(&retrievedImages[i]);
    if(error != FlyCapture2::PGRERROR_OK)
      throw runtime_error(error.GetDescription());
  }

  cdl::CASTTime time = getCASTTime();
  for(size_t i = 0; i < grabTimes.size(); i++)
    grabTimes[i] = time;
  framerateMillis.insert();
}


/**
 * @brief Grab frames. Before grabing new frames: Set properties of the \n
 * cameras to equal values, if activated.
 */
void PointGreyServer::grabFrames()
{
  // If automatic adjustment of properties is active
  if(setAutomaticPropertyAdjustment) CopyAllPropertyValues();
  grabFramesInternal();
}

/**
 * @brief Retrieve frames internal. \n
 * Convert color format to RGB and resize image, if necessary.
 * @param camIdx Camera id
 * @param width Image width
 * @param height Image height
 * @param frame Video frame
 */
void PointGreyServer::retrieveFrameInternal(int camIdx, int width, int height, Video::Image &frame)
{
  // we convert the image to RGB8, if format is different
  FlyCapture2::Image image;
  if(retrievedImages[camIdx].GetPixelFormat() != FlyCapture2::PIXEL_FORMAT_RGB8)
    retrievedImages[camIdx].Convert(FlyCapture2::PIXEL_FORMAT_RGB8, &image);
  else
    image = retrievedImages[camIdx];

  // if image size is greater than actual image size, change camera capturing mode to higher resolution.
  if (width > this->width)
  {
    log("Image with higher resolution requested: %u x %u. Change resolution", width, height);
    FlyCapture2::Error error;

    // change image resolution and frame rate
    if(!useVideoMode7)
    {
      for(size_t i=0; i<(size_t)getNumCameras(); i++)
      {
        int w = width;
        int h = height;
        // stop capturing of images from all cameras
        error = cameras[i]->StopCapture();
        if(error != FlyCapture2::PGRERROR_OK)
          throw runtime_error(error.GetDescription());

        log("setting video mode %d x %d", width, height);

        FlyCapture2::VideoMode mode = selectVideoMode(w, h);
        FlyCapture2::FrameRate rate = selectFrameRate(fps);
        error = cameras[i]->SetVideoModeAndFrameRate(mode, rate);
        if(error != FlyCapture2::PGRERROR_OK)
          throw runtime_error(error.GetDescription());
      }
      this->width = width;
      this->height = height;
    }
    else log("Image with higher resolution requested: not possible in Format7 mode.");

    // start syncronized capturing of images from all cameras with new video mode
    error = FlyCapture2::Camera::StartSyncCapture(getNumCameras(), (const FlyCapture2::Camera**)cameras);
    if(error != FlyCapture2::PGRERROR_OK)
      throw runtime_error(error.GetDescription());

    // set new width and height
    this->width = width;
    this->height = height;
  }

  frame.time = grabTimes[camIdx];
  frame.camId = camIds[camIdx];
  frame.camPars = camPars[camIdx];

  // no size given, use native size
  if((width == 0 || height == 0) || (width == this->width && height == this->height))
  {
    copyImage(image, frame);
    // adjust to native size
    // (note that calibration image size need not be the same as currently set
    // native capture size)
    changeImageSize(frame.camPars, this->width, this->height);
  }
  else
  {
    char id[16];
    sprintf(id, "frame%d", camIdx);

    // use image cache to avoid allocate/deallocating all the time
    IplImage *tmp_resized = m_imageCache.getImage(id, width, height, IPL_DEPTH_8U, 3);

#if USE_WRAPIMAGE_HACK
    IplImage *tmp = wrapFlyCaptureImage(image);
    cvResize(tmp, tmp_resized);
    convertImageFromIpl(tmp_resized, frame);
    Video::releaseWrappedImage(&tmp);
    // adjust to scaled image size
    changeImageSize(frame.camPars, width, height);
#else
    debug("Resize image. Bad solution with copying to iplImage.");
    /// TODO TODO TODO TODO We should find another solution!
    // NOTE: this is very wasteful! And should only be a temporary solution!
    // convert to iplImage, resize and convert back to Video::Image
    copyImage(image, frame);
    IplImage *tmp = 0;
    convertImageToIpl(frame, &tmp);
    cvResize(tmp, tmp_resized);
    convertImageFromIpl(tmp_resized, frame);
    cvReleaseImage(&tmp);
    // adjust to scaled image size
    changeImageSize(frame.camPars, width, height);
#endif
  }
}

/**
 * @brief Retrieve frames internal
 * @param camIds Camera ids
 * @param width Image width
 * @param height Image height
 * @param frame Video frame
 */
void PointGreyServer::retrieveFrames(const std::vector<int> &camIds, int width, int height, std::vector<Video::Image> &frames)
{
  frames.resize(camIds.size());
  for(size_t j = 0; j < camIds.size(); j++)
  {
    size_t i = getCamIndex(camIds[j]);
    retrieveFrameInternal(i, width, height, frames[j]);
  }
}

/**
 * @brief Retrieve frames.
 * @param width Image width
 * @param height Image height
 * @param frame Video frame
 */
void PointGreyServer::retrieveFrames(int width, int height, std::vector<Video::Image> &frames)
{
  frames.resize((size_t)getNumCameras());
  for(size_t i = 0; i < (size_t)getNumCameras(); i++)
    retrieveFrameInternal(i, width, height, frames[i]);
}

/**
 * @brief Retrieve frame
 * @param camId Camera id
 * @param width Image width
 * @param height Image height
 * @param frame Video frame
 */
void PointGreyServer::retrieveFrame(int camId, int width, int height, Video::Image &frame)
{
  size_t i = getCamIndex(camIds[camId]);
  retrieveFrameInternal(i, width, height, frame);
}

/**
 * @brief Retrieve frame
 * @param camId Camera id
 * @param width Image width
 * @param height Image height
 * @param frame Video frame
 */
void PointGreyServer::retrieveHRFrames(std::vector<Video::Image> &frames)
{
  printf("PointGreyServer::retrieveHRFrames: Not yet implemented!\n");
}

/**
 * @brief Get image size.
 * @param width Image width
 * @param height Image height
 */
void PointGreyServer::getImageSize(int &width, int &height)
{
  width = this->width;
  height = this->height;
}

/**
 * @brief Get frame rate in milliseconds.
 * @return Returns the frame rate in milliseconds.
 */
int PointGreyServer::getFramerateMilliSeconds()
{
  return framerateMillis.getRate();
}

/**
 * @brief Change the properties in the PointGreyServer for the Format7 mode.
 * @param width Image width
 * @param height Image height
 * @param offsetX Offset in x- direction for Format7 mode.
 * @param offsetY Offset in y- direction for Format7 mode.
 * @param mode Image grabbing mode for the Format7 mode.
 * @param fps Requested framerate [1/s]
 */
void PointGreyServer::changeFormat7Properties(int width, int height, int offsetX, int offsetY, int mode, int paketSize)
{
  if (!useVideoMode7) return;

  lockComponent();
  SetFormat7Properties(width, height, offsetX, offsetY, mode, paketSize);
  for(int i=0; i<getNumCameras(); i++)
    SetVideoMode7(i);
  unlockComponent();
}


/**
 * @brief Copy image. \n
 * note: If img is of appropriate size already, no memory allocation takes place.
 * @param flyImg FlyCapture2 image
 * @param img Video image
 */
void PointGreyServer::copyImage(const FlyCapture2::Image &flyImg, Video::Image &img) throw(runtime_error)
{
  if(flyImg.GetPixelFormat() != FlyCapture2::PIXEL_FORMAT_RGB8)
    throw runtime_error("PointGreyServer: can only handle RGB8 image format");
  if(flyImg.GetStride() != flyImg.GetCols()*3)
    throw runtime_error("PointGreyServer: can only handle images with no padding");
  if(flyImg.GetDataSize() != flyImg.GetCols()*flyImg.GetRows()*3)
    throw runtime_error("PointGreyServer: can only handle images with size 3*w*h bytes");

  img.width = flyImg.GetCols();
  img.height = flyImg.GetRows();
  img.data.resize(img.width*img.height*3);
  memcpy(&img.data[0], flyImg.GetData(), img.data.size());
}

#if USE_WRAPIMAGE_HACK
/**
 * @brief Create an IplImage that uses the data from a FlyCapture2::Image. \n
 * The image data is shared between FlyCapture2::image and IplImage. \n
 * WARNING: DO NOT cvReleaseImage() the obtained VideoImage.
 *    Use releaseWrappedImage() instead. \n
 * WARNING: The Video::Image MUST NOT BE DESTROYED while using the
 *    IplImage created with this function.
 * WARNING: At the moment it is not clear if the hack works with
 *    images whose width is not aligned to 4 bytes.
 * WARNING: THIS IS A SEVERE HACK: USE AT YOUR OWN RISK.
 */
IplImage* wrapFlyCaptureImage(const FlyCapture2::Image &flyImg)
{
  if(flyImg.GetPixelFormat() != FlyCapture2::PIXEL_FORMAT_RGB8)
    throw runtime_error("PointGreyServer: can only handle RGB8 image format");
  if(flyImg.GetStride() != flyImg.GetCols()*3)
    throw runtime_error("PointGreyServer: can only handle images with no padding");
  if(flyImg.GetDataSize() != flyImg.GetCols()*flyImg.GetRows()*3)
    throw runtime_error("PointGreyServer: can only handle images with size 3*w*h bytes");

  // HACK: Data shared between IplImage and FlyCapture2::Image
  IplImage* pImg = cvCreateImageHeader(cvSize(flyImg.GetCols(), flyImg.GetRows()), IPL_DEPTH_8U, 3);
  pImg->imageData = (char*) flyImg.GetData();
  pImg->imageDataOrigin = pImg->imageData;
  pImg->widthStep = flyImg.GetCols() * 3;
  pImg->imageSize = pImg->widthStep * flyImg.GetRows();
  return pImg;
}
#endif

}

