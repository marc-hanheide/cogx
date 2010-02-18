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
  width = height = 0;
  cameras = 0;
	setSamePropertiesActiveFlag = false;
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
		if (i == 0) // set the first camera properties auto.
			for(unsigned j=0; j< sizeof(propTypeNames)/30; j++)
				if(propActive[j] == 1)
					SetPropertyManual(i, (FlyCapture2::PropertyType) j, false);
		if (i > 0) // set all other camera properties manual
			for(unsigned j=0; j< sizeof(propTypeNames)/30; j++)
				if(propActive[j] == 1)
					SetPropertyManual(i, (FlyCapture2::PropertyType) j, true);
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
	if (counter > 15)										/// TODO add framerate
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
void PointGreyServer::SetSamePropertiesActive()
{
printf("SetSamePropertiesActive!\n");
	setSamePropertiesActiveFlag = true;

	SetAllPropertiesManual();
	CopyAllPropertyValues();
}


/**
 * @brief Select the video mode in respect to the image width.\n
 * The heigt will be set automatically to get a 4:3 image. We choose the highest \n
 * colour resolution.
 * @param width Image width
 * @param heigt Image height
 */
FlyCapture2::VideoMode PointGreyServer::selectVideoMode(int &_width, int &_height)
{
  // note: we use only width to select the video mode and assume height to be 3/4 of width
  // If we have several colour formats to choose from, we always select colour
  // over greyscale and select the one with hightest colour resolution, e.g. RGB
  // over YUV422

//   if(_width == 0)			// TODO ADD CUSTOM VIDEO MODE7 ?
//   {
// 		_width = 1280;
//     _height = 960;
//     return FlyCapture2::VIDEOMODE_FORMAT7;
//   }
  if(_width == 160)
  {
    _height = 120;
    return FlyCapture2::VIDEOMODE_160x120YUV444;
  }
  if(_width == 320)
  {
    _height = 240;
    return FlyCapture2::VIDEOMODE_320x240YUV422;
  }
  if(_width == 640)
  {
    _height = 480;
    return FlyCapture2::VIDEOMODE_640x480RGB;
  }
  if(_width == 800)
  {
    _height = 600;
    return FlyCapture2::VIDEOMODE_800x600RGB;
  }
  if(_width == 1024)
  {
    _height = 768;
    return FlyCapture2::VIDEOMODE_1024x768RGB;
  }
  if(_width == 1280)
  {
    _height = 960;
    return FlyCapture2::VIDEOMODE_1280x960RGB;
  }
  else
  {
    // the default value
		log("unknown video mode: video mode set to default value: 640x480RGB");
    _width = 640;
    _height = 480;
    return FlyCapture2::VIDEOMODE_640x480RGB;
  }
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
		  log("unknown framerate: set to default value: 30fps\n");
			return FlyCapture2::FRAMERATE_30;
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
    FlyCapture2::CameraInfo camInfo;
    error = cameras[i]->GetCameraInfo(&camInfo);
    if(error != FlyCapture2::PGRERROR_OK)
      throw runtime_error(error.GetDescription());
    LogCameraInfo(&camInfo);

		// set video mode and frame rate
    FlyCapture2::VideoMode mode = selectVideoMode(width, height);
    FlyCapture2::FrameRate rate = selectFrameRate(fps);
    error = cameras[i]->SetVideoModeAndFrameRate(mode, rate);
    if(error != FlyCapture2::PGRERROR_OK)
      throw runtime_error(error.GetDescription());
  }

	// start syncronized capturing of images from all cameras
  error = FlyCapture2::Camera::StartSyncCapture(getNumCameras(), (const FlyCapture2::Camera**)cameras);
  if(error != FlyCapture2::PGRERROR_OK)
    throw runtime_error(error.GetDescription());
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

  // do some initialisation based on configured items
  init();

  // set same properties AFTER initialisation
  if((it = _config.find("--setSameProp")) != _config.end())
  {
    SetSamePropertiesActive();
    istringstream str(it->second);
    str >> propActive[1] >> propActive[2] >> propActive[3] >> propActive[5] >> propActive[12] >> propActive[13];
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
 * @brief Grab frames. Before grabing frames set properties of the cameras to equal values.
 */
void PointGreyServer::grabFrames()
{
	// If automatic adjustment of properties is active
	if(setSamePropertiesActiveFlag) CopyAllPropertyValues();

  grabFramesInternal();
}


/**
 * @brief Retrieve frames internal
 * @param camIdx Camera id
 * @param width Image width
 * @param height Image height
 * @param frame Video frame
 */
void PointGreyServer::retrieveFrameInternal(int camIdx, int width, int height, Video::Image &frame)
{
  // no size given, use native size
  if((width == 0 || height == 0) || (width == this->width && height == this->height))
  {
    copyImage(retrievedImages[camIdx], frame);
  }
  else
  {
		/// TODO TODO TODO TODO We should find another solution!
    // NOTE: this is very wasteful! And should only be a temporary solution!
		// convert to iplImage, resize and convert back to Video::Image
    copyImage(retrievedImages[camIdx], frame);
    IplImage *tmp = 0; 
    IplImage *tmp_resized = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    convertImageToIpl(frame, &tmp);
    cvResize(tmp, tmp_resized);
    convertImageFromIpl(tmp_resized, frame);
    cvReleaseImage(&tmp_resized);
    cvReleaseImage(&tmp);
  }

  frame.time = grabTimes[camIdx];
  frame.camId = camIds[camIdx];
  frame.camPars = camPars[camIdx];
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
 * @brief Copy image. \n
 * note: If img is of appropriate size already, no memory allocation takes
 * place.
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

}

