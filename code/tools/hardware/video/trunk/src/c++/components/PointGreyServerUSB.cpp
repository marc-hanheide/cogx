/**
 * @file PointGreyServerUSB.cpp
 * @author Andreas Richtsfeld, Michael Zillich
 * @date Februrary 2011
 * @version 0.1
 * @brief Video server for the USB PointGray stereo cameras.
 */

#include <cmath>
#include <cast/core/CASTUtils.hpp>
#include <VideoUtils.h>
#include "PointGreyServerUSB.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr newComponent() {
    return new cast::PointGreyServerUSB();
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
 * @brief Constructor of class MeanRate.
 */
PointGreyServerUSB::MeanRate::MeanRate(int size) : mean(size)
{
  gettimeofday(&prev, 0);
}

/**
 * @brief Returns the current rate as 1/(current time - last time) or 0 if current time
 * == last time
 * @return Returns the current calculated (frame-) rate.
 */
float PointGreyServerUSB::MeanRate::calculateCurrentRate()
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
PointGreyServerUSB::PointGreyServerUSB()
{
  cameras = 0;
  width = height = 0;
  offsetX = offsetY = 0;
  paketSize = 0;
}


/**
 * @brief Destructor of class PointGreyServer.
 */
PointGreyServerUSB::~PointGreyServerUSB()
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
FlyCapture2::VideoMode PointGreyServerUSB::selectVideoMode(int &_width, int &_height)
{
  if(_width == 640)
  {
    _height = 480;
    if(fps > 15) log("Maybe too high framerate requested: <= 15fps recommended.");
    return FlyCapture2::VIDEOMODE_640x480Y16; //VIDEOMODE_640x480RGB;                                                                 /// TODO TODO TODO 
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
    return FlyCapture2::VIDEOMODE_1280x960Y8; //VIDEOMODE_1280x960YUV422;
  }
  if(_width <= 640)
  {
    // the default value
    log("unknown video mode: video mode set to: 640x480RGB");
    _width = 640;
    _height = 480;
    return FlyCapture2::VIDEOMODE_640x480Y16;
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
FlyCapture2::FrameRate PointGreyServerUSB::selectFrameRate(int &_fps)
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
void PointGreyServerUSB::init() throw(runtime_error)
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

  // Connect to all detected cameras and attempt to set them to a common video mode and frame rate
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
//     FlyCapture2::CameraInfo camInfo;
//     error = cameras[i]->GetCameraInfo(&camInfo);
//     if(error != FlyCapture2::PGRERROR_OK)
//       throw runtime_error(error.GetDescription());
//     LogCameraInfo(&camInfo);

    log("setting video mode %d x %d with %d fps", width, height, fps);
    FlyCapture2::VideoMode mode = selectVideoMode(width, height);
    FlyCapture2::FrameRate rate = selectFrameRate(fps);

    error = cameras[i]->SetVideoModeAndFrameRate(mode, rate);
    if(error != FlyCapture2::PGRERROR_OK)
      throw runtime_error(error.GetDescription());
  }

  // start unsyncronized capturing of images
  for(size_t i=0; i<(size_t)getNumCameras(); i++)
  {
    error = cameras[i]->StartCapture();
    if(error != FlyCapture2::PGRERROR_OK)
      throw runtime_error(error.GetDescription());
  }
}


/**
 * @brief Configure the PointGreyServerUSB component for cast.
 * @param width Configuration
 */
void PointGreyServerUSB::configure(const map<string,string> & _config) throw(runtime_error)
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
}


/**
 * @brief Grab frames internal
 */
void PointGreyServerUSB::grabFramesInternal()
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
void PointGreyServerUSB::grabFrames()
{
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
void PointGreyServerUSB::retrieveFrameInternal(int camIdx, int width, int height, Video::Image &frame)
{
printf("retirieveFrameInternal: %u\n", camIdx);
  // we convert the image to RGB8, if format is different
  FlyCapture2::Image image;
  if(retrievedImages[camIdx].GetPixelFormat() == FlyCapture2::PIXEL_FORMAT_MONO8)
    printf(" => is pixel format mono8! \n");
  if(retrievedImages[camIdx].GetPixelFormat() != FlyCapture2::PIXEL_FORMAT_RGB8) 
  {
    retrievedImages[camIdx].Convert(FlyCapture2::PIXEL_FORMAT_RGB8, &image);
//     if(image.GetPixelFormat() != FlyCapture2::PIXEL_FORMAT_RGB8) {                                                     /// TODO Was machen wir hier? Umwandeln in RGB format, oder wie?
//       printf(" => is not pixel format mono16! \n"); 
//     }
  }
  else
    image = retrievedImages[camIdx];

  
  // show image
// printf("retirieveFrameInternal: show image\n");
//   static bool first = true;
//   if (first){
//     printf("retirieveFrameInternal: create window\n");
//     cvNamedWindow("retrieved", CV_WINDOW_AUTOSIZE);
//   }
//   first = false;
// printf("retirieveFrameInternal: show image 1\n");
//   Video::Image s_img;
//   IplImage *ipl_img = 0;
//   copyImage(image, s_img);
// printf("retirieveFrameInternal: show image 2\n");
//   convertImageToIpl(s_img, &ipl_img);
// printf("retirieveFrameInternal: show image 3\n");
//   cvShowImage("retrieved", ipl_img);
// printf("retirieveFrameInternal: show image done\n");
 
  
  // if image size is greater than actual image size, change camera capturing mode to higher resolution.
  if (width > this->width)
  {
    log("Image with higher resolution requested: %u x %u. Change resolution", width, height);
    FlyCapture2::Error error;


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
      
      error = cameras[i]->StartCapture();
      if(error != FlyCapture2::PGRERROR_OK)
        throw runtime_error(error.GetDescription());
    }

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
    // (note that calibration image size need not be the same as currently set native capture size)
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
void PointGreyServerUSB::retrieveFrames(const std::vector<int> &camIds, int width, int height, std::vector<Video::Image> &frames)
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
void PointGreyServerUSB::retrieveFrames(int width, int height, std::vector<Video::Image> &frames)
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
void PointGreyServerUSB::retrieveFrame(int camId, int width, int height, Video::Image &frame)
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
void PointGreyServerUSB::retrieveHRFrames(std::vector<Video::Image> &frames)
{
  printf("PointGreyServerUSB::retrieveHRFrames: Not yet implemented!\n");
}

/**
 * @brief Get image size.
 * @param width Image width
 * @param height Image height
 */
void PointGreyServerUSB::getImageSize(int &width, int &height)
{
  width = this->width;
  height = this->height;
}

/**
 * @brief Get frame rate in milliseconds.
 * @return Returns the frame rate in milliseconds.
 */
int PointGreyServerUSB::getFramerateMilliSeconds()
{
  return framerateMillis.getRate();
}


/**
 * @brief Copy image. \n
 * note: If img is of appropriate size already, no memory allocation takes place.
 * @param flyImg FlyCapture2 image
 * @param img Video image
 */
void PointGreyServerUSB::copyImage(const FlyCapture2::Image &flyImg, Video::Image &img) throw(runtime_error)
{
  if(flyImg.GetPixelFormat() != FlyCapture2::PIXEL_FORMAT_RGB8)
    throw runtime_error("PointGreyServerUSB: can only handle RGB8 image format");
  if(flyImg.GetStride() != flyImg.GetCols()*3)
    throw runtime_error("PointGreyServerUSB: can only handle images with no padding");
  if(flyImg.GetDataSize() != flyImg.GetCols()*flyImg.GetRows()*3)
    throw runtime_error("PointGreyServerUSB: can only handle images with size 3*w*h bytes");

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
    throw runtime_error("PointGreyServerUSB: can only handle RGB8 image format");
  if(flyImg.GetStride() != flyImg.GetCols()*3)
    throw runtime_error("PointGreyServerUSB: can only handle images with no padding");
  if(flyImg.GetDataSize() != flyImg.GetCols()*flyImg.GetRows()*3)
    throw runtime_error("PointGreyServerUSB: can only handle images with size 3*w*h bytes");

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

