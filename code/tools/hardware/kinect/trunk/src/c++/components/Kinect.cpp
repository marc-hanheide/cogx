/**
 * @file Kinect.cpp
 * @author Richtsfeld
 * @date March 2011
 * @version 0.1
 * @brief Wraper for the OpenNI Driver for the Kinect sensor for use inside of cast-framework
 */


#include "Kinect.h"
#include <openni/XnCodecIDs.h>

#include <fstream>
#include <climits>

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	printf("New User %d\n", nId);
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	printf("Lost user %d\n", nId);
}

#define LOCK_KINECT

namespace Kinect
{


/**
 * @brief Constructor of Class Kinect
 */
Kinect::Kinect(const char *kinect_xml_file)
{
  ni_pause = false;
  
  if(!Init(kinect_xml_file))
  {
    printf("Kinect: Error: Initialisation not successful! Abort!\n");
    exit(0);
  }
}

#ifdef KINECT_CAST_LOGGING
Kinect::Kinect(cast::CASTComponent* pComponent, const char *kinect_xml_file)
  : CCastLoggerMixin(pComponent)
{
  ni_pause = false;
  
  if(!Init(kinect_xml_file))
  {
    printf("Kinect: Error: Initialisation not successful! Abort!\n");
    exit(0);
  }
  m_frameMilliseconds = 50; // limit to 20 fps
  m_grabTimer.restart();
}
#endif

/**
 * @brief Destructor of Class Kinect
 */
Kinect::~Kinect()
{
  StopCapture();
}

/**
 * @brief Initialisation of the device.
 * @param kinect_xml_path
 * @return Return true if initialization was successful.
 */
bool Kinect::Init(const char *kinect_xml_file)
{
  XnStatus rc = XN_STATUS_OK;
  
  // Initialisation from xml file
  EnumerationErrors errors;
  rc = kinect::openDeviceFromXml(kinect_xml_file, errors);
  if (rc != XN_STATUS_OK)
  {
    char buf[255];
    errors.ToString(buf,255);
    printf("Errors: %s\n", buf);
    printf("Kinect::Init: Error: Initialisation from xml-file failed (%s). Check filename and connection to Kinect.\n", kinect_xml_file);
    return false;
  }

  // Input format should be 6 ???
  rc = kinect::getImageGenerator()->SetIntProperty("InputFormat", 6);
  if(rc != XN_STATUS_OK)
    printf("Kinect::Init: Error: Changing input format failed.\n");

  // set pixel format to grayscale (bayer image)
  rc = kinect::getImageGenerator()->SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT);
  if(rc != XN_STATUS_OK) 
    printf("Kinect::Init: Error: Changing pixel format to grayscale failed.\n");

  // Input format should be 2 for software registration
  if (kinect::getDepthGenerator()->SetIntProperty("RegistrationType", 2) != XN_STATUS_OK)
  if(rc != XN_STATUS_OK)
    printf("Kinect::Init: Error: Changing registration type failed.\n");
    
  // Change registration  
  rc = kinect::getDepthGenerator()->GetAlternativeViewPointCap().ResetViewPoint();
  if (rc != XN_STATUS_OK)
    printf("Kinect::Init: Error: Switching off depth stream registration failed.\n");

  rc = kinect::getDepthGenerator()->GetAlternativeViewPointCap().SetViewPoint( *kinect::getImageGenerator());
  if (rc != XN_STATUS_OK)
    printf("Kinect::Init: Error: Switching on depth stream registration failed.\n");
  
  rc = kinect::getDepthGenerator()->GetIntProperty ("ShadowValue", shadow_value);
  if (rc != XN_STATUS_OK)
    printf("Kinect::Init: Error: Could not read shadow value.\n");
  
  rc = kinect::getDepthGenerator()->GetIntProperty ("NoSampleValue", no_sample_value);
  if (rc != XN_STATUS_OK)
    printf("Kinect::Init: Error: Could not read \"no sample\" value.\n");
  
  rc = kinect::getDepthGenerator()->GetRealProperty("ZPPS", pixel_size);
  if (rc != XN_STATUS_OK)
    printf("Kinect::Init: Error: Geting pixel size failed.\n");

  rc = kinect::getDepthGenerator()->GetIntProperty("ZPD", depth_focal_length_SXGA);
  if (rc != XN_STATUS_OK)
    printf("Kinect::Init: Error: Geting depth focal length failed.\n");

  UserGenerator* userGenerator = kinect::getUserGenerator();

  if (userGenerator!=NULL) {
	  XnCallbackHandle hUserCallbacks;
	  printf("we have a user generator to use\n");
	  userGenerator->RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	 // userGenerator->GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
  }

  rgbWidth = 640;	/// TODO Get width and height from file!
  rgbHeight = 480;
  depWidth = 640;
  depHeight = 480;

//   XnUInt64 width;
//   kinect::getImageGenerator()->GetIntProperty("xRes", width);
// printf("rgbWidth: %u\n", (int) width);
  
  centerX = (depWidth >> 1) - 0.5f;
  centerY = (depHeight >> 1) - 0.5f;

  // PCL: This magic value is taken from a calibration routine, unless calibrated params are not supported we rely on thsi value!
  const float rgb_focal_length_SXGA_ = 1050;
//   depth_focal_length_SXGA_ = (float)depth_focal_length_SXGA / pixel_size;
  depth_focal_length_SXGA_ = rgb_focal_length_SXGA_;
  
  float output_x_resolution = rgbWidth;
  depthScale = output_x_resolution / (float)XN_SXGA_X_RES; 	// XN_SXGA_X_RES = 1280;
  depthFocalLength = depth_focal_length_SXGA_ * depthScale;
  constant = 0.001 / depthFocalLength; // 0.001 (mm => m)

  return(rc == XN_STATUS_OK);
}


/**
 * @brief Start capturing from kinect sensor.
 * @param delay Delay in miliseconds.
 */
void Kinect::StartCapture(int delay)
{
  if (ni_pause)
    printf("Kinect::StartCapture: Warning: Cannot record when paused!\n");
  else
    kinect::captureStart(delay);
}

/**
 * @brief Stop capturing from kinect sensor.
 */
void Kinect::StopCapture()
{
  if (ni_pause)
    printf("Kinect::StopCapture: Warning: Cannot stop when paused!\n");
  else
    kinect::captureStop(0);
}

void Kinect::pullData()
{
  if (m_frameMilliseconds <= 0) {
#ifdef LOCK_KINECT
    IceUtil::RWRecMutex::WLock lock(m_kinectMutex);
#endif
    kinect::readFrame();
    return;
  }

  if (m_grabTimer.elapsed() < m_frameMilliseconds) {
    return;
  }
  m_grabTimer.restart();

#ifdef LOCK_KINECT
  IceUtil::RWRecMutex::WLock lock(m_kinectMutex);
#endif
  kinect::readFrame();
}


/**
 * @brief Convert the meta data map to an openCV ipl-image.
 * @param pImageMD Meta data map
 * @param iplImg Destination ipl-image
 */
void Kinect::MapMetaData2IplImage(const MapMetaData* pImageMD, IplImage **iplImg)
{
  if(*iplImg != 0)
    if((*iplImg)->width != pImageMD->FullXRes() || (*iplImg)->height != pImageMD->FullYRes())
      cvReleaseImage(iplImg);
  if(*iplImg == 0)
    *iplImg = cvCreateImage(cvSize(pImageMD->FullXRes(), pImageMD->FullYRes()), IPL_DEPTH_8U, pImageMD->BytesPerPixel());
  assert(*iplImg != 0);
  
  for(unsigned co=0; co<pImageMD->DataSize(); co++)
    (*iplImg)->imageData[co] = pImageMD->Data()[co];
}

/**
 * @brief Convert the meta data depth map to an openCV ipl-image and normalize it to 16 bit.
 * @param pDepthMD Meta data depth map
 * @param iplImg Destination ipl-image
 */
void Kinect::DepthMetaData2IplImage(const DepthMetaData* pDepthMD, IplImage **iplImg)
{
  printf("Kinect::DepthMetaData2IplImage: Not yet implemented!\n");
//   if(*iplImg != 0)
//     if((*iplImg)->width != pDepthMD->FullXRes() || (*iplImg)->height != pDepthMD->FullYRes())
//       cvReleaseImage(iplImg);
//   if(*iplImg == 0)
//     *iplImg = cvCreateImage(cvSize(pDepthMD->FullXRes(), pDepthMD->FullYRes()), IPL_DEPTH_16U, 1);
//   assert(*iplImg != 0);
//   
//   unsigned short *d = (unsigned short *)((*iplImg)->imageData);
//   unsigned short *d2 = (unsigned short *)(pDepthMD->Data());
// 
//   for(unsigned co=0; co<pDepthMD->DataSize()/2; co++)
//   {
//     if(d2[co] < 4096)
//     {
// //       printf("resize: ");
//       double resized = ((double) d2[co]) *65536./4096.;
// //       printf(" %4.3f\n", resized);
//       unsigned shorti = (unsigned short) resized;
// //       printf("shorti %u \n", shorti);
//       d[co] = shorti;
//     }
//     else d[co] = 0;
//   }
}

/** /// TODO überarbeiten: NextFrame verwenden und dann xx2IplImage ausführen!
 * @brief Get Frame from the Kinect sensor
 * @param iplImg Video image as ipl image.
 * @param iplDepthImg Depth image as ipl-image.
 * @return Return true, if both images are captured successful.
 */
bool Kinect::GetFrame(IplImage **iplImg, IplImage **iplDepthImg)
{
  printf("Kinect::GetFrame: Warning: depricated.\n");

//   if(!kinect::isCapturing())
//   {
//     printf("Kinect::NextFrame: Warning: Kinect is not capturing data.\n");
//     return false;
//   }
//
// //   kinect::captureFrame();	/// captureFrame muss nicht aufgerufen werden (nur für Recorder?)
//   kinect::readFrame();        // read the next frame into the data field "CapturingData"					/// TODO wer soll read frame ausführen?
//
//   // get depth meta data
//   const DepthMetaData* pDepthMD = kinect::getDepthMetaData();
//   if (kinect::isDepthOn())
//     DepthMetaData2IplImage(pDepthMD, iplDepthImg);
//   else
//     printf("Kinect::GetFrame: Warning: No depth data available!\n");
//
//   // get image data (color or ir image data)
//   const MapMetaData* pImageMD = NULL;
//   if (kinect::isImageOn())
//   {
//     pImageMD = kinect::getImageMetaData();
//
//     if(*iplImg != 0)
//       if((*iplImg)->width != pImageMD->FullXRes() || (*iplImg)->height != pImageMD->FullYRes())
// 	cvReleaseImage(iplImg);
//     if(*iplImg == 0)
//       *iplImg = cvCreateImage(cvSize(pImageMD->FullXRes(), pImageMD->FullYRes()), IPL_DEPTH_8U, 3);
//     assert(*iplImg != 0);
//
//     IplImage *grayImg = cvCreateImage(cvSize(pImageMD->FullXRes(), pImageMD->FullYRes()), IPL_DEPTH_8U, 1);
//     if (pImageMD != NULL)
//       MapMetaData2IplImage(pImageMD, &grayImg);
//     if(grayImg->nChannels == 1)
//       cvCvtColor(grayImg, *iplImg, CV_BayerGB2RGB);  // convert Bayer image to RGB color image
//     cvReleaseImage(&grayImg);
//     return true;
//   }
//   else if (kinect::isIROn())
//   {
// //       pImageMD = kinect::getIRMetaData();
//     printf("Kinect::GetFrame: Warning: request for IR-image data: not yet implemented!\n");
//     return false;
//   }
//   else
//   {
//     printf("Kinect::GetFrame: Warning: No image data available!\n");
    return false;
//   }
}

const DepthMetaData* Kinect::getNextDepthMD()
{
  if(!kinect::isCapturing()) {
    printf("Kinect::NextFrame: Warning: Kinect is not capturing data.\n");
    return 0;
  }
    
  pullData(); // kinect::readFrame();  // read next frame
  return kinect::getDepthMetaData();
}


std::pair<const DepthMetaData*, const ImageGenerator*> Kinect::getNextFrame()
{
  if(!kinect::isCapturing())
    printf("Kinect::NextFrame: Warning: Kinect is not capturing data.\n");
    
  pullData(); // kinect::readFrame();  // read next frame

  // get depth image
  return std::make_pair(kinect::getDepthMetaData(), kinect::getImageGenerator());
}

/**
 * @brief Get the next frame from the Kinect sensor and copy to openCV matrices.
 * @return Return true, if both images are captured successful.
 */
bool Kinect::NextFrame()
{
  if(!kinect::isCapturing())
  {
    printf("Kinect::NextFrame: Warning: Kinect is not capturing data.\n");
    return false;
  }

  pullData(); // kinect::readFrame();  // read next frame

#ifdef LOCK_KINECT
  //log("NextFrame Write Lock");
  IceUtil::RWRecMutex::WLock lock(m_kinectMutex);
  //log("NextFrame Write Lock OK");
#endif

  // get depth image
  const DepthMetaData* pDepthMD = kinect::getDepthMetaData();
  frameNumber = pDepthMD->FrameID();
  if (kinect::isDepthOn())
  {
    depImage = cv::Mat(depHeight, depWidth, CV_16S);
    short *d  = depImage.ptr<short>(0);
    for(int co=0; co<depHeight*depWidth; co++)
      d[co] = pDepthMD->Data()[co];
  }
  else
    printf("Kinect::NextFrame: Warning: No depth data available!\n");
  
  // get image data (color or ir image data)
  const MapMetaData* pImageMD = NULL;
  if (kinect::isImageOn())
  {
    pImageMD = kinect::getImageMetaData();
    grayImage = cv::Mat(rgbHeight, rgbWidth, CV_8UC1);
    uchar *d  = grayImage.ptr<uchar>();
    for(int co=0; co<rgbHeight*rgbWidth; co++)
      d[co] = pImageMD->Data()[co];

    rgbImage = cv::Mat(rgbHeight, rgbWidth, CV_8UC3);
    cv::cvtColor(grayImage, rgbImage, CV_BayerGB2BGR/* CV_BayerGB2RGB*/, 3);
    
    return true;
  }
  else if (kinect::isIROn())
  {
//       pImageMD = kinect::getIRMetaData();
    printf("Kinect::NextFrame: Warning: request for IR-image data: not yet implemented!\n");
    return false;
  }
  else
  {
    printf("Kinect::NextFrame: Warning: No image data available!\n");
    return false;
  }
  return false;
}

/**
 * @brief Get color image as openCV iplImage from the Kinect sensor
 * @param rgbIplImg Video image as ipl image. A new image will be allocated.
 * @return Return true, if image is captured successful.
 */
bool Kinect::GetColorImage(IplImage **rgbIplImg)
{
#ifdef LOCK_KINECT
  IceUtil::RWRecMutex::RLock lock(m_kinectMutex);
#endif

  (*rgbIplImg) = cvCreateImage(cvSize(rgbWidth, rgbHeight), IPL_DEPTH_8U, 3);
  IplImage tmp = rgbImage;
  cvCopy(&tmp, (*rgbIplImg));
  return true;
}

void Kinect::setDepthColors(const std::vector<long> &rgbColors, const std::vector<double> &positions)
{
  assert(rgbColors.size() > 1 && rgbColors.size() == positions.size());
  m_depthColors = rgbColors;
  m_depthColorRanges = positions;
  m_depthPalette.clear();
  std::sort(m_depthColorRanges.begin(), m_depthColorRanges.end());
  if (m_depthColorRanges.front() != 0 || m_depthColorRanges.back() != 1) {
    log("Kinect: Adjusting a bad list of ranges in setDepthColors");
    // The range has to be adjausted
    double min = m_depthColorRanges.front();
    double range = m_depthColorRanges.back() - min;
    if (range <= 0) {
      m_depthColorRanges.back() = min + 1;
      range = 1;
    }
    std::vector<double>::iterator it;
    for (it = m_depthColorRanges.begin(); it != m_depthColorRanges.end(); ++it) {
      *it = (*it - min) / range;  
    }
  }
}

void Kinect::calculateDepthPalette()
{
  if (m_depthColors.size() < 1 || m_depthColorRanges.size() < 1) {
    const long rgb[] = {
      0x330033, 0xbb00bb, 0x0033ff, 0x00ee55, 0xeebb00,
      0xee0055, 0xee22ee, 0xffffff, 0x00bb33, 0x000000
    };
    const double limits[] = {
      0.0, 0.3, 0.5, 0.6, 0.68,
      0.75, 0.8, 0.85, 0.9, 1.0
    };
    const int N = sizeof(rgb) / sizeof(long);
    setDepthColors(std::vector<long>(rgb, rgb+N), std::vector<double>(limits, limits+N));
  }
  std::vector<long> &rgb = m_depthColors;
  std::vector<double> &limits = m_depthColorRanges;
  const short nvals = 512;
  for (int i = 0; i < nvals; i++) {
    double p = double(i) / nvals;
    int j = 0;
    while (limits[j] < p) {
      j++;
    }
    if (j == 0) j = 1;
    p = (p - limits[j-1]) / (limits[j] - limits[j-1]);
    double r = (1-p) * ((rgb[j-1] & 0xff0000) >> 16) + p * ((rgb[j] & 0xff0000) >> 16);
    double g = (1-p) * ((rgb[j-1] & 0xff00) >> 8) + p * ((rgb[j] & 0xff00) >> 8);
    double b = (1-p) * (rgb[j-1] & 0xff) + p * (rgb[j] & 0xff);
    m_depthPalette.push_back((unsigned char)b);
    m_depthPalette.push_back((unsigned char)g);
    m_depthPalette.push_back((unsigned char)r);
  }
}

/**
 * @brief Get depth image as openCV RGB iplImage from the Kinect sensor
 * @param iplImg Depth image as ipl image. A new image will be allocated. 
 * @return Return true, if image is captured successful.
 */
bool Kinect::GetDepthImageRgb(IplImage **rgbIplImg, bool useHsv)
{
#ifdef LOCK_KINECT
  IceUtil::RWRecMutex::RLock lock(m_kinectMutex);
#endif

  if (useHsv) {
    (*rgbIplImg) = cvCreateImage(cvSize(depWidth, depHeight), IPL_DEPTH_8U, 3);
    short* d = depImage.ptr<short>(0);
    for (int i = 0; i < depImage.rows*depImage.cols; i++)
    {
      if (d[i] == shadow_value || d[i] == no_sample_value) {
        (*rgbIplImg)->imageData[3*i+0] = 0;
        (*rgbIplImg)->imageData[3*i+1] = 0;
        (*rgbIplImg)->imageData[3*i+2] = 0;
      }
      else {
        // depth mask = 0xfff
        unsigned char h = (d[i] >> 4) & 0xff;
        unsigned char v = 0xff - ((d[i] & 0xfff) >> 4);
        unsigned char s = 0xff - ((d[i] >> 2) & 0x7f);
        (*rgbIplImg)->imageData[3*i+0] = h;
        (*rgbIplImg)->imageData[3*i+1] = s;
        (*rgbIplImg)->imageData[3*i+2] = v;
      }
    }
    cvCvtColor(*rgbIplImg, *rgbIplImg, CV_HSV2RGB);
  }
  else {
    if (m_depthPalette.size() < 3) {
      calculateDepthPalette();
    }

    (*rgbIplImg) = cvCreateImage(cvSize(depWidth, depHeight), IPL_DEPTH_8U, 3);
    short* d = depImage.ptr<short>(0);
    long nVals = depImage.rows*depImage.cols;
    for(int i = 0; i < nVals; i++)
    {
      if (d[i] == shadow_value || d[i] == no_sample_value) {
        (*rgbIplImg)->imageData[3*i+0] = 0;
        (*rgbIplImg)->imageData[3*i+1] = 0;
        (*rgbIplImg)->imageData[3*i+2] = 0;
      }
      else {
        short si = (0xfff - (d[i] & 0xfff)) >> 3;
        // TODO: if (m_showPalette)
        //   if (i % depImage.cols < 512 && i / 512 < 128) si = i % depImage.cols;
        (*rgbIplImg)->imageData[3*i+0] = m_depthPalette[si*3+0];
        (*rgbIplImg)->imageData[3*i+1] = m_depthPalette[si*3+1];
        (*rgbIplImg)->imageData[3*i+2] = m_depthPalette[si*3+2];
      }
    }
  }
  return true;
}

/**
 * @brief Get depth image as openCV GS iplImage from the Kinect sensor
 * @param iplImg Depth image as ipl image. A new image will be allocated. 
 * @return Return true, if image is captured successful.
 */
bool Kinect::GetDepthImageGs(IplImage **gsIplImg)
{
#ifdef LOCK_KINECT
  IceUtil::RWRecMutex::RLock lock(m_kinectMutex);
#endif

  (*gsIplImg) = cvCreateImage(cvSize(depWidth, depHeight), IPL_DEPTH_8U, 1);
  short* d = depImage.ptr<short>(0);
  for(int i = 0; i < depImage.rows*depImage.cols; i++)
  {
    unsigned char value = (d[i] >> 3) & 0xff;
    (*gsIplImg)->imageData[i] = value;
  }
  return true;
}

/**
 * @brief Get undistorted and registered images from the kinect sensor.
 * @param rgbImg Video image as openCV matrix.
 * @param depImg Depth image as openCV matrix with 16 bit depth!
 * @return Return true, if both images are captured successful.
 */
bool Kinect::GetImages(cv::Mat &rgbImg, cv::Mat &depImg)
{
#ifdef LOCK_KINECT
  IceUtil::RWRecMutex::RLock lock(m_kinectMutex);
#endif

  rgbImg = rgbImage;
  depImg = depImage;
  return true;
}

cv::Point3f Kinect::WorldToColor(unsigned x, unsigned y)
{
#ifdef LOCK_KINECT
  IceUtil::RWRecMutex::RLock lock(m_kinectMutex);
#endif
  return WorldToColorInternal(x, y);
}

/**
 * @brief Get a world point from the kinect image.
 * @param x x-coordinate of the point on the image plane.
 * @param y y-coordinate of the point on the image plane.
 * @return Returns the 3d world point.
 */
cv::Point3f Kinect::Get3dWorldPoint(unsigned x, unsigned y)
{
#ifdef LOCK_KINECT
  IceUtil::RWRecMutex::RLock lock(m_kinectMutex);
#endif

  short depth = depImage.at<short>(y, x);
  return DepthToWorld(x, y, (int) depth);;
}

/**
 * @brief Get a world point cloud from the kinect image.
 * @param cloud 3D point cloud.
 * @param colCloud Cloud with the color values for each point in the cloud.
 * @param scale Get scaled (reduced) point cloud (Everey 2,3,4 ... point of the full cloud)
 */
void Kinect::Get3dWorldPointCloud(cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud, int scale)
{
#ifdef LOCK_KINECT
  IceUtil::RWRecMutex::RLock lock(m_kinectMutex);
#endif

  cloud = cv::Mat_<cv::Point3f>(rgbHeight/scale, rgbWidth/scale);
  colCloud = cv::Mat_<cv::Point3f>(rgbHeight/scale, rgbWidth/scale);
  int rgb2depthRatio = rgbWidth/depWidth;                 /// TODO Bei 1280 Auflösung gibt es eine Verzerrung in z-Richtung?
  for(int row = 0; row<depHeight; row+=scale)
  {
    for(int col = 0; col<depWidth; col+=scale)
    {
      int col4tel = col/scale;
      int row4tel = row/scale;
      short depth = depImage.at<short>(row, col);
      if(depth != shadow_value && depth != no_sample_value)
      {
        cloud.at<cv::Point3f>(row4tel, col4tel) = DepthToWorld(col, row, (int) depth);
        colCloud.at<cv::Point3f>(row4tel, col4tel) = WorldToColorInternal(col*rgb2depthRatio, row*rgb2depthRatio);
      }
      else {
        /* Initialize points if we have no valid data (to transmit via ice-interface) */
        cloud.at<cv::Point3f>(row4tel, col4tel) = cv::Point3f(FLT_MAX, FLT_MAX, FLT_MAX);
        colCloud.at<cv::Point3f>(row4tel, col4tel) = WorldToColorInternal(col*rgb2depthRatio, row*rgb2depthRatio);
      }
    }
  }
}


}




