/**
 * @file Kinect.cpp
 * @author Richtsfeld
 * @date March 2011
 * @version 0.1
 * @brief Wraper for the OpenNI Driver for the Kinect sensor for use inside of cast-framework
 */


#include "Kinect.h"
#include <ni/XnCodecIDs.h>

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
    printf("Kinect: Error: Initialisation not successful!\n");
    exit(0);
  }
}

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
    printf("Kinect::Init: Error: Initialisation from xml-file failed. Check filename and connection to Kinect.\n");

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
  
  rc = kinect::getDepthGenerator()->GetIntProperty ("ShadowValue", no_sample_value);
  if (rc != XN_STATUS_OK)
    printf("Kinect::Init: Error: Could not read \"no sample\" value.\n");
  

  rgbWidth = 640;	/// TODO Get width and height from file!
  rgbHeight = 480;
  depWidth = 640;
  depHeight = 480;

  rc = kinect::getDepthGenerator()->GetRealProperty("ZPPS", pixel_size);
  if (rc != XN_STATUS_OK)
    printf("Kinect::Init: Error: Geting pixel size failed.\n");

  rc = kinect::getDepthGenerator()->GetIntProperty("ZPD", depth_focal_length_SXGA);
  if (rc != XN_STATUS_OK)
    printf("Kinect::Init: Error: Geting focal length failed.\n");
  
  centerX = (depWidth >> 1) - 0.5f;
  centerY = (depHeight >> 1) - 0.5f;
  depth_focal_length_SXGA_ = (float)depth_focal_length_SXGA / pixel_size;
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

/**							/// TODO überarbeiten: NextFrame verwenden und dann xx2IplImage ausführen!
 * @brief Get color image as openCV iplImage from the Kinect sensor
 * @param iplImg Video image as ipl image.
 * @return Return true, if both images are captured successful.
 */
bool Kinect::GetColorImage(IplImage **iplImg)
{
  NextFrame();
  (*iplImg) = cvCreateImage(cvSize(rgbWidth, rgbHeight), IPL_DEPTH_8U, 3);
  IplImage tmp = rgbImage;
  cvCopy(&tmp, (*iplImg));
  return true;
}

/**							/// TODO überarbeiten: NextFrame verwenden und dann xx2IplImage ausführen!
 * @brief Get Frame from the Kinect sensor
 * @param iplImg Video image as ipl image.
 * @param iplDepthImg Depth image as ipl-image.
 * @return Return true, if both images are captured successful.
 */
bool Kinect::GetFrame(IplImage **iplImg, IplImage **iplDepthImg)
{
  printf("Kinect::GetFrame: Not yet implemented!\n");

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
//     return false;
//   }
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
    
  kinect::readFrame();  // read next frame

  // get depth image
  const DepthMetaData* pDepthMD = kinect::getDepthMetaData();
  if (kinect::isDepthOn())
  {
    depImage = cv::Mat(depHeight, depWidth, CV_16S);
    short *d  = depImage.ptr<short>(0);
    for(unsigned co=0; co<depHeight*depWidth; co++)
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
    for(unsigned co=0; co<rgbHeight*rgbWidth; co++)
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
 * @brief Get undistorted and registered images from the kinect sensor.
 * @param rgbImg Video image as openCV matrix.
 * @param depImg Depth image as openCV matrix with 16 bit depth!
 * @return Return true, if both images are captured successful.
 */
bool Kinect::GetImages(cv::Mat &rgbImg, cv::Mat &depImg)
{
  rgbImg = rgbImage;
  depImg = depImage;
  return true;
}

/**
 * @brief Transform depth point to world coordinates.
 * @param x x-coordinate
 * @param y y-coordinate
 * @param depthValue Raw depth value from the sensor.
 * @return Returns the 3d point in the world coordinate system.
 */
cv::Point3f Kinect::DepthToWorld(int x, int y, int depthValue)
{
  cv::Point3f result;
  result.x = (x - centerX) * depthValue * constant;
  result.y = (y - centerY) * depthValue * constant;
  result.z = depthValue * 0.001;
  return result;
}

/**
 * @brief Get the color for world points.
 * @param x x-coordinate in depth image
 * @param y y-coordinate in depth image
 * @return Returns the color as 3d point.
 */
cv::Point3f Kinect::WorldToColor(unsigned x, unsigned y)
{
  uchar *ptr = rgbImage.data;
  cv::Point3f col;
  col.x = ptr[(y*rgbWidth + x)*3 +2];	// change red and blue channel
  col.y = ptr[(y*rgbWidth + x)*3 +1];
  col.z = ptr[(y*rgbWidth + x)*3];
  return col;
}

/**
 * @brief Get a world point from the kinect image.
 * @param x x-coordinate of the point on the image plane.
 * @param y y-coordinate of the point on the image plane.
 * @return Returns the 3d world point.
 */
cv::Point3f Kinect::Get3dWorldPoint(unsigned x, unsigned y)
{
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
  cloud = cv::Mat_<cv::Point3f>(rgbHeight/scale, rgbWidth/scale);
  colCloud = cv::Mat_<cv::Point3f>(rgbHeight/scale, rgbWidth/scale);
  for(unsigned row = 0; row<depHeight; row+=scale)
  {
    for(unsigned col = 0; col<depWidth; col+=scale)
    {
      short depth = depImage.at<short>(row, col);
      if(depth != shadow_value && depth != no_sample_value)
      {
	int col4tel = col/scale;
	int row4tel = row/scale;
	cloud.at<cv::Point3f>(row4tel, col4tel) = DepthToWorld(col, row, (int) depth);
	
	int rgb2depthRatio = rgbWidth/depWidth;									/// TODO Bei 1280 Auflösung gibt es eine Verzerrung in z-Richtung?
	colCloud.at<cv::Point3f>(row4tel, col4tel) = WorldToColor(col*rgb2depthRatio, row*rgb2depthRatio);
      }
    }
  }
}


}




