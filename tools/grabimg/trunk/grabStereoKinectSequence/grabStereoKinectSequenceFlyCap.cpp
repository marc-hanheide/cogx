/**
 * 
 * @brief Save a sequence of images/point clouds from a kinect sensor and a stereo camera setup.
 * @author Richtsfeld, Zillich
 * @date March 2012
 * 
 * Compile:
 *    cmake .
 *    make
 *    
 * Usage:
 *    q ... quit
 *    g ... grab grey level images (bmp) (for camera calibration)
 *    
 */


#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Kinect.h"
#include "FileReaderWriter.h"
#include <flycapture/FlyCapture2.h>

#define ESCAPE 27

bool left_stereo_only = false;
IplImage* frame_stereo_left = 0;
IplImage* frame_stereo_right = 0;
IplImage* frame_kinect = 0;
cv::Mat_<cv::Point3f> cloud;
cv::Mat_<cv::Point3f> colCloud;
int imgcnt = 0;

void Usage(const char *argv0)
{
  printf(
    "usage: %s [-h] [-l]\n"
    " -h .. this help\n"
    " -l .. left stereo camera only\n",
    argv0);
}

void SaveCurrentImage(bool color)
{
printf("SaveCurrentImage!\n");
  char filename_stereo_left[1024];
  char filename_stereo_right[1024];
  char filename_kinect[1024];
  char filename_kinect_cloud[1024];
  if(!frame_stereo_left || !frame_kinect) {
    printf("SaveCurrentImages: Error: Unable to save images!\n");
    return;
  }
  if(!left_stereo_only && !frame_stereo_right) {
      printf("SaveCurrentImages: Error: Unable to save images!\n");
    return;
  }
  
  if(color)
  {  
    snprintf(filename_stereo_left, 1024, "img%03d-L.jpg", imgcnt);
    if(!left_stereo_only) snprintf(filename_stereo_right, 1024, "img%03d-R.jpg", imgcnt);
    snprintf(filename_kinect, 1024, "img%03d-K.jpg", imgcnt);
    snprintf(filename_kinect_cloud, 1024, "img%03d-C.pts", imgcnt++);

    cvSaveImage(filename_stereo_left, frame_stereo_left, 0);
    if(!left_stereo_only) cvSaveImage(filename_stereo_right, frame_stereo_right, 0);
    cvSaveImage(filename_kinect, frame_kinect, 0);
    AR::writeToFile(filename_kinect_cloud, cloud, colCloud);
  }
  else
  {
    IplImage *greyStereoLeft = cvCreateImage(cvSize(frame_stereo_left->width, frame_stereo_left->height), IPL_DEPTH_8U, 1);
    IplImage *greyStereoRight;
    if(!left_stereo_only) greyStereoRight = cvCreateImage(cvSize(frame_stereo_right->width, frame_stereo_right->height), IPL_DEPTH_8U, 1);
    IplImage *greyKinect = cvCreateImage(cvSize(frame_kinect->width, frame_kinect->height), IPL_DEPTH_8U, 1);

    cvCvtColor(frame_stereo_left, greyStereoLeft, CV_BGR2GRAY);
    if(!left_stereo_only) cvCvtColor(frame_stereo_right, greyStereoRight, CV_BGR2GRAY);
    cvCvtColor(frame_kinect, greyKinect, CV_BGR2GRAY);

    snprintf(filename_stereo_left, 1024, "img%03d-L.bmp", imgcnt);
    if(!left_stereo_only) snprintf(filename_stereo_right, 1024, "img%03d-R.bmp", imgcnt);
    snprintf(filename_kinect, 1024, "img%03d-K.bmp", imgcnt);
    snprintf(filename_kinect_cloud, 1024, "img%03d-C.pts", imgcnt++);

    cvSaveImage(filename_stereo_left, greyStereoLeft);
    if(!left_stereo_only) cvSaveImage(filename_stereo_right, greyStereoRight);
    cvSaveImage(filename_kinect, greyKinect);
    AR::writeToFile(filename_kinect_cloud, cloud, colCloud);

    cvReleaseImage(&greyStereoLeft);
    cvReleaseImage(&greyStereoRight);
    cvReleaseImage(&greyKinect);
  }
}

int main(int argc, char** argv)
{
  bool done = false;
  int c = -1;
  Kinect::Kinect *kinect;
  kinect = new Kinect::Kinect("KinectConfig.xml");
  kinect->StartCapture(0);

  /// Init fly-capture cameras
  FlyCapture2::BusManager busMgr;                     ///< FlyCapture2 bus manager
  FlyCapture2::Camera **cameras;                      ///< FlyCapture2 cameras
  std::vector<FlyCapture2::Image> retrievedImages;    ///< FlyCapture2 retrieved images from different cameras
//   int width = 640;
//   int height = 480;
//   int fps = 15;
  
  while((c = getopt(argc, argv, ":lh")) != -1)
  {
    switch(c)
    {
      case 'l':
        left_stereo_only = true;
        break;
      case 'h':
      default:
        Usage(argv[0]);
        exit(0);
        break;
    }
  }

  int numCams = 0;
  if(!left_stereo_only) numCams = 2;
  else numCams = 1;
  
  unsigned int numAvailableCameras;
  FlyCapture2::Error error;
  error = busMgr.GetNumOfCameras(&numAvailableCameras);
  if(error != FlyCapture2::PGRERROR_OK)
    throw std::runtime_error(error.GetDescription());

  if(numAvailableCameras < numCams)
    throw std::runtime_error("PointGreyServer: insufficient number of cameras detected");
  cameras = new FlyCapture2::Camera*[numCams];
  retrievedImages.resize(numCams);

  // Connect to all detected cameras and attempt to set them to a common video mode and frame rate
  for(size_t i = 0; i < (size_t)numCams; i++) {
    cameras[i] = new FlyCapture2::Camera();

    FlyCapture2::PGRGuid guid;
    error = busMgr.GetCameraFromIndex(i, &guid);
    if(error != FlyCapture2::PGRERROR_OK)
      throw std::runtime_error(error.GetDescription());

    error = cameras[i]->Connect(&guid);
    if(error != FlyCapture2::PGRERROR_OK)
      throw std::runtime_error(error.GetDescription());

//     printf("Setting video mode %d x %d with %d fps\n", width, height, fps);
    FlyCapture2::VideoMode mode;
    FlyCapture2::FrameRate rate;
    if(left_stereo_only) {
      mode = FlyCapture2::VIDEOMODE_640x480Y8;
      rate = FlyCapture2::FRAMERATE_15;
    }
    else {
      mode = FlyCapture2::VIDEOMODE_640x480Y8;
      rate = FlyCapture2::FRAMERATE_7_5;
    }

    error = cameras[i]->SetVideoModeAndFrameRate(mode, rate);
    if(error != FlyCapture2::PGRERROR_OK)
      throw std::runtime_error(error.GetDescription());
  }

  // start capturing of flyCap images
  for(size_t i=0; i<numCams; i++) {
    error = cameras[i]->StartCapture();
    if(error != FlyCapture2::PGRERROR_OK)
      throw std::runtime_error(error.GetDescription());
  }

  
  printf("Hot keys: \n"
         "\tESC, q - quit the program\n"
         "\tg - save current image (bmp - grey level)\n"
         "\tc - save current image (jpg - color)\n");

  cvNamedWindow("Left stereo image", CV_WINDOW_AUTOSIZE);
  if(!left_stereo_only) cvNamedWindow("Right stereo image", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Kinect image", CV_WINDOW_AUTOSIZE);
  
  while(!done)
  {
    // grab flyCap images
    for(size_t i = 0; i < numCams; i++) {
      FlyCapture2::Error error;
      error = cameras[i]->RetrieveBuffer(&retrievedImages[i]);
      if(error != FlyCapture2::PGRERROR_OK)
        throw std::runtime_error(error.GetDescription());
    }
    
    FlyCapture2::Image left_image;
    if(retrievedImages[0].GetPixelFormat() != FlyCapture2::PIXEL_FORMAT_RGB8) 
      retrievedImages[0].Convert(FlyCapture2::PIXEL_FORMAT_RGB8, &left_image);
    else
      left_image = retrievedImages[0];
    
    FlyCapture2::Image right_image;
    if(!left_stereo_only)
    {
      if(retrievedImages[1].GetPixelFormat() != FlyCapture2::PIXEL_FORMAT_RGB8) 
        retrievedImages[1].Convert(FlyCapture2::PIXEL_FORMAT_RGB8, &right_image);
      else
        right_image = retrievedImages[1];
    }    

    /// wrap flyCap image(s) into ipl-image
    if(left_image.GetPixelFormat() != FlyCapture2::PIXEL_FORMAT_RGB8)
      throw std::runtime_error("PointGreyServerUSB: can only handle RGB8 image format");
    if(left_image.GetStride() != left_image.GetCols()*3)
      throw std::runtime_error("PointGreyServerUSB: can only handle images with no padding");
    if(left_image.GetDataSize() != left_image.GetCols()*left_image.GetRows()*3)
      throw std::runtime_error("PointGreyServerUSB: can only handle images with size 3*w*h bytes");

    // HACK: Data shared between IplImage and FlyCapture2::Image
    frame_stereo_left = cvCreateImageHeader(cvSize(left_image.GetCols(), left_image.GetRows()), IPL_DEPTH_8U, 3);
    frame_stereo_left->imageData = (char*) left_image.GetData();
    frame_stereo_left->imageDataOrigin = frame_stereo_left->imageData;
    frame_stereo_left->widthStep = left_image.GetCols() * 3;
    frame_stereo_left->imageSize = frame_stereo_left->widthStep * left_image.GetRows();
    
    if(!left_stereo_only) {
      frame_stereo_right = cvCreateImageHeader(cvSize(right_image.GetCols(), right_image.GetRows()), IPL_DEPTH_8U, 3);
      frame_stereo_right->imageData = (char*) right_image.GetData();
      frame_stereo_right->imageDataOrigin = frame_stereo_right->imageData;
      frame_stereo_right->widthStep = right_image.GetCols() * 3;
      frame_stereo_right->imageSize = frame_stereo_right->widthStep * right_image.GetRows();      
    }
    
    // grab kinect image and point cloud
    kinect->GetColorImage(&frame_kinect);
    kinect->Get3dWorldPointCloud(cloud, colCloud, 1);

    if(!frame_kinect) {
        printf("Failed to capture Kinect frame. Stopping\n");
        break;
    }

    cvShowImage("Left stereo image", frame_stereo_left);
    if(!left_stereo_only) cvShowImage("Right stereo image", frame_stereo_right);
    cvShowImage("Kinect image", frame_kinect);

    // note: filter the integer you get with 0xFF - it seems the low byte is
    // actually the correct ASCII key
    switch(cvWaitKey(10) & 0xFF)
    {
      case ESCAPE:
      case 'q':
        done = true;
        break;
      case 'g':
          SaveCurrentImage(false);
          break;
      case 'c':
          SaveCurrentImage(true);
          break;
      default:
        break;
    }


    cvReleaseImage(&frame_kinect);
    cloud.release();
    colCloud.release();
  }

  kinect->StopCapture();
  delete kinect;

  cvDestroyWindow("Left stereo image");
  cvDestroyWindow("Right stereo image");
  cvDestroyWindow("Kinect image");

  exit(0);
}

