/**
 * 
 * @brief Save a sequence of images/point clouds from a kinect sensor and a stereo camera setup.
 * @author Richtsfeld, Zillich, Bader
 * @date July 2011
 * 
 * Compile:
 *    mkdir BUILD
 *    cd BUILD
 *    ccmake ..
 *        (Then c and g)
 *    make
 *    
 * Usage:
 *    q ... quit
 *    g ... grab grey level images (bmp) (for camera calibration)
 *    c ... grap color images (jpg)
 */


#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>

#include "cv.h"
#include "highgui.h"

#include "v4r/grabStereoKinectSequence/Kinect.h"
#include "v4r/grabStereoKinectSequence/FileReaderWriter.h"

#define ESCAPE 27

IplImage* frame_kinect = 0;
cv::Mat_<cv::Point3f> cloud;
cv::Mat_<cv::Point3f> colCloud;
int imgcnt = 0;

void Usage(const char *argv0)
{
  printf("usage: %s\n", argv0);
  printf("Needs configuration file KinectConfig.xml in the same directory!\n", argv0);
}

void SaveCurrentImage(bool color)
{
  char filename_kinect[1024];
  char filename_kinect_cloud[1024];
  if(!frame_kinect)
  {
    printf("SaveCurrentImages: Error: Unable to save images!\n");
    return;
  }
  
  if(color)
  {  
    snprintf(filename_kinect, 1024, "img%03d-K.jpg", imgcnt);
    snprintf(filename_kinect_cloud, 1024, "img%03d-C.pts", imgcnt++);

    cvSaveImage(filename_kinect, frame_kinect, 0);
    AR::writeToFile(filename_kinect_cloud, cloud, colCloud);

    // for testing
//     cv::Mat_<cv::Point3f> rcloud;
//     cv::Mat_<cv::Point3f> rcolCloud;
//     AR::readFromFile(filename_kinect_cloud, rcloud, rcolCloud);
//     AR::writeToFile("copy.dat", rcloud, rcolCloud);
    
    printf("written images for kinect '%s' & cloud: '%s'\n", filename_kinect, filename_kinect_cloud);
  }
  else
  {
    IplImage *greyKinect = cvCreateImage(cvSize(frame_kinect->width, frame_kinect->height), IPL_DEPTH_8U, 1);
    cvCvtColor(frame_kinect, greyKinect, CV_BGR2GRAY);

    snprintf(filename_kinect, 1024, "img%03d-K.bmp", imgcnt);
    snprintf(filename_kinect_cloud, 1024, "img%03d-C.pts", imgcnt++);

    cvSaveImage(filename_kinect, greyKinect);
    AR::writeToFile(filename_kinect_cloud, cloud, colCloud);

    cvReleaseImage(&greyKinect);
    printf("written grey images for kinect '%s' & cloud: '%s'\n", filename_kinect, filename_kinect_cloud);
  }
}

int main(int argc, char** argv)
{
  Kinect::Kinect *kinect;
  kinect = new Kinect::Kinect("../v4r/grabStereoKinectSequence/KinectConfig.xml");
  kinect->StartCapture(0);

//   int device_class = CV_CAP_ANY;  // or V4L2, IEEE1394
//   int cam = 0;    // camera (device) number
//   int bayer = 0;  // do Bayer->RGB conversion
  int c;
  extern char *optarg;
  int done = 0;

  while((c = getopt(argc, argv, "h")) != -1)
  {
    switch(c)
    {
/*      case 'b':
        bayer = 1;
        break;
      case 'd':
        cam = atoi(optarg);
        break;
      case 'v':
        if(device_class == CV_CAP_ANY)
          device_class = CV_CAP_V4L2;
        else
          printf("device class can be set to either V4L2 or IEEE1394\n");
        break;
      case 'f':
        if(device_class == CV_CAP_ANY)
          device_class = CV_CAP_IEEE1394;
        else
          printf("device class can be set to either V4L2 or IEEE1394\n");
        break;*/
      case 'h':
      default:
        Usage(argv[0]);
        exit(0);
        break;
    }
  }
  
printf("after getopt!\n");

//   capture_left = cvCreateCameraCapture(device_class + cam);
//   capture_right = cvCreateCameraCapture(device_class + cam + 1);
//   if(!capture_left || !capture_right)
//   {
//     fprintf(stderr,"Could not initialize capturing...\n");
//     exit(1);
//   }

  /* print a welcome message, and the OpenCV version */
  printf("Start grabing images, using OpenCV version %s (%d.%d.%d)\n",
      CV_VERSION, CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_SUBMINOR_VERSION);

  //printf("set convert: %d\n",
  //  cvSetCaptureProperty(capture, CV_CAP_PROP_CONVERT_RGB));
  //printf("get mode: %d\n",
  //  cvGetCaptureProperty(capture, CV_CAP_PROP_MODE));
  /*printf("%d\n",
      cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640.0));
  printf("%d\n",
      cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480.0));*/
//   cvSetCaptureProperty(capture_left, CV_CAP_PROP_FRAME_WIDTH, 640);
//   cvSetCaptureProperty(capture_left, CV_CAP_PROP_FRAME_HEIGHT, 480);
//   cvSetCaptureProperty(capture_right, CV_CAP_PROP_FRAME_WIDTH, 640);
//   cvSetCaptureProperty(capture_right, CV_CAP_PROP_FRAME_HEIGHT, 480);
         
  /// for what?
//   printf("image size left stereo %.0f x %.0f\n",
//     cvGetCaptureProperty(capture_left, CV_CAP_PROP_FRAME_WIDTH),
//     cvGetCaptureProperty(capture_left, CV_CAP_PROP_FRAME_HEIGHT));
//   printf("image size right stereo %.0f x %.0f\n",
//     cvGetCaptureProperty(capture_right, CV_CAP_PROP_FRAME_WIDTH),
//     cvGetCaptureProperty(capture_right, CV_CAP_PROP_FRAME_HEIGHT));
//   printf("framerate left %.2f Hz\n", cvGetCaptureProperty(capture_left, CV_CAP_PROP_FPS));
//   printf("framerate right %.2f Hz\n", cvGetCaptureProperty(capture_right, CV_CAP_PROP_FPS));
//   if(bayer)
//   {
//     cvSetCaptureProperty(capture_left, CV_CAP_PROP_CONVERT_RGB, 0.0);
//     cvSetCaptureProperty(capture_right, CV_CAP_PROP_CONVERT_RGB, 0.0);
//   }

  printf("Hot keys: \n"
         "\tESC, q - quit the program\n"
         "\tg - save current image (bmp - grey level)\n"
         "\tc - save current image (jpg - color)\n");

//   cvNamedWindow("Left stereo image", CV_WINDOW_AUTOSIZE);
//   cvNamedWindow("Right stereo image", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Kinect image", CV_WINDOW_AUTOSIZE);
//   frame_stereo_left = cvQueryFrame( capture_left );
//   frame_stereo_right = cvQueryFrame( capture_right );

//   printf("frame size: %d x %d\n", frame_stereo_left->width, frame_stereo_left->height);
//   cvResizeWindow("grabimg", frame_stereo_left->width, frame_stereo_left->height);

  while(!done)
  {
//     frame_stereo_left = cvQueryFrame(capture_left);
//     frame_stereo_right = cvQueryFrame(capture_right);
    kinect->GetColorImage(&frame_kinect);
    kinect->Get3dWorldPointCloud(cloud, colCloud, 1);

    if(/*!frame_stereo_left || !frame_stereo_right || */!frame_kinect)
    {
        printf("failed to capture frame, stopping\n");
        break;
    }
//     if(bayer)
//     {
//         IplImage *buf0 = cvCreateImage(cvSize(frame_stereo_left->width, frame_stereo_left->height), IPL_DEPTH_8U, 3);
//         cvCvtColor(frame_stereo_left, buf0, CV_BayerRG2RGB);
//         frame_stereo_left = buf0;  // TODO don't forget to free later
//         cvReleaseImage(&buf0);
// 
//         IplImage *buf1 = cvCreateImage(cvSize(frame_stereo_left->width, frame_stereo_left->height), IPL_DEPTH_8U, 3);
//         cvCvtColor(frame_stereo_left, buf1, CV_BayerRG2RGB);
//         frame_stereo_left = buf1;  // TODO don't forget to free later
//         cvReleaseImage(&buf1);
//     }

//     cvShowImage("Left stereo image", frame_stereo_left);
//     cvShowImage("Right stereo image", frame_stereo_right);
    cvShowImage("Kinect image", frame_kinect);

    // note: filter the integer you get with 0xFF - it seems the low byte is
    // actually the correct ASCII key
    switch(cvWaitKey(10) & 0xFF)
    {
      case ESCAPE:
      case 'q':
        done = 1;
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

//     if(bayer)
//     {
//       cvReleaseImage(&frame_stereo_left);
//       cvReleaseImage(&frame_stereo_right);
//       cvReleaseImage(&frame_kinect);
//     }
    cvReleaseImage(&frame_kinect);
    cloud.release();
    colCloud.release();
  }

  kinect->StopCapture();
  delete kinect;

//   cvReleaseCapture(&capture_left);
//   cvReleaseCapture(&capture_right);
//   cvDestroyWindow("Left stereo image");
//   cvDestroyWindow("Right stereo image");
  cvDestroyWindow("Kinect image");

  exit(0);
}

