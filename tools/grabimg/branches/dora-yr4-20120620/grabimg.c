/**
 * display current camera image and allow saving to file.
 *
 * gcc -o grabimg grabimg.c `pkg-config opencv --cflags --libs`
 */

#ifdef _CH_
#pragma package <opencv>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>
#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#endif

#define ESCAPE 27

IplImage* frame = 0;
int imgcnt = 0;

void Usage(const char *argv0)
{
  printf(
    "usage: %s [-h] [-b] [-f | -v] [-d device]\n"
    " -h .. this help\n"
    " -d device id .. capture from device, e.g. 0, 1. default is 0\n"
    " -v .. device class V4L2, i.e. typically USB cameras\n"
    " -f .. device class firewire (IEEE1394)\n"
    "       default is ANY, whichever is found first\n"
    "       Note that in case the specified device class has no camera of the\n"
    "       given id, the first other available camera will be selected\n"
    " -b .. perform Bayer to RGB conversion\n"
    "       (for cameras that only return Bayer images), default is no\n",
    argv0);
}

void SaveCurrentImage()
{
  char filename[1024];
  if(!frame)
    return;
  snprintf(filename, 1024, "img%03d.jpg", imgcnt++);
  cvSaveImage(filename, frame, 0);
  printf("written image '%s'\n", filename);
}

int main(int argc, char** argv)
{
  CvCapture* capture = 0;
  int device_class = CV_CAP_ANY;  // or V4L2, IEEE1394
  int cam = 0;    // camera (device) number
  int bayer = 0;  // do Bayer->RGB conversion
  int c;
  extern char *optarg;
  int done = 0;

  while((c = getopt(argc, argv, "bd:vfh")) != -1)
  {
    switch(c)
    {
      case 'b':
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
        break;
      case 'h':
      default:
        Usage(argv[0]);
        exit(0);
        break;
    }
  }

  capture = cvCreateCameraCapture(device_class + cam);
  if(!capture)
  {
    fprintf(stderr,"Could not initialize capturing...\n");
    exit(1);
  }

  /* print a welcome message, and the OpenCV version */
  printf("Welcome to grabimg, using OpenCV version %s (%d.%d.%d)\n",
      CV_VERSION, CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_SUBMINOR_VERSION);

  //printf("set convert: %d\n",
  //  cvSetCaptureProperty(capture, CV_CAP_PROP_CONVERT_RGB));
  //printf("get mode: %d\n",
  //  cvGetCaptureProperty(capture, CV_CAP_PROP_MODE));
  /*printf("%d\n",
      cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640.0));
  printf("%d\n",
      cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480.0));*/
  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
  printf("image size %.0f x %.0f\n",
      cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH),
      cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT));
  printf("framerate %.2f Hz\n", cvGetCaptureProperty(capture, CV_CAP_PROP_FPS));
  if(bayer)
    cvSetCaptureProperty(capture, CV_CAP_PROP_CONVERT_RGB, 0.0);

  printf("Hot keys: \n"
         "\tESC, q - quit the program\n"
         "\tg - save current image\n");

  cvNamedWindow("grabimg", 0);
  frame = cvQueryFrame( capture );
  printf("frame size: %d x %d\n", frame->width, frame->height);
  cvResizeWindow("grabimg", frame->width, frame->height);

  while(!done)
  {
    frame = cvQueryFrame(capture);
    if(!frame)
    {
        printf("failed to capture frame, stopping\n");
        break;
    }
    if(bayer)
    {
        IplImage *buf = cvCreateImage(cvSize(frame->width, frame->height),
            IPL_DEPTH_8U, 3);
        cvCvtColor(frame, buf, CV_BayerRG2RGB);
        frame = buf;  // don't forget to free later
    }

    cvShowImage("grabimg", frame);

    // note: filter the integer you get with 0xFF - it seems the low byte is
    // actually the correct ASCII key
    switch(cvWaitKey(10) & 0xFF)
    {
      case ESCAPE:
      case 'q':
        done = 1;
        break;
      case 'g':
          SaveCurrentImage();
          break;
      default:
        break;
    }

    if(bayer)
      cvReleaseImage(&frame);
  }

  cvReleaseCapture(&capture);
  cvDestroyWindow("grabimg");

  exit(0);
}

