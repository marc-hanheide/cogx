/**
 * display current camera image and allow saving to file.
 *
 * gcc -o grabstereo grabstereo.c `pkg-config opencv --cflags --libs`
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

IplImage* frame[2] = {0, 0};
int imgcnt = 0;

void Usage(const char *argv0)
{
  printf(
    "Saves stereo image pairs img000-L.bmp, img000-R.bmp, img001-L.bmp ...\n"
    "Default format is grey level BMP (as this is required for SVS calibration\n"
    "software) but can be changed to color JPG with option -c\n"
    "usage: %s [-h] [-f | -v] -c [-d \"left right\"]\n"
    " -h .. this help\n"
    " -d left right .. device IDs for left and right images, e.g. \"0 1\". default is 0 and 1\n"
    " -v .. device class V4L2, i.e. typically USB cameras\n"
    " -f .. device class firewire (IEEE1394)\n"
    "       default is ANY, whichever is found first\n"
    "       Note that in case the specified device class has no camera of the\n"
    "       given id, the first other available camera will be selected\n"
    " -c .. save color .jpg images instead of grey .bmp images (the default)\n",
    argv0);
}

void SaveCurrentImages(int save_color)
{
  char filename[1024];
  int i;
  for(i = 0; i <= 1; i++)
  {
    if(!frame[i])
      return;
    if(save_color)
    {
      snprintf(filename, 1024, "img%03d-%c.jpg", imgcnt, i == 0 ? 'L' : 'R');
      cvSaveImage(filename, frame[i]);
    }
    else
    {
      IplImage *grey = cvCreateImage(cvSize(frame[0]->width, frame[0]->height),
          IPL_DEPTH_8U, 1);
      cvCvtColor(frame[i], grey, CV_BGR2GRAY);
      snprintf(filename, 1024, "img%03d-%c.bmp", imgcnt, i == 0 ? 'L' : 'R');
      cvSaveImage(filename, grey);
      cvReleaseImage(&grey);
    }
    printf("written image '%s'\n", filename);
  }
  imgcnt++;
}

int main(int argc, char** argv)
{
  CvCapture* capture[2] = {0, 0};
  int device_class = CV_CAP_ANY;  // or V4L2, IEEE1394
  int cam[2] = {0, 1};    // camera (device) numbers
  int c;
  extern char *optarg;
  int done = 0;
  int save_color = 0;
  int i;

  while((c = getopt(argc, argv, "bd:vfhc")) != -1)
  {
    switch(c)
    {
      case 'd':
      {
        if(sscanf(optarg, "%d %d", &cam[0], &cam[1]) != 2)
        {     
          fprintf(stderr,"must provide two device IDs\n");
          exit(1);
        }
        break;
      }
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
      case 'c':
        save_color = 1;
        break;
      case 'h':
      default:
        Usage(argv[0]);
        exit(0);
        break;
    }
  }

  /* print a welcome message, and the OpenCV version */
  printf("Welcome to grabstereo, using OpenCV version %s (%d.%d.%d)\n",
      CV_VERSION, CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_SUBMINOR_VERSION);

  for(i = 0; i <= 1; i++)
  {
    capture[i] = cvCreateCameraCapture(device_class + cam[i]);
    if(!capture[i])
    {
      fprintf(stderr,"Could not initialize capturing...\n");
      exit(1);
    }

    //printf("set convert: %d\n",
    //  cvSetCaptureProperty(capture[i], CV_CAP_PROP_CONVERT_RGB));
    //printf("get mode: %d\n",
    //  cvGetCaptureProperty(capture[i], CV_CAP_PROP_MODE));
    /*printf("%d\n",
        cvSetCaptureProperty(capture[i], CV_CAP_PROP_FRAME_WIDTH, 640.0));
    printf("%d\n",
        cvSetCaptureProperty(capture[i], CV_CAP_PROP_FRAME_HEIGHT, 480.0));*/
    cvSetCaptureProperty(capture[i], CV_CAP_PROP_FRAME_WIDTH, 640);
    cvSetCaptureProperty(capture[i], CV_CAP_PROP_FRAME_HEIGHT, 480);
    printf("image size %.0f x %.0f\n",
        cvGetCaptureProperty(capture[i], CV_CAP_PROP_FRAME_WIDTH),
        cvGetCaptureProperty(capture[i], CV_CAP_PROP_FRAME_HEIGHT));
    printf("framerate %.2f Hz\n", cvGetCaptureProperty(capture[i], CV_CAP_PROP_FPS));
  }

  printf("Hot keys: \n"
         "\tESC, q - quit the program\n"
         "\tg - save current image\n");

  cvNamedWindow("grabstereo - left", 0);
  cvNamedWindow("grabstereo - right", 0);
  frame[0] = cvQueryFrame( capture[0] );
  printf("frame size: %d x %d\n", frame[0]->width, frame[0]->height);
  cvResizeWindow("grabstereo - left", frame[0]->width, frame[0]->height);
  cvResizeWindow("grabstereo - right", frame[0]->width, frame[0]->height);

  while(!done)
  {
    // first grab quickly
    for(i = 0; i <= 1; i++)
    {
      cvGrabFrame(capture[i]);
    }
    // then retrieve actual frames
    for(i = 0; i <= 1; i++)
    {
      frame[i] = cvRetrieveFrame(capture[i], 0);
      if(!frame[i])
      {
        printf("failed to capture frame, stopping\n");
        break;
      }
    }

    cvShowImage("grabstereo - left", frame[0]);
    cvShowImage("grabstereo - right", frame[1]);

    // note: filter the integer you get with 0xFF - it seems the low byte is
    // actually the correct ASCII key
    switch(cvWaitKey(10) & 0xFF)
    {
      case ESCAPE:
      case 'q':
        done = 1;
        break;
      case 'g':
          SaveCurrentImages(save_color);
          break;
      default:
        break;
    }
  }

  cvReleaseCapture(&capture[0]);
  cvReleaseCapture(&capture[1]);
  cvDestroyWindow("grabstereo - left");
  cvDestroyWindow("grabstereo - right");

  exit(0);
}

