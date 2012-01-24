#include <cstdlib>
#include <cstdio>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "CensusGPU.h"


#define ESCAPE 27

void Usage(const char *argv0)
{
  printf(
    "usage: %s [-h] [-f | -v] [-d \"left right\"]\n"
    " -h .. this help\n"
    " -d left right .. device IDs for left and right images, e.g. \"0 1\". default is 0 and 1\n"
    " -v .. device class V4L2, i.e. typically USB cameras\n"
    " -f .. device class firewire (IEEE1394)\n"
    "       default is ANY, whichever is found first\n"
    "       Note that in case the specified device class has no camera of the\n"
    "       given id, the first other available camera will be selected\n",
    argv0);
}

/*void SaveCurrentImages()
{
  char filename[1024];
  int i;
  for(i = 0; i <= 1; i++)
  {
    if(!img[i])
      return;
    snprintf(filename, 1024, "img%03d-%c.jpg", imgcnt, i == 0 ? 'L' : 'R');
    cvSaveImage(filename, img[i], 0);
    printf("written image '%s'\n", filename);
  }
  imgcnt++;
}*/

int main(int argc, char** argv)
{
  CvCapture* capture[2] = {0, 0};
  int device_class = CV_CAP_ANY;  // or V4L2, IEEE1394
  int cam[2] = {0, 1};    // camera (device) numbers
	IplImage *img[2];
  IplImage *grey[2];
  IplImage *disp;
	CensusGPU census;
  int c;
  extern char *optarg;
  extern int optind, optopt;
  int done = 0;
  int i;

  while((c = getopt(argc, argv, "bd:vfh")) != -1)
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
    printf("imgrate %.2f Hz\n", cvGetCaptureProperty(capture[i], CV_CAP_PROP_FPS));
  }

  printf("Hot keys: \n"
         "\tESC, q - quit the program\n"
         "\tg - save current image\n");

  // get an image for size etc.
  img[0] = cvQueryFrame( capture[0] );
  printf("img size: %d x %d\n", img[0]->width, img[0]->height);

  cvNamedWindow("grabstereo - left", 0);
  cvNamedWindow("grabstereo - right", 0);
  cvNamedWindow("grabstereo - disparity", 0);
  // display camera images in half size
  cvResizeWindow("grabstereo - left", img[0]->width/2, img[0]->height/2);
  cvResizeWindow("grabstereo - right", img[0]->width/2, img[0]->height/2);
  // display disparity image in full size
  cvResizeWindow("grabstereo - disparity", img[0]->width, img[0]->height);

	disp = cvCreateImage(cvSize(img[0]->width, img[0]->height), IPL_DEPTH_8U, 1);
	grey[0] = cvCreateImage(cvSize(img[0]->width, img[0]->height), IPL_DEPTH_8U, 1);
	grey[1] = cvCreateImage(cvSize(img[0]->width, img[0]->height), IPL_DEPTH_8U, 1);

  while(!done)
  {
    // first grab quickly
    for(i = 0; i <= 1; i++)
    {
      cvGrabFrame(capture[i]);
    }
    // then retrieve actual imgs
    for(i = 0; i <= 1; i++)
    {
      img[i] = cvRetrieveFrame(capture[i], 0);
      if(!img[i])
      {
        printf("failed to capture img, stopping\n");
        break;
      }
      cvCvtColor(img[i], grey[i], CV_BGR2GRAY);
    }

    cvSet(disp, cvScalar(0));
    census.setImages(grey[0], grey[1]);
    census.match();
    //census.printTiming();
    census.getDisparityMap(disp);

    cvShowImage("grabstereo - left", grey[0]);
    cvShowImage("grabstereo - right", grey[1]);
    cvShowImage("grabstereo - disparity", disp);

    switch(cvWaitKey(10))
    {
      case ESCAPE:
      case 'q':
        done = 1;
        break;
      case 'g':
          //SaveCurrentImages();
          break;
      default:
        break;
    }
  }

  cvReleaseCapture(&capture[0]);
  cvReleaseCapture(&capture[1]);
  cvDestroyWindow("grabstereo - left");
  cvDestroyWindow("grabstereo - right");
  cvDestroyWindow("grabstereo - disparity");
  cvReleaseImage(&grey[0]);
  cvReleaseImage(&grey[1]);
  cvReleaseImage(&disp);

  exit(0);
}

