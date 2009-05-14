#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <opencv/cv.h>
#include <opencv/highgui.h>


#define RETURN_OK                 0
#define ERROR                    -1
#define ERROR_OPENING_DEVICE     -2
#define ERROR_DEVICE_NOT_OPEN    -3
#define ERROR_NO_PIC_PARAM       -4
#define ERROR_FRAMERATE_NOT_SET  -5
#define ERROR_MMAP_UNAVAILABLE   -6
#define ERROR_CAPTURING_FRAME    -7
#define ERROR_SYNC_BUFFER        -8
#define ERROR_NO_VIDEO_BUFFER    -9

#define ERROR_LOAD_IMAGE        -10


#define ERROR_NOT_IN_MAP          -100
#define ERROR_VECTOR_OUTOFBOUND   -101

void printError( long _errorCode );

void imshow(IplImage* image,const char * caption="temp");

#endif
