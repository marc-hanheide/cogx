#include "siftGlobal.h"
#include <iostream>

using namespace std;

void imshow(IplImage* image,const char * caption) {
  cvvNamedWindow(caption,1);
  cvvShowImage(caption,image);
}

void printError( long _errorCode )
{
	switch ( _errorCode )
	{
	case ERROR:
		cerr << "An unknown error has occured ..."
		     << endl;
		break;
	case ERROR_OPENING_DEVICE:
		cerr << "Could not open the specified device ..."
		     << endl;
		break;
	case ERROR_DEVICE_NOT_OPEN:
		cerr << "Device not open  ..."
		     << endl;
		break;
	case ERROR_NO_PIC_PARAM:
		cerr << "Failed to get picture parameters  ..."
		     << endl;
		break;
	case ERROR_FRAMERATE_NOT_SET:
		cerr << "Failed to set the framerate   ..."
		     << endl;
		break;
	case ERROR_MMAP_UNAVAILABLE:
		cerr << "Failed to create the mmap  ..."
		     << endl;
		break;
	case ERROR_CAPTURING_FRAME:
		cerr << "Error capturing a frame  ..."
		     << endl;
		break;
	case ERROR_SYNC_BUFFER:
		cerr << "Failed to synchronize the buffer  ..."
		     << endl;
		break;
	case ERROR_NO_VIDEO_BUFFER:
		cerr << "Failed to allocate a video buffer  ..."
		     << endl;
		break;


	case ERROR_LOAD_IMAGE:
		cerr << "Failed to load image  ..."
		     << endl;
		break;
	

	case ERROR_NOT_IN_MAP:
	  cerr << "Failed to locate item in a map  ..."
	       << endl;
	  break;	


	case ERROR_VECTOR_OUTOFBOUND:
	  cerr << "Failed <vector> out of bound  ..."
	       << endl;
	  break;


	}
}
