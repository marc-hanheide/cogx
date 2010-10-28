/**
 * $Id$
 *
 * @author Michael Zillich
 * @date September 2006
 */

#include "OpenCvBufferVideo.hh"
#include <iostream>

namespace Z
{

OpenCvBufferVideo::OpenCvBufferVideo()
{
  own = false;
  frame_cnt = 0;
  image = 0;
}

OpenCvBufferVideo::~OpenCvBufferVideo()
{
  if (image!=0 && own==true)
    cvReleaseImage(&image);
}

void OpenCvBufferVideo::Init(IplImage *img, bool copy) throw(Except)
{
  if(img->nChannels!=3)
    throw Except(__HERE__, "invalid color format, must be RGB24 or BGR24");
  if(copy)
  {
    image = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
    cvCopy(img, image);
    own = true;
  }
  else
  {
    image = img;
    own = false;
  }
  frame_cnt = 0;
}

/**
 * Set a new buffer, assuming that image size and format do not change
 */
void OpenCvBufferVideo::SetBuffer(IplImage *img)
{
  if(own)
    cvCopy (img, image);
  else
    image = img;
}

string OpenCvBufferVideo::DeviceInfo()
{
  return string("Memory Buffer Video");
}

int OpenCvBufferVideo::Width()
{
  if (image!=0)
    return image->width;
  return 0;
}

int OpenCvBufferVideo::Height()
{
  if (image!=0)
    return image->height;
  return 0;
}

int OpenCvBufferVideo::BytesPerPixel()
{
  return 3;
}

int OpenCvBufferVideo::BytesPerLine()
{
  if(image != 0)
    return image->widthStep;
  return 0;
}

ColorFormat OpenCvBufferVideo::ColorFmt()
{
  return RGB24;
// 	return BGR24;
}

bool OpenCvBufferVideo::IsLive()
{
  return false;
}

bool OpenCvBufferVideo::IsBuffer()
{
  return true;
}

const IplImage* OpenCvBufferVideo::CurrentFramePtr()
{
  return image;
}

int OpenCvBufferVideo::FrameCount()
{
  return frame_cnt;
}

string OpenCvBufferVideo::FrameName()
{
  return string("buffer");
}
  
bool OpenCvBufferVideo::Forward()
{
  // do nothing, we ever just read whatever is in the buffer
  frame_cnt++;
}

}

