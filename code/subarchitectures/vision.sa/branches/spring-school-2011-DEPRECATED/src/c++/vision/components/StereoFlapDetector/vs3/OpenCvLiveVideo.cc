/**
 * $Id: OpenCvLiveVideo.cc,v 1.1 2007/03/25 21:36:26 mxz Exp mxz $
 */

#include "OpenCvLiveVideo.hh"

namespace Z
{

OpenCvLiveVideo::OpenCvLiveVideo(int w, int h, int cam, int device_class)
  throw(Except)
{
  frame_cnt = 0;
  cur_frame = 0;
  capture = cvCaptureFromCAM(device_class + cam);
  if(capture == 0)
    throw Except(__HERE__, "failed to init capture");
  // try setting size (might fail if not supported)
  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, w);
  cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, h);
  // get first image (so we know size etc.)
  Forward();
}

OpenCvLiveVideo::~OpenCvLiveVideo()
{
  cvReleaseCapture(&capture);
}

string OpenCvLiveVideo::DeviceInfo()
{
  return string("OpenCV live capture");
}

int OpenCvLiveVideo::Width()
{
  return CurrentFramePtr()->width;
}

int OpenCvLiveVideo::Height()
{
  return CurrentFramePtr()->height;
}

int OpenCvLiveVideo::BytesPerPixel()
{
  return 3;
}

int OpenCvLiveVideo::BytesPerLine()
{
  return CurrentFramePtr()->widthStep;
}

ColorFormat OpenCvLiveVideo::ColorFmt()
{
  return RGB24;
}

bool OpenCvLiveVideo::IsLive()
{
  return true;
}

/**
 * Note: A live video guarantees that there is always a current frame.
 */
const IplImage* OpenCvLiveVideo::CurrentFramePtr()
{
  return cur_frame;
}

int OpenCvLiveVideo::FrameCount()
{
  return frame_cnt;
}

string OpenCvLiveVideo::FrameName()
{
  char buf[100];
  snprintf(buf, 100, "live video, frame %d", frame_cnt);
  return string(buf);
}

bool OpenCvLiveVideo::Forward()
{
  cvGrabFrame(capture);
  // note: cvRetrieve() might call various colour conversion! So do it only
  // once and remember returned pointer.
  cur_frame = cvRetrieveFrame(capture);
  frame_cnt++;
  return true;
}

}

