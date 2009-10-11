/**
 * $Id: OpenCvLiveVideo.hh,v 1.1 2007/03/25 21:36:26 mxz Exp mxz $
 *
 */

#ifndef Z_OPENCV_LIVE_VIDEO_HH
#define Z_OPENCV_LIVE_VIDEO_HH

#include <opencv/highgui.h>
#include "Except.hh"
#include "Video.hh"

namespace Z
{

/**
 * Use the convenient OpenCV video capture.
 */
class OpenCvLiveVideo : public Video
{
private:
  static const int DEF_WIDTH = 640;
  static const int DEF_HEIGHT = 480;
  CvCapture* capture;
  IplImage* cur_frame;
  int frame_cnt;

public:
  OpenCvLiveVideo(int w = DEF_WIDTH, int h = DEF_HEIGHT, int cam = 0)
      throw(Except);
  virtual ~OpenCvLiveVideo();
  virtual string DeviceInfo();
  virtual int Width();
  virtual int Height();
  virtual int BytesPerPixel();
  virtual int BytesPerLine();
  virtual ColorFormat ColorFmt();
  virtual bool IsLive();
  virtual bool IsBuffer();
  virtual const IplImage* CurrentFramePtr();
  virtual int FrameCount();
  virtual string FrameName();
  virtual bool Forward();
};

}

#endif


