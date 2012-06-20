/**
 * $Id$
 */

#ifndef Z_OPENCV_BUFFER_VIDEO_HH
#define Z_OPENCV_BUFFER_VIDEO_HH

#include <opencv/highgui.h>
#include "Except.hh"
#include "Video.hh"

namespace Z
{

/**
 * Video from buffer in memory, being filled by some other source.
 */
class OpenCvBufferVideo : public Video
{
private:
  int frame_cnt;
  bool own;
  IplImage *image;
  

public:
  OpenCvBufferVideo();
  virtual ~OpenCvBufferVideo();
  void Init(IplImage *img, bool copy)
    throw(Except);
  void SetBuffer(IplImage *img);
  // common methods
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
