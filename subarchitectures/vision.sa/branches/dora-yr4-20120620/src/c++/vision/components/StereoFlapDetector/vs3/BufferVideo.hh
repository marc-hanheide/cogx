/**
 * $Id: BufferVideo.hh,v 1.1 2006/11/24 13:54:59 mxz Exp mxz $
 *
 * @author Michael Zillich
 * @date September 2006
 */

#ifndef Z_BUFFER_VIDEO_HH
#define Z_BUFFER_VIDEO_HH

#include <opencv/cv.h>
#include "Except.hh"
#include "Video.hh"

namespace Z
{

/**
 * Video from buffer in memory, being filled by some other source.
 */
class BufferVideo : public Video
{
private:
  bool have_init;
  unsigned width;
  unsigned height;
  unsigned bytes_per_pixel;
  unsigned bytes_per_line;
  unsigned frame_cnt;
  ColorFormat col_fmt;
  char *buffer;
  IplImage *img;
  bool own;

public:
  BufferVideo();
  virtual ~BufferVideo();
  void Init(unsigned w, unsigned h, ColorFormat fmt, bool own_buffer)
    throw(Except);
  void Init(unsigned w, unsigned h, ColorFormat fmt, char *buf, bool copy);
  void SetBuffer(char *buf);
  // common methods
  virtual string DeviceInfo();
  virtual int Width();
  virtual int Height();
  virtual int BytesPerPixel();
  virtual int BytesPerLine();
  virtual ColorFormat ColorFmt();
  virtual bool IsLive();
  virtual const IplImage* CurrentFramePtr();
  virtual int FrameCount();
  virtual string FrameName();
  virtual bool Forward();
};

}

#endif
