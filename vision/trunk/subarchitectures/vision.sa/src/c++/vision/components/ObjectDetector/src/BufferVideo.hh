/**
 * $Id: BufferVideo.hh,v 1.1 2006/11/24 13:54:59 mxz Exp mxz $
 *
 * @author Michael Zillich
 * @date September 2006
 */

#ifndef Z_BUFFER_VIDEO_HH
#define Z_BUFFER_VIDEO_HH

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
  unsigned width;
  unsigned height;
  unsigned bytes_per_pixel;
  unsigned bytes_per_line;
  unsigned frame_cnt;
  ColorFormat col_fmt;
  char *buffer;
  bool own;

public:
  BufferVideo();
  virtual ~BufferVideo();
  void Init(unsigned w, unsigned h, ColorFormat fmt, char *buf, bool copy)
    throw(Except);
  void SetBuffer(char *buf);
  // common methods
  virtual const char* DeviceInfo();
  virtual unsigned Width();
  virtual unsigned Height();
  virtual unsigned BytesPerPixel();
  virtual unsigned BytesPerLine();
  virtual ColorFormat ColorFmt();
  virtual bool IsLive();
  virtual char* CurrentFramePtr();
  virtual unsigned FrameCount();
  virtual const char *FrameName();
  virtual bool Forward();
};

}

#endif