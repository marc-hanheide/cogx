/**
 * $Id: BufferVideo.cc,v 1.1 2006/11/24 13:55:13 mxz Exp mxz $
 *
 * @author Michael Zillich
 * @date September 2006
 */

#include "BufferVideo.hh"
#include <string.h>

namespace Z
{

BufferVideo::BufferVideo()
{
  width = height = bytes_per_pixel = bytes_per_line = 0;
  buffer = 0;
  own = false;
  frame_cnt = 0;
}

BufferVideo::~BufferVideo()
{
  if(own)
    delete[] buffer;
}

void BufferVideo::Init(unsigned w, unsigned h, ColorFormat fmt, char *buf,
    bool copy) throw(Except)
{
  width = w;
  height = h;
  col_fmt = fmt;
  if(col_fmt == RGB24 || col_fmt == BGR24)
    bytes_per_pixel = 3;
  else
    throw (__HERE__, "invalid color format, must be RGB24 or BGR24");
  bytes_per_line = width*bytes_per_pixel;
  if(copy)
  {
    buffer = new char[width*height*bytes_per_pixel];
    memcpy(buffer, buf, width*height*bytes_per_pixel);
    own = true;
  }
  else
  {
    buffer = buf;
    own = false;
  }
  frame_cnt = 0;
}

/**
 * Set a new buffer, assuming that image size and format do not change
 */
void BufferVideo::SetBuffer(char *buf)
{
  if(own)
    memcpy(buffer, buf, width*height*bytes_per_pixel);
  else
    buffer = buf;
}

const char* BufferVideo::DeviceInfo()
{
  return "Memory Buffer Video";
}

unsigned BufferVideo::Width()
{
  return width;
}

unsigned BufferVideo::Height()
{
  return height;
}

unsigned BufferVideo::BytesPerPixel()
{
  return bytes_per_pixel;
}

unsigned BufferVideo::BytesPerLine()
{
  return bytes_per_line;
}

ColorFormat BufferVideo::ColorFmt()
{
  return col_fmt;
}

bool BufferVideo::IsLive()
{
  return false;
}

char* BufferVideo::CurrentFramePtr()
{
  return buffer;
}

unsigned BufferVideo::FrameCount()
{
  return frame_cnt;
}

const char *BufferVideo::FrameName()
{
  return "buffer";
}
  
bool BufferVideo::Forward()
{
  // do nothing, we ever just read whatever is in the buffer
  frame_cnt++;
}

}

