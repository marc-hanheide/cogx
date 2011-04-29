/**
 * $Id: BufferVideo.cc,v 1.1 2006/11/24 13:55:13 mxz Exp mxz $
 *
 * @author Michael Zillich
 * @date September 2006
 */

#include "BufferVideo.hh"

namespace Z
{

BufferVideo::BufferVideo()
{
  width = height = bytes_per_pixel = bytes_per_line = 0;
  buffer = 0;
  img = 0;
  own = false;
  frame_cnt = 0;
  have_init = false;
}

BufferVideo::~BufferVideo()
{
  cvReleaseImageHeader(&img);
  if(own)
    delete[] buffer;
}

void BufferVideo::Init(unsigned w, unsigned h, ColorFormat fmt, bool own_buffer)
  throw(Except)
{
  if(have_init)
    throw Except(__HERE__, "Init() may only be called once");

  width = w;
  height = h;
  col_fmt = fmt;
  if(col_fmt == RGB24 || col_fmt == BGR24)
    bytes_per_pixel = 3;
  else
    throw Except(__HERE__, "invalid color format, must be RGB24 or BGR24");
  bytes_per_line = width*bytes_per_pixel;
  if(own_buffer)
  {
    buffer = new char[width*height*bytes_per_pixel];
    own = true;
  }
  else
  {
    buffer = 0;
    own = false;
  }
  img = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
  // Note: if !own then imageData now points to 0, only after SetBuffer()
  // imageData is valid.
  img->imageData = buffer;
}

void BufferVideo::Init(unsigned w, unsigned h, ColorFormat fmt, char *buf,
    bool copy)
{
  Init(w, h, fmt, copy);
  SetBuffer(buf);
}

/**
 * Set a new buffer, assuming that image size and format do not change
 */
void BufferVideo::SetBuffer(char *buf)
{
  if(own)
    memcpy(buffer, buf, width*height*bytes_per_pixel);
  else
  {
    buffer = buf;
    img->imageData = buffer;
  }
}

string BufferVideo::DeviceInfo()
{
  return "Memory Buffer Video";
}

int BufferVideo::Width()
{
  return (int)width;
}

int BufferVideo::Height()
{
  return (int)height;
}

int BufferVideo::BytesPerPixel()
{
  return (int)bytes_per_pixel;
}

int BufferVideo::BytesPerLine()
{
  return (int)bytes_per_line;
}

ColorFormat BufferVideo::ColorFmt()
{
  return col_fmt;
}

bool BufferVideo::IsLive()
{
  return false;
}

const IplImage* BufferVideo::CurrentFramePtr()
{
  return img;
}

int BufferVideo::FrameCount()
{
  return frame_cnt;
}

string BufferVideo::FrameName()
{
  return "buffer";
}
  
bool BufferVideo::Forward()
{
  // do nothing, we ever just read whatever is in the buffer
  frame_cnt++;
  return true;
}

}

