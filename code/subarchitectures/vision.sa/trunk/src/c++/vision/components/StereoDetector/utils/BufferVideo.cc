/**
 * @file BufferVideo.cc
 * @author Michael Zillich, Richtsfeld Andreas
 * @date 2006, 2010
 * @version 0.1
 * @brief Video from buffer in memory, being filled by some other source.
 **/

#include "BufferVideo.hh"

namespace Z
{

/**
 * @brief Constructor of class BufferVideo.
 */
BufferVideo::BufferVideo()
{
  width = height = bytes_per_pixel = bytes_per_line = 0;
  buffer = 0;
  img = 0;
  own = false;
  frame_cnt = 0;
  have_init = false;
}

/**
 * @brief Destructor of class BufferVideo.
 */
BufferVideo::~BufferVideo()
{
  cvReleaseImageHeader(&img);
  if(own)
    delete[] buffer;
}

/**
 * @brief Initialise BufferVideo class.
 * @param w Width of image.
 * @param h Height of image.
 * @param fmt Color format.
 * @param own_buffer Use own buffer or not.
 */
void BufferVideo::Init(unsigned w, unsigned h, ColorFormat fmt, bool own_buffer) throw(Except)
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

/**
 * @brief Initialise BufferVideo class
 * @param w Width of image.
 * @param h Height of image.
 * @param fmt Color format.
 * @param buf Video buffer.
 * @param copy Copy image to buffer.
 */
void BufferVideo::Init(unsigned w, unsigned h, ColorFormat fmt, char *buf, bool copy)
{
  Init(w, h, fmt, copy);
  SetBuffer(buf);
}

/**
 * @brief Set a new buffer, assuming that image size and format do not change.
 * @param buf Video buffer
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

}

