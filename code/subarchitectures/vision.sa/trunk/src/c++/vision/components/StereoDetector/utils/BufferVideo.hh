/**
 * @file BufferVideo.hh
 * @author Michael Zillich, Richtsfeld Andreas
 * @date 2006, 2010
 * @version 0.1
 * @brief Video from buffer in memory, being filled by some other source.
 **/

#ifndef Z_BUFFER_VIDEO_HH
#define Z_BUFFER_VIDEO_HH

#include <opencv/cv.h>
#include "Except.hh"
#include "Video.hh"

namespace Z
{

/**
 * @brief Video from buffer in memory, being filled by some other source.
 */
class BufferVideo : public Video
{
private:
  bool have_init;							///< BufferVideo already initialised. Only once possible.
  unsigned width;							///< Width of image
  unsigned height;						///< Height of image
  unsigned bytes_per_pixel;		///< Bytes per pixel
  unsigned bytes_per_line;		///< Bytes per line
  unsigned frame_cnt;					///< Frame counter
  ColorFormat col_fmt;				///< Color format
  char *buffer;								///< buffer for video data
  IplImage *img;							///< OpenCV IplImage
  bool own;										///< Own buffer for video data

public:
  BufferVideo();
  virtual ~BufferVideo();
  void Init(unsigned w, unsigned h, ColorFormat fmt, bool own_buffer) throw(Except);
  void Init(unsigned w, unsigned h, ColorFormat fmt, char *buf, bool copy);
  void SetBuffer(char *buf);

  // common methods
  virtual string DeviceInfo() {return "Memory Buffer Video";}		///< Return device info
  virtual int Width() {return (int)width;}											///< Return width
  virtual int Height() {return (int)height;}										///< Return height
  virtual int BytesPerPixel() {return (int)bytes_per_pixel;}		///< Return bytes per pixel
  virtual int BytesPerLine() {return (int)bytes_per_line;}			///< Return bytes per line
  virtual ColorFormat ColorFmt() {return col_fmt;}							///< Return color format
  virtual bool IsLive() {return false;}													///< Return if alive
  virtual const IplImage* CurrentFramePtr() {return img;}				///< Return current frame buffer
  virtual int FrameCount() {return frame_cnt;}									///< Return frame counter
  virtual string FrameName() {return "buffer";}									///< Return frame name
  virtual bool Forward() {frame_cnt++; return true;}						///< Forward frame counter
};

}

#endif
