/**
 * @file Video.hh
 * @author Zillich
 * @date March 2007
 * @version 0.1
 * @brief Abstract video base class.
 **/

#ifndef Z_VIDEO_HH
#define Z_VIDEO_HH

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include "Color.hh"

namespace Z
{

enum VideoType
{
// 	VIDEO_TYPE_IMG_SEQ,
  //VIDEO_TYPE_FILESEQ,
//   VIDEO_TYPE_BUFFER,
// 	VIDEO_TYPE_LIVE,
//   VIDEO_TYPE_FIREWIRE,
//   VIDEO_TYPE_OPENCV

  VIDEO_TYPE_BUFFER,   ///< somebody provides a buffer, which we wrap
  VIDEO_TYPE_IMG_SEQ,  ///< read a sequence of individual image files
  VIDEO_TYPE_FILE,     ///< read a video file
  VIDEO_TYPE_LIVE      ///< live video
};

/**
 * @brief Abstract video base class.
 */
class Video
{
public:
//   virtual ~Video() {};
//   virtual const char* DeviceInfo() = 0;
//   virtual unsigned Width() = 0;
//   virtual unsigned Height() = 0;
//   virtual unsigned BytesPerPixel() = 0;
//   virtual unsigned BytesPerLine() = 0;
//   virtual ColorFormat ColorFmt() = 0;
//   virtual bool IsLive() = 0;
//   virtual char* CurrentFramePtr() = 0;
//   virtual unsigned FrameCount() = 0;
//   virtual const char *FrameName() = 0;
//   virtual bool Forward() = 0;

  virtual ~Video() {};
  // information about the video device
  virtual string DeviceInfo() = 0;
  virtual int Width() = 0;
  virtual int Height() = 0;
  virtual int BytesPerPixel() = 0;
  virtual int BytesPerLine() = 0;
  virtual ColorFormat ColorFmt() = 0;
  virtual bool IsLive() = 0;
  virtual bool IsBuffer() = 0;
  // get current frame and frame info
  virtual const IplImage* CurrentFramePtr() = 0;
  virtual int FrameCount() = 0;
  virtual string FrameName() = 0;
  // move forward/backward (if supported)
  virtual bool Forward() = 0;
  virtual bool Backward() {return false;}
  virtual bool MoveToStart() {return false;}
  virtual bool MoveToEnd() {return false;}
  // modify frames (if supported)
  virtual void ClearFrames() {}
  virtual void AddFrame(const string &filename) {}
  virtual void AddFrames(const vector<string> &filenames) {}
  virtual void ReplaceFrames(const vector<string> &filenames) {}
};

}

#endif

