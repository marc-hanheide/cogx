/**
 * $Id: Video.hh,v 1.6 2007/03/03 10:12:24 mxz Exp mxz $
 */

#ifndef Z_VIDEO_HH
#define Z_VIDEO_HH

#include <string>
#include <vector>
#include <opencv/cv.h>
#include "Color.hh"

namespace Z
{

enum VideoType
{
  VIDEO_TYPE_BUFFER,   // somebody provides a buffer, which we wrap
  VIDEO_TYPE_IMG_SEQ,  // read a sequence of individual image files
  VIDEO_TYPE_FILE,     // read a video file
  VIDEO_TYPE_LIVE      // live video
};

/**
 * Abstract video base class.
 */
class Video
{
public:
  virtual ~Video() {};
  // information about the video device
  virtual string DeviceInfo() = 0;
  virtual int Width() = 0;
  virtual int Height() = 0;
  virtual int BytesPerPixel() = 0;
  virtual int BytesPerLine() = 0;
  virtual ColorFormat ColorFmt() = 0;
  virtual bool IsLive() = 0;
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

