/**
 * $Id: OpenCvImgSeqVideo.hh,v 1.5 2006/11/24 13:47:03 mxz Exp mxz $
 *
 * Michael Zillich 2007-11-17
 */

#ifndef Z_OPENCV_IMG_SEQ_VIDEO_HH
#define Z_OPENCV_IMG_SEQ_VIDEO_HH

#include <vector>
#include <string>
#include <opencv/cv.h>
#include "Except.hh"
#include "Video.hh"

namespace Z
{

/**
 * Video from stored image files.
 */
class OpenCvImgSeqVideo : public Video
{
private:
  vector<string> image_files;
  int frame_count;
  IplImage *img;

  void ConstructFilenames(const char *file_template, int first, int last,
      int inc);
  bool LoadImage(int frame) throw(Except);
  bool LoadNextImage();
  bool LoadPrevImage();

public:
  OpenCvImgSeqVideo();
  OpenCvImgSeqVideo(const vector<string> &files);
  OpenCvImgSeqVideo(const char *file_template, int first, int last,
      int inc = 0);
  virtual ~OpenCvImgSeqVideo();
  virtual string  DeviceInfo();
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
  virtual bool Backward();
  virtual void ClearFrames();
  virtual void AddFrame(const string &filename);
  virtual void AddFrames(const vector<string> &filenames);
  virtual void ReplaceFrames(const vector<string> &filenames);
  virtual bool MoveToStart();
  virtual bool MoveToEnd();
};

}

#endif

