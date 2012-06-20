/**
 * $Id: OpenCvImgSeqVideo.cc,v 1.6 2006/11/24 13:47:03 mxz Exp mxz $
 *
 * Michael Zillich 2004-03-07
 */

#include <opencv/highgui.h>
#include "OpenCvImgSeqVideo.hh"
#include <limits.h>
#include <stdio.h>

namespace Z
{

/**
 * Construct empty, files will have to be add with AddFrame().
 */
OpenCvImgSeqVideo::OpenCvImgSeqVideo()
{
  img = 0;
  frame_count = INT_MAX;
}

/**
 * Construct with a list of filenames.
 */
OpenCvImgSeqVideo::OpenCvImgSeqVideo(const vector<string> &files)
{
  img = 0;
  frame_count = INT_MAX;
  image_files = files;
  MoveToStart();
}

/**
 * Construct with a filename template and frame numbers.
 * \param file  (in) printf-style template string for filename,
 *                   e.g. "data/img%02d.jpg" for data/img00.jpg, data/img01.jpg
 * \param first (in) first frame number, to start e.g. with data/img04.jpg
 * \param last  (in) last frame number
 * \param inc   (in) frame number increment
 */
OpenCvImgSeqVideo::OpenCvImgSeqVideo(const char *file_template, int first,
    int last, int inc)
{
  img = 0;
  frame_count = INT_MAX;
  ConstructFilenames(file_template, first, last, inc);
  MoveToStart();
}

OpenCvImgSeqVideo::~OpenCvImgSeqVideo()
{
  cvReleaseImage(&img);
}

void OpenCvImgSeqVideo::ConstructFilenames(const char *file_template, int first,
    int last, int inc)
{
  char filename[1024];
  // normal order: first < last, positive increment
  // (note first = last is a special case: just one image)
  if(first <= last)
  {
    // if no increment given, assume default
    if(inc == 0)
      inc = 1;
    // just in case we were given a stupid increment
    else if(inc < 0)
      inc = -inc;
    for(int i = first; i <= last; i += inc)
    {
      snprintf(filename, 1024, file_template, i);
      image_files.push_back(filename);
    }
  }
  // reverse order (to run a movie backwards): first > last, negative increment
  else
  {
    if(inc == 0)
      inc = -1;
    else if(inc > 0)
      inc = -inc;
    for(int i = first; i >= last; i += inc)
    {
      snprintf(filename, 1024, file_template, i);
      image_files.push_back(filename);
    }
  }
}

/**
 * Load an image.
 * Returns true if image was loaded successfully.
 */
bool OpenCvImgSeqVideo::LoadImage(int frame) throw(Except)
{
  cvReleaseImage(&img);
  if(frame < 0 || frame >= (int)image_files.size())
    throw Except(__HERE__, "frame %u is out of range 0..%u",
        frame, image_files.size());
  // note: we always want colour images
  img = cvLoadImage(image_files[frame].c_str(), CV_LOAD_IMAGE_COLOR);
  if(img == 0)
    throw Except(__HERE__, "failed to open image '%s'",
        image_files[frame].c_str());
  return true;
}

/**
 * Loads the next image.
 * Returns true if image was loaded successfully.
 * If at end of sequence keep the last image and return false.
 */
bool OpenCvImgSeqVideo::LoadNextImage()
{
  // if not at end of sequence
  if(frame_count + 1 < (int)image_files.size())
  {
    frame_count++;
    LoadImage(frame_count);
    return true;
  }
  // if at end of sequence, do nothing (keep last image)
  // frame_count remains image_files.size() - 1
  else
    return false;
}

/**
 * Loads the previous image.
 * Returns true if image was loaded successfully.
 * If at start of sequence keep the last image and return false.
 */
bool OpenCvImgSeqVideo::LoadPrevImage()
{
  // if not at end of sequence
  if(frame_count > 0)
  {
    frame_count--;
    LoadImage(frame_count);
    return true;
  }
  // if at start of sequence, do nothing (keep first image)
  // frame_count remains 0
  else
    return false;
}

string OpenCvImgSeqVideo::DeviceInfo()
{
  return string("File Video using Imlib");
}

int OpenCvImgSeqVideo::Width()
{
  if(img != 0)
    return img->width;
  else
    return 0;
}

int OpenCvImgSeqVideo::Height()
{
  if(img != 0)
    return img->height;
  else
    return 0;
}

int OpenCvImgSeqVideo::BytesPerPixel()
{
  return 3;
}

int OpenCvImgSeqVideo::BytesPerLine()
{
  if(img != 0)
    return img->widthStep;
  else
    return 0;
}

ColorFormat OpenCvImgSeqVideo::ColorFmt()
{
  return RGB24;
}

/**
 * Note: This pointer can be NULL, in case no images have been loaded yet.
 */
const IplImage* OpenCvImgSeqVideo::CurrentFramePtr()
{
  return img;
}

bool OpenCvImgSeqVideo::IsLive()
{
  return false;
}

bool OpenCvImgSeqVideo::IsBuffer()
{
  return false;
}

/**
 * Returns the current frame number, starting at 0.
 */
int OpenCvImgSeqVideo::FrameCount()
{
  return frame_count;
}

/**
 * Returns the filename of the current frame
 */
string OpenCvImgSeqVideo::FrameName()
{
  if(frame_count < (int)image_files.size())
    return image_files[frame_count];
  else
    return string("no frame loaded");
}

/**
 * Load the next image.
 * Returns true if the next image was successfully loaded, false if at end of
 * sequence, in which case the last image remains loaded.
 */
bool OpenCvImgSeqVideo::Forward()
{
  return LoadNextImage();
}

bool OpenCvImgSeqVideo::Backward()
{
  return LoadPrevImage();
}

void OpenCvImgSeqVideo::ClearFrames()
{
  cvReleaseImage(&img);
  frame_count = INT_MAX;
  image_files.clear();
}

void OpenCvImgSeqVideo::AddFrame(const string &filename)
{
  bool first_frame = image_files.empty();
  image_files.push_back(filename);
  if(first_frame)
    MoveToStart();
}

void OpenCvImgSeqVideo::AddFrames(const vector<string> &filenames)
{
  bool first_frames = image_files.empty();
  for(size_t i = 0; i < filenames.size(); i++)
    image_files.push_back(filenames[i]);
  if(first_frames)
    MoveToStart();
}

void OpenCvImgSeqVideo::ReplaceFrames(const vector<string> &filenames)
{
  ClearFrames();
  AddFrames(filenames);
}

/**
 * Loads first image (e.g. after construction).
 * Resets frame counter to 0.
 * @return true if moved, false if already at start
 */
bool OpenCvImgSeqVideo::MoveToStart()
{
  if(image_files.size() > 0)
  {
    frame_count = 0;
    LoadImage(frame_count);
    return true;
  }
  else
    return false;
}

/**
 * Loads last image (e.g. after adding a frame)
 * Sets frame counter to size - 1.
 * @return true if moved, false if already at end
 */
bool OpenCvImgSeqVideo::MoveToEnd()
{
  if(image_files.size() > 0)
  {
    frame_count = image_files.size() - 1;
    LoadImage(frame_count);
    return true;
  }
  else
    return false;
}

}

