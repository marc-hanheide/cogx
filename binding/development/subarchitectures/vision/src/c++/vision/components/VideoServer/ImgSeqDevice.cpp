/**
 * @author Michael Zillich
 * @date October 2006
 */

#include <cstring>
#include <algorithm>
#include <opencv/highgui.h>
#include <vision/utils/VisionUtils.h>
#include "ImgSeqDevice.h"

using namespace Vision;

// default framerate if none specified is 1 s
#define FRAMERATE_DEFAULT 1000

ImgSeqDevice::ImgSeqDevice()
{
  framerate_ms = -1;
  frame_cnt = -1;
}

/**
 * Construct with a list of filenames.
 * If there are several camera ids, file names must be arranged as:
 * frame0/cam0, frame0/cam1, frame1/cam0, frame1/cam1, frame2/cam0, frame2/cam1, ...
 * Note that file names need not follow any particular pattern.
 * \param ids  (in) camera ids, e.g. 0, 1 or 2, 3
 * \param rate_ms  (in) frame rate in milli seconds
 */
ImgSeqDevice::ImgSeqDevice(const vector<int> &ids,
    const vector<string> &files, int rate_ms)
{
  if(ids.size() == 0 || files.size() == 0)
    throw BALTException(__HERE__, "need at least one camera");
  if(ids.size() % files.size() != 0)
    throw BALTException(__HERE__, "number of cameras does not match file list");
  Init(ids, rate_ms);
  filenames = files;
}

/**
 * Construct with a filename template and frame numbers.
 * If there are several camera ids, each camera has its own file template,
 * e.g. img_left_%03d.jpg img_right_%03d.jpg
 * \param ids  (in) camera ids, e.g. 0, 1 or 2, 3
 * \param file  (in) printf-style template string for filename,
 *                   e.g. "data/img%02d.jpg" for data/img00.jpg, data/img01.jpg
 * \param first (in) first frame number, to start e.g. with data/img04.jpg
 * \param last  (in) last frame number
 * \param rate_ms  (in) frame rate in milli seconds
 * \param inc   (in) frame number increment
 */
ImgSeqDevice::ImgSeqDevice(const vector<int> &ids,
    const vector<string> &file_templates,
    int first, int last, int rate_ms, int inc)
{
  if(ids.size() == 0 || file_templates.size() == 0)
    throw BALTException(__HERE__, "need at least one camera");
  if(ids.size() != file_templates.size())
    throw BALTException(__HERE__, "number of cameras does not match file list");
  Init(ids, rate_ms);
  ConstructFilenames(file_templates, first, last, inc);
}

void ImgSeqDevice::Init(const vector<int> &ids, int rate_ms)
{
  cam_ids = ids;
  if(rate_ms > 0)
    framerate_ms = rate_ms;
  else
    framerate_ms = FRAMERATE_DEFAULT;
  times.resize(cam_ids.size());
  images.resize(cam_ids.size());
  for(unsigned i = 0; i < images.size(); i++)
    images[i] = 0;
  frame_cnt = 0;
}

/**
 * Construct filenames from file templates.
 * Each camera has a file template, e.g. img_left_%03d.jpg img_right_%03d.jpg
 */
void ImgSeqDevice::ConstructFilenames(const vector<string> &file_templates,
    int first, int last, int inc)
{
  char filename[1024];

  if(first <= last)
  {
    if(inc <= 0)  // ensure inc is positive
      inc = 1;
    for(int i = first; i <= last; i += inc)
      for(unsigned c = 0; c < file_templates.size(); c++)
      {
        snprintf(filename, 1024, file_templates[c].c_str(), i);
        filenames.push_back(filename);
      }
  }
  else // first > last
  {
    if(inc >= 0)  // ensure inc is negative
      inc = -1;
    for(int i = last; i >= first; i += inc)
      for(unsigned c = 0; c < file_templates.size(); c++)
      {
        snprintf(filename, 1024, file_templates[c].c_str(), i);
        filenames.push_back(filename);
      }
  }
}

void ImgSeqDevice::GrabFrames()
{
  // number of current frame, note that we loop
  int fn = frame_cnt % NumImages();
  BALTTime time;

  if(filenames.size() == 0)
    throw BALTException(__HERE__, "video not initialised");

  // note that by just calling sleep(framrate) we actually do not really
  // get a fixed framerate, this would require setitimer() and pause()
  usleep(1000*framerate_ms);

  for(unsigned i = 0; i < images.size(); i++)
  {
    cvReleaseImage(&images[i]);
    images[i] = cvLoadImage(filenames[fn*NumCameras() + i].c_str(),
        CV_LOAD_IMAGE_COLOR);
    if(images[i] == 0)
      throw BALTException(__HERE__, "failed load image '%s'",
          filenames[fn*NumCameras() + i].c_str());
  }
  time = BALTTimer::getBALTTime();
  for(unsigned i = 0; i < times.size(); i++)
    times[i] = time;
  frame_cnt++;
}

/**
 * Use only after GrabFrames
 */
void ImgSeqDevice::RetrieveFrames(vector<ImageFrame*> &frames)
{
  if(frames.size() != (unsigned)NumCameras())
    throw BALTException(__HERE__, "requested %d images from %d cameras",
        frames.size(), NumCameras());
  // To handle the case where Retrieve is called before any Grab
  if(!HaveFrames())
    GrabFrames();
  for(unsigned i = 0; i < images.size(); i++)
  {
    if(frames[i] == 0)
      throw BALTException(__HERE__, "provided frame pointer is NULL");
    ConvertFrame(images[i], frames[i]);
  }
  for(unsigned i = 0; i < images.size(); i++)
  {
    frames[i]->m_time = times[i];
    frames[i]->m_camNum = cam_ids[i];
  }
}

/**
 * Use only after GrabFrames
 */
void ImgSeqDevice::RetrieveFrame(int camNum, Vision::ImageFrame *frame)
{
  bool haveCam = false;
  if(frame == 0)
    throw BALTException(__HERE__, "provided frame pointer is NULL");
  // To handle the case where Retrieve is called before any Grab
  if(!HaveFrames())
    GrabFrames();
  for(unsigned i = 0; i < images.size(); i++)
  {
    if(camNum == cam_ids[i])
    {
      ConvertFrame(images[i], frame);
      frame->m_time = times[i];
      frame->m_camNum = cam_ids[i];
      haveCam = true;
    }
  }
  if(!haveCam)
    throw BALTException(__HERE__, "video has no camera %d", camNum);
}

/**
 * Returns whether any frames have been grabbed yet.
 */
bool ImgSeqDevice::HaveFrames()
{
  for(int i = 0; i < NumCameras(); i++)
    if(images[i] == 0)
      return false;
  return true;
}

int ImgSeqDevice::NumCameras()
{
  return (int)cam_ids.size();
}

void ImgSeqDevice::GetCameraIds(vector<int> &ids)
{
  ids = cam_ids;
}

void ImgSeqDevice::GetImageSize(int &width, int &height)
{
  IplImage *img = 0;

  if(filenames.size() == 0)
    throw BALTException(__HERE__, "video not initialised");
  // load the first image and have a look at its size
  img = cvLoadImage(filenames[0].c_str(), CV_LOAD_IMAGE_COLOR);
  if(img == 0)
    throw BALTException(__HERE__, "failed load image '%s'", filenames[0].c_str());
  width = img->width;
  height = img->height;
  cvReleaseImage(&img);
}

int ImgSeqDevice::GetFramerate()
{
  return framerate_ms;
}

void ImgSeqDevice::ConvertFrame(IplImage *img, ImageFrame *frame)
{
  int channels = 3;  // Image frame always has BGR24

  assert(img != 0 && frame != 0);
  if(img->nChannels == channels)
  {
    // note: We assume that ImageFrame and IplImage have same colour format
    // and there is no padding at the end of lines.
    /*frame->m_width = img->width;
    frame->m_height = img->height;
    frame->m_image.length(img->imageSize);
    for(unsigned p = 0; p < img->imageSize; p++)
      frame->m_image[p] = img->imageData[p];*/

    // note: This is the clean, slow way of copying. We do not assume
    // any specific memory layout of the two images, but use regular array
    // access operators.
    frame->m_width = img->width;
    frame->m_height = img->height;
    frame->m_image.length(img->width*img->height*channels);
    if((unsigned)img->depth == IPL_DEPTH_8U || (unsigned)img->depth == IPL_DEPTH_8S)
    {
      int x, y, c;
      for(y = 0; y < img->height; y++)
        for(x = 0; x < img->width; x++)
          for(c = 0; c < channels; c++)
            frame->m_image[channels*(y*frame->m_width + x) + c] =
              img->imageData[y*img->widthStep + channels*x + c];
    }
    else
      throw BALTException(__HERE__, "can only handle 8 bit colour values");
  }
  else
    throw BALTException(__HERE__, "can only handle colour images - "
      "the video seems to be grey scale");
}

/*void catcher(int sig)
{
  // does not have to do anything
}*/

/*void ImgSeqDevice::SetTimer()
{
  struct sigaction sact;

  sigemptyset( &sact.sa_mask );
  sact.sa_flags = 0;
  sact.sa_handler = catcher;
  sigaction(SIGALRM, &sact, NULL);

  if(setitimer( which, &value, NULL) != 0)
    throw BALTException(__HERE__, "failed to set timer");

  // in the capture function call
  //pause();
}*/

