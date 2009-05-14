/**
 * @author Michael Zillich
 * @date October 2006
 */

#include <opencv/cv.h>
#include <vision/utils/VisionUtils.h>
#include "OpenCvDevice.h"

using namespace Vision;

/**
 * @param ids  list of camera IDs
 * @param dev_nums  list of device numbers (typically 0, 1) corresponding to
 *                  camera IDs
 * @param bayer  Some cameras (e.g. Point Grey Flea) return the raw Bayer
 *               pattern rather than YUV or RGB. For these we have to perform
 *               Bayer ro RGB conversion. This parameter indicates the
 *               order of R,G and B pixels in the Bayer pattern: one of
 *               "BGGR" "GBBR" "RGGB" "GRRB" or "" (for no conversion).
 */
OpenCvDevice::OpenCvDevice(const vector<int> &ids, const vector<int> &dev_nums,
    const string &bayer)
{
  if(ids.size() == 0)
    throw BALTException(__HERE__, "must specify at least one camera");
  if(ids.size() != dev_nums.size())
    throw BALTException(__HERE__, "number of cameras and devices does not match");

  cam_ids = ids;
  captures.resize(cam_ids.size());
  times.resize(cam_ids.size());
  latest_images.resize(cam_ids.size());
  for(unsigned i = 0; i < cam_ids.size(); i++)
    latest_images[i] = 0;
  for(unsigned i = 0; i < cam_ids.size(); i++)
  {
    captures[i] = cvCreateCameraCapture(dev_nums[i]);
    if(captures[i] == 0)
      throw BALTException(__HERE__, "failed to create capture for video device %d",
        dev_nums[i]);
    if(bayer.empty())
    {
      bayer_cvt = CV_COLORCVT_MAX;
    }
    else
    {
      if(bayer == "BGGR")
        bayer_cvt = CV_BayerBG2RGB;
      else if(bayer == "GBBR")
        bayer_cvt = CV_BayerGB2RGB;
      else if(bayer == "RGGB")
        bayer_cvt = CV_BayerRG2RGB;
      else if(bayer == "GRRB")
        bayer_cvt = CV_BayerGR2RGB;
      else
        throw BALTException(__HERE__, "invalid bayer order '%s', must be one of "
            "'BGGR' 'GBBR' 'RGGB' 'GRRB'",
            bayer.c_str());
      cvSetCaptureProperty(captures[i], CV_CAP_PROP_CONVERT_RGB, 0.0);
    }
  }
}

OpenCvDevice::~OpenCvDevice()
{
  for(unsigned i = 0; i < captures.size(); i++)
    cvReleaseCapture(&captures[i]);
}

void OpenCvDevice::GrabFrames()
{
  for(unsigned i = 0; i < captures.size(); i++)
  {
    cvGrabFrame(captures[i]);
    // pointer to internal frame, must not be deleted
    latest_images[i] = cvRetrieveFrame(captures[i]);
  }
  BALTTime time = BALTTimer::getBALTTime();
  for(unsigned i = 0; i < times.size(); i++)
    times[i] = time;
}

void OpenCvDevice::RetrieveFrames(std::vector<ImageFrame*> &frames)
{
  if(frames.size() != (unsigned)NumCameras())
    throw BALTException(__HERE__, "requested %d images from %d cameras",
        frames.size(), NumCameras());
  // To handle the case where Retrieve is called before any Grab
  if(!HaveFrames())
    GrabFrames();
  for(unsigned i = 0; i < captures.size(); i++)
  {
    if(frames[i] == 0)
      throw BALTException(__HERE__, "provided frame pointer is NULL");
    ConvertFrame(latest_images[i], frames[i]);
  }
  for(unsigned i = 0; i < captures.size(); i++)
  {
    frames[i]->m_time = times[i];
    frames[i]->m_camNum = cam_ids[i];
  }
}

void OpenCvDevice::RetrieveFrame(int camNum, ImageFrame *frame)
{
  bool haveCam = false;
  if(frame == 0)
    throw BALTException(__HERE__, "provided frame pointer is NULL");
  // To handle the case where Retrieve is called before any Grab
  if(!HaveFrames())
    GrabFrames();
  for(unsigned i = 0; i < captures.size(); i++)
  {
    if(camNum == cam_ids[i])
    {
      ConvertFrame(latest_images[i], frame);
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
bool OpenCvDevice::HaveFrames()
{
  for(int i = 0; i < NumCameras(); i++)
    if(latest_images[i] == 0)
      return false;
  return true;
}

int OpenCvDevice::NumCameras()
{
  return captures.size();
}

void OpenCvDevice::GetCameraIds(std::vector<int> &ids)
{
  ids = cam_ids;
}

void OpenCvDevice::GetImageSize(int &width, int &height)
{
  if(captures.size() > 0)
  {
    width = (int)cvGetCaptureProperty(captures[0], CV_CAP_PROP_FRAME_WIDTH);
    height = (int)cvGetCaptureProperty(captures[0], CV_CAP_PROP_FRAME_HEIGHT);
  }
  else
    width = height = 0;
}

int OpenCvDevice::GetFramerate()
{
  if(captures.size() > 0)
  {
    double fps = cvGetCaptureProperty(captures[0], CV_CAP_PROP_FPS);
    if(fps > 0.)
      return (int)(1000./fps);  // return frame rate in [ms]
    else
      return 0;
  }
  else
    return 0;
}

void OpenCvDevice::ConvertFrame(IplImage *src, ImageFrame *frame)
{
  int channels = 3;  // Image frame always has BGR24
  IplImage *img = 0;

  assert(src != 0);
  assert(frame != 0);
  if(!HaveBayer())
  {
    img = src;
    if(img->nChannels != channels)
      throw BALTException(__HERE__, "can only handle colour images - "
        "the video seems to be grey scale");
  }
  else
  {
    // HACK: how do we know the correct colour code? 
    img = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 3);
    if(img == 0)
      throw BALTException(__HERE__, "failed to allocate image buffer");
    cvCvtColor(src, img, bayer_cvt);
  }
  // note: We assume that ImageFrame and IplImage have same colour format
  // and there is no padding at the end of lines.
  frame->m_width = img->width;
  frame->m_height = img->height;
  frame->m_image.length(img->imageSize);
  for(unsigned p = 0; p < (unsigned)img->imageSize; p++)
    frame->m_image[p] = img->imageData[p];

  // note: This is the clean, slow way of copying. We do not assume
  // any specific memory layout of the two images, but use regular array
  // access operators.
  /*frame->m_width = img->width;
  frame->m_height = img->height;
  frame->m_image.length(img->width*img->height*img->channels);
  if(img->depth == IPL_DEPTH_8U || img->depth == IPL_DEPTH_8S)
  {
    int x, y, c;
    for(y = 0; y < img->height; y++)
      for(x = 0; x < img->width; x++)
        for(c = 0; c < channels; c++)
          frame->m_image[channels*(y*frame->m_width + x) + c] =
            img->imageData[y*img->widthStep + channels*x + c];
  }
  else
    throw BALTException(__HERE__, "can only handle 8 bit colour values");*/
  if(HaveBayer())
    cvReleaseImage(&img);
}
