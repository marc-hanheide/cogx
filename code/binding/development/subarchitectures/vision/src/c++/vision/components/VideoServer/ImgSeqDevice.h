/**
 * @author Michael Zillich
 * @date October 2006
 */

#ifndef IMG_SEQ_DEVICE_H
#define IMG_SEQ_DEVICE_H

#include <string>
#include <opencv/cv.h>
#include <balt/interface/BALTTimer.hpp>
#include "VideoDevice.h"

/**
 * Video device reading from stored files.
 * A user supplied frame rate simlulates the synchronisation of a real video
 * device.
 */
class ImgSeqDevice : public VideoDevice
{
private:
  // filenames, e.g. for 2 cameras:
  // (file0-0 file1-0 file0-1 file1-1 file0-2 file1-2 ... )
  std::vector<std::string> filenames;
  std::vector<int> cam_ids;
  std::vector<IplImage*> images;
  std::vector<FrameworkBasics::BALTTime> times;
  int frame_cnt;
  int framerate_ms;

  void Init(const std::vector<int> &ids, int rate_ms);
  void ConstructFilenames(const std::vector<std::string> &file_templates,
      int first, int last, int inc);
  int NumImages() {return filenames.size()/cam_ids.size();}
  void ConvertFrame(IplImage *img, Vision::ImageFrame *frame);
  bool HaveFrames();

public:
  ImgSeqDevice();
  /**
   * Construct using a list of filenames.
   * @param ids  camera IDs
   * @param files  array of filenames for each camera, e.g. for 2 cameras:
   *               (file0-0 file1-0 file0-1 file1-1 file0-2 file1-2 ... )
   * @param rate_ms  frame rate in [ms]
   */
  ImgSeqDevice(const std::vector<int> &ids,
      const std::vector<std::string> &files, int rate_ms);
  /**
   * Construct using a printf-style filename pattern and number range.
   * @param ids  camera IDs
   * @param file_templates  printf-style templates for each camera,
   *                        e.g. ("image%03d-L.jpg" "image%03d-R.jpg")
   * @param first  first image of sequence to load
   * @param last  last image of sequence to load
   * @param rate_ms  frame rate in [ms]
   * @param inc  increment of sequence numbers, default is 1
   */
  ImgSeqDevice(const std::vector<int> &ids,
      const std::vector<std::string> &file_templates,
      int first, int last, int rate_ms, int inc = 1);
  virtual void GrabFrames();
  virtual void RetrieveFrames(std::vector<Vision::ImageFrame*> &frames);
  virtual void RetrieveFrame(int camNum, Vision::ImageFrame *frame);
  virtual int NumCameras();
  virtual void GetCameraIds(std::vector<int> &ids);
  virtual void GetImageSize(int &width, int &height);
  virtual int GetFramerate();
};

#endif

