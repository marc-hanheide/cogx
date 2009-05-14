/**
 * @author Michel Zillich
 * @date October 2006
 */

#ifndef VIDEO_DEVICE_H
#define VIDEO_DEVICE_H

#include <vector>
#include "vision/idl/Vision.hh"
#include <CoSyCommon/idl/Math.hh>

/**
 * Device to capture images from one or several (typically two) sources
 * (= cameras).
 * If several sources are used, they all have the same image format and
 * synchronisation between sources is handled by this class.
 * If several sources are used, calls to Capture() shall aways fill the
 * provided array in the same order.
 */
class VideoDevice
{
public:
  virtual ~VideoDevice() {}
  /**
   * Grabs frames and stores them internally.
   */
  virtual void GrabFrames() = 0;
  /**
   * Retrieves previously grabbed frames.
   * Also sets time and camera ID value.
   */
  virtual void RetrieveFrames(std::vector<Vision::ImageFrame*> &frames) = 0;
  /**
   * Retrieve just frame from one camera
   */
  virtual void RetrieveFrame(int camNum, Vision::ImageFrame *frame) = 0;


/*   virtual void RetrieveFrame(int camNum, FrameworkBasics::BALTTime time, */
/* 			     Vision::ImageFrame *frame) = 0; */


  /**
   * Waits for and then fills images (equivalent to using GrabFrames and RetrieveFrames())
   * Also sets time and camera ID value.
   */
  void CaptureFrames(std::vector<Vision::ImageFrame*> &frames)
  {
    GrabFrames();
    RetrieveFrames(frames);
  }
  /**
   * Returns number of cameras this device manages.
   */
  virtual int NumCameras() = 0;
  /**
   * Returns the IDs of cameras managed by this device.
   * At leas one camera must be present.
   * Note that there are sometimes more that one video devices, as the whole
   * system is distributed and hardware setups may require video servers on
   * different machines, So using array indices as camera IDs would not work.
   * Therefore camera IDs can be chosen freely by the user (e.g. by some config
   * file).
   */
  virtual void GetCameraIds(std::vector<int> &ids) = 0;
  /**
   * Returns the devices image size, must be > 0.
   * Note that all images have the same size (and colour format). This class is
   * not intended to manage video sources of different sizes/formats.
   */
  virtual void GetImageSize(int &width, int &height) = 0;
  /**
   * Returns the frame rate in [ms], must be > 0.
   * Is used e.g. when allocating buffers for given time interval.
   * Note that all video sources have the same frame rate.
   */
  virtual int GetFramerate() = 0;
};

#endif

