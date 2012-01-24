/**
 * @author Marko Mahnič
 * @date April 2011
 * @brief
 */

#ifndef PLAYERVIDEOSERVER_5M62XU1U
#define PLAYERVIDEOSERVER_5M62XU1U

#include <string>
#include <vector>
#include <stdexcept>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libplayerc++/playerc++.h>
#if 0
#include <highgui.h>
#endif 
#include <ImageCache.h>
#include <Timers.h>
#include "VideoServer.h"

namespace cast
{

class CCameraInfo
{
  static Video::CIplImageCache m_imageCache;
public:
  PlayerCc::CameraProxy* m_pCamera;
  int m_devNum;
  bool m_bFrameCached;   // is the last frame grabbed by Read() already cached
  int m_width;
  int m_height;
  IplImage* m_pRetrievedFrame;
  cast::cdl::CASTTime m_grabTime;
  CCameraInfo(int device, PlayerCc::CameraProxy *pCamera)
  {
    m_devNum = device;
    m_pCamera = pCamera;
    m_pRetrievedFrame = NULL;
    m_bFrameCached = false;
    m_width = 0;
    m_height = 0;
  }
  ~CCameraInfo()
  {
    if (m_pCamera) delete m_pCamera;
    m_pCamera = NULL;
    m_pRetrievedFrame = NULL;
  }
  void readParams()
  {
    m_width = m_pCamera->GetWidth();
    m_height = m_pCamera->GetHeight();
  }
  // Transfer the frame from Read() buffer to m_pRetrievedFrame
  void retrieveFrame()
  {
    char id[32];
    sprintf(id, "player%d", m_devNum);
    m_pRetrievedFrame = m_imageCache.getImage(id, m_width, m_height, IPL_DEPTH_8U, 3);
    m_pCamera->Decompress();
    m_pCamera->GetImage((uint8_t*)m_pRetrievedFrame->imageData);
    m_bFrameCached = true;
  }
};

/**
 * @brief Video device simply wrapping the OpenCV capture API.
 */
class PlayerVideoServer : public VideoServer
{
private:
  std::string m_playerHost;
  int m_playerPort;
  PlayerCc::PlayerClient *m_pPlayer;
  std::vector<CCameraInfo*> m_cameras;
  std::vector<bool> m_bFrameCached;

#if 0
  /**
   * pointers to retrieved images, point to internal memory of the capture
   * device, do not delete!
   */
  std::vector<IplImage*> retrievedImages;

  /**
   * time stamps when Ipl images were captured.
   */
  std::vector<cast::cdl::CASTTime> grabTimes;

  /**
   * format for Bayer to RGB conversion, CV_COLORCVT_MAX for no conversion
   */
  //int bayerCvt;  
  CvSize captureSize;
#endif

  int framerateMillis;

  /**
   * Timer to measure actual frame rate.
   */
  Video::CTimeStats timeStats;
  long long m_nextFrameTime;
  long m_frameDurationMs;

  Video::CIplImageCache m_imageCache;

  /**
   * get video resolution for given camera
   * @param camIdx which camera
   * @param size video resolution
   */
  // void getResolution(int camIdx, CvSize &size);
  /**
   * set resolution for given camera
   * @param camIdx which camera
   * @param size requested video resolution, on exit contains the actually
   *        set resolution, which might differ dependig on the cameras
   *        cpabilities
   * @return true if the requested resolution could be set, false if another
   *        reslution was chosen
   */
  // bool setResolution(int camIdx, CvSize &size);
  void init(const std::vector<int> &dev_nums) throw(std::runtime_error);
  /**
   * Converts an IplImage to a system Image format.
   */
  //void copyImage(const IplImage *iplImg, Video::Image &img) throw(std::runtime_error);
  void grabFramesInternal();
  void retrieveFrameInternal(int camIdx, int width, int height, Video::Image &frame);
  virtual void retrieveFrames(const std::vector<int> &camIds, int width, int height, std::vector<Video::Image> &frames);
  virtual void retrieveFrames(int width, int height, std::vector<Video::Image> &frames);
  virtual void retrieveFrame(int camId, int width, int height, Video::Image &frame);
  virtual void retrieveHRFrames(std::vector<Video::Image> &frames);

public:
  PlayerVideoServer();
  virtual ~PlayerVideoServer();
  virtual void configure(const std::map<std::string,std::string> & _config)
    throw(std::runtime_error);
  virtual void start();
  virtual void grabFrames();
  virtual void getImageSize(int &width, int &height);
  virtual int getFramerateMilliSeconds();
  virtual const std::string getServerName();
  virtual void runComponent();
};

}

#endif
/* vim:set sw=2 sts=4 ts=8 et:vim */
