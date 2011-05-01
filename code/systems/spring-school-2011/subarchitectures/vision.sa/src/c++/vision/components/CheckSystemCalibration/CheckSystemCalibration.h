/**
 * @author Michael Zillich
 * @date February 2009
 *
 * Just receives images and displays them.
 * A dummy component showing how to get images.
 */

#ifndef CHECK_SYSTEM_CALIBRATION_H
#define CHECK_SYSTEM_CALIBRATION_H

#include <cast/architecture/ManagedComponent.hpp>
#include <Video.hpp>
#include <VisionData.hpp>
#include <VideoClient.h>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

namespace cast
{

class CheckSystemCalibration:
    public ManagedComponent,
    public VideoClient
{
private:
  /**
   * Which camera to get images from
   */
  int camId;
  /**
   * component ID of the video server to connect to
   */
  std::string videoServerName;
  /**
   * our ICE proxy to the video server
   */
  Video::VideoInterfacePrx videoServer;
  /**
   * Whether the component is currently receiving images.
   */
  bool receiving;

  void drawCalibrationPattern(const Video::CameraParameters &cam, IplImage *iplImg);

#ifdef FEAT_VISUALIZATION
  class CVvDisplayClient: public cogx::display::CDisplayClient
  {
     CheckSystemCalibration* pComp;
  public:
    CVvDisplayClient() { pComp = NULL; }
    void setClientData( CheckSystemCalibration* pc) { pComp = pc; }
    void handleEvent(const Visualization::TEvent &event);
  };
  CVvDisplayClient m_display;
#endif

protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();
  /**
   * called by the framework upon deletion of the component
   */
  virtual void destroy();

public:
  CheckSystemCalibration()
  {
    camId = 0;
    receiving = false;
#ifdef FEAT_VISUALIZATION
    m_display.setClientData(this);
#endif
  }
  virtual ~CheckSystemCalibration();
  /**
   * The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif

