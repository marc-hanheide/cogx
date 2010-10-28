/**
 * @author Michael Zillich
 * @date February 2009
 *
 * Just receives images and displays them.
 * A dummy component showing how to get images.
 */

#ifndef VIDEO_VIEWER_H
#define VIDEO_VIEWER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <Video.hpp>
#include <VisionData.hpp>
#include <VideoClient.h>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

namespace cast
{

class VideoViewer:
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
   * wether we are currently receiving images from the server
   */
  bool receiving;

#ifdef FEAT_VISUALIZATION
  class CVvDisplayClient: public cogx::display::CDisplayClient
  {
    VideoViewer* pViewer;
  public:
    CVvDisplayClient() { pViewer = NULL; }
    void setClientData(VideoViewer* pVideoViewer) { pViewer = pVideoViewer; }
    void handleEvent(const Visualization::TEvent &event); /*override*/
    std::string getControlState(const std::string& ctrlId); /*override*/
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
  /**
   * Our run loop. Essentially just wait for key strokes.
   */
  virtual void runComponent();

public:
  VideoViewer() : camId(0)
  {
#ifdef FEAT_VISUALIZATION
    m_display.setClientData(this);
#endif
  }
  virtual ~VideoViewer() {}
  /**
   * The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   */
  virtual void receiveImages(const std::vector<Video::Image>& images);
};

}

#endif

