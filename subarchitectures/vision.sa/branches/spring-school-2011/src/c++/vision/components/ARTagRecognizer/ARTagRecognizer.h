/**
 * @author Alper aydemir 
 * @date August 2010
 *
 */

#ifndef ARTAG_RECOGNIZER_H
#define ARTAG_RECOGNIZER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <Video.hpp>
#include <VisionData.hpp>
#include <VideoClient.h>

#include <AR/ar.h>
#include <Matrix33.h>
#include "ModelLoader.h"
namespace cast
{

class ARTagRecognizer:
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
  Video::Image m_image;

  // struct to hold objects tagged with an AR tag
  struct object{
    
    std::string label;
    std::string filename;
    cogx::Math::Pose3 trans;
    int id;
    std::string wmid;
  };
  std::vector<object> taggedObjects;

  int             marker_width;
  double*          marker_center;                                                                        

  int             xsize, ysize;                                                                                         
  int             thresh;                                                                         
  int             count;
  ARParam         cparam;                    

  bool dummy;
  std::string tagpath;
  void LoadPatterns(std::string filename);
  void printPose(cogx::Math::Pose3 pose);
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
   * Our run loop. Essentially just wait for key strokes.
   */
  virtual void runComponent();
public:
  ARTagRecognizer() : camId(0)
  {
  }
  virtual ~ARTagRecognizer() {}
  void newARTagCommand(const cast::cdl::WorkingMemoryChange &objID);
  /**
   * The callback function for images pushed by the image server.
   * To be overwritten by derived classes.
   */
   virtual cogx::Math::Pose3 getMarkerPosition(std::string label);
  void init();
};

}

#endif

