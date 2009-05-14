#ifndef VIDEO_SERVER_H
#define VIDEO_SERVER_H

#include <string>
#include <cast/architecture/UnmanagedProcess.hpp>
#include <vision/idl/Vision.hh>
#include <CoSyCommon/idl/Math.hh>
#include "vision/utils/ConnHub.hpp"
#include "vision/components/VideoServer/VideoDevice.h"

using namespace cast; using namespace std; using namespace boost; //default useful namespaces, fix to reflect your own code

/**
 *  @author Somboon Hongeng, Michael Zillich
 */ 
class VideoServer : public UnmanagedProcess,
                    public PullReceiver<Vision::ImageFrame>
{  
public:
  VideoServer(const string &_id);
  virtual ~VideoServer();

  virtual void configure(map<string,string> & _config);
  virtual void runComponent();
  virtual void receivePullQuery(const FrameworkQuery & _query, 
      FrameworkLocalData<Vision::ImageFrame> * & _pData);
  /**
   * Retrieves previously grabbed frame for the given camera number and stores
   * it as current frame. Access it with getCurrentFrame().
   */
  void retrieveCurrentFrame(int camNum);
  /**
   * Returns a reference to the current frame (retrieved with
   * retrieveCurrentFrame().
   */
  const Vision::ImageFrame& getCurrentFrame();

protected:
  virtual void redrawGraphics2D();
  virtual void redrawGraphicsText();

private:
  // how much time to buffer in [ms]
  //static const int BUFFERED_TIME = 5000;

  VideoDevice *video;
  /**
   * The current frame, i.e. the last that was retrieved from the video device.
   * If downsampling is set, this frame will be in the downsamled size.
   */
  Vision::ImageFrame *curFrame;
  //vector<std::string> stateIds;
  //vector<Vision::ImageFrame*> ring_buf;
  //int ring_idx;
  int downsample;
  // hub handling all socket connection related things
  ConnHub *connHub;

  //Vision::ImageFrame* FindFrame(int camNum, FrameworkBasics::BALTTime time);
  //void AllocateRingBuffer();
  //void FreeRingBuffer();
  VideoDevice* ConfigureImgSeq(map<string,string> &_config);
  VideoDevice* ConfigureV4L(map<string,string> &_config);
  VideoDevice* ConfigureOpenCv(map<string,string> &_config);
  void initSocketComm(int listenPort);
};

#endif

