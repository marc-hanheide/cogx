/**
 * Video client process.
 * A process which can receive images from a video server.
 *
 * @author Michael Zillich
 * @date October 2006
 */

#include <cast/architecture/ManagedProcess.hpp>
#include "vision/idl/Vision.hh"
#include "vision/utils/ConnHub.hpp"
#include "vision/utils/VideoClt.hpp"

#ifndef VIDEO_CLIENT_PROCESS
#define VIDEO_CLIENT_PROCESS

using namespace cast; using namespace std; using namespace boost; //default useful namespaces, fix to reflect your own code

class VideoClientProcess : public ManagedProcess,
                           public PullSender<Vision::ImageFrame>
{
private:
  PullConnectorOut<Vision::ImageFrame> *img_pull;
  // hub handling all socket connection related things
  ConnHub *connHub;
  // the socket-based connection to the video server
  VideoClt *video;

protected:
  /**
   * Convenience function for getting the most recent image from the video
   * server.
   * Note that it is the callers responsibility to delete the image.
   */
  Vision::ImageFrame* GetImage(int camNum);
  /**
   * Get the most recent image from the video server for the given camera.
   * Note: the image must be provided by the user (and is resized if necessary).
   */
  void getImage(int camNum, Vision::ImageFrame &img);


  virtual void addChangeFilter(const cdl::WorkingMemoryChangeFilter & _filter,  
			       WorkingMemoryChangeReceiver * _pReceiver);

  /*
  virtual void addChangeFilter(const string &_type, 
			       const cdl::WorkingMemoryOperation & _op, 
			       const bool & _local,
			       WorkingMemoryChangeReceiver * _pReceiver);
  */

  /**
   * Open the extra socket based connectiton to the video server.
   * This must be called by every derived class before any images can be
   * received.
   * @param hostname  Host the video server is running on, e.g. irobot1
   * @param port  port number the video server is listening on, e.g. 50000
   */
  void openVideoConnection(const string &hostname, int port);

public:
  VideoClientProcess(const string &_id);
  virtual ~VideoClientProcess();
  virtual void configure(map<string,string> & _config);
  virtual void setPullConnector(const string &_connectionID,
    PullConnectorOut<Vision::ImageFrame> *gc);
  
};

#endif

