/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <highgui.h>
#include <VideoUtils.h>
#include "VideoViewer.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::VideoViewer();
  }
}

namespace cast
{

using namespace std;
using namespace VisionData;

void VideoViewer::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
 
  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> camId;
  }

  // sanity checks: Have all important things be configured? Is the
  // configuration consistent?
  if(videoServerName.empty())
    throw runtime_error(exceptionMessage(__HERE__, "no video server name given"));
}

void VideoViewer::start()
{
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

  cvNamedWindow(getComponentID().c_str(), 1);

  vector<int> camIds;
  camIds.push_back(camId);
  // start receiving images pushed by the video server
  videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);
  receiving = true;
}

void VideoViewer::destroy()
{
  cvDestroyWindow(getComponentID().c_str());
}

void VideoViewer::receiveImages(const std::vector<Video::Image>& images)
{
  IplImage *iplImage = convertImageToIpl(images[0]);
  cvShowImage(getComponentID().c_str(), iplImage);
  cvReleaseImage(&iplImage);
}

void VideoViewer::runComponent()
{
  sleepComponent(5000);

  println("press <s> to stop/start receving images");
  while(isRunning())
  {
    // needed to make the window appear
    // (an odd behaviour of OpenCV windows!)
    int key = cvWaitKey(100);
    switch(key)
    {
      case 's':  // start/stop getting images
        if(receiving)
        {
          videoServer->stopReceiveImages(getComponentID().c_str());
          println("stopped receving images");
        }
        else
        {
          vector<int> camIds;
          camIds.push_back(camId);
          videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);
          println("started receving images");
        }
        receiving = !receiving;
        break;
      default:
        break;
    }
  }
}

}

