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

// Convenience
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <ctime>

std::string sfloat(double f, int precision=6)
{
   std::ostringstream out;
   out << std::fixed << std::setprecision(precision) << f;
   return out.str();
}

double fclocks()
{
   return 1.0 * clock() / CLOCKS_PER_SEC;
}
// --------------------------


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

#ifdef FEAT_VISUALIZATION
  m_display.configureDisplayClient(_config);
#endif

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

#ifdef FEAT_VISUALIZATION
// printf("Feat_Visualization is ON!!!\n");
  m_display.connectIceClient(*this);
  m_display.installEventReceiver();
  m_display.addCheckBox(getComponentID(), "toggle.viewer.running", "&Streaming");
#else
  cvNamedWindow(getComponentID().c_str(), CV_WINDOW_AUTOSIZE);
#endif

  vector<int> camIds;
  camIds.push_back(camId);
  // start receiving images pushed by the video server
  videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);
  receiving = true;
}

#ifdef FEAT_VISUALIZATION
void VideoViewer::CVvDisplayClient::handleEvent(const Visualization::TEvent &event)
{
  //debug(event.data + " (received by VideoViewer)");
  if (event.type == Visualization::evCheckBoxChange) {
    if (event.sourceId == "toggle.viewer.running") {
      //debug(std::string("Time: ") + sfloat (fclocks()));
      bool newrcv = (event.data != "0");
      if (newrcv != pViewer->receiving) {
        if(pViewer->receiving) {
          pViewer->videoServer->stopReceiveImages(pViewer->getComponentID());
          pViewer->println("Stopped receiving images");
        }
        else {
          vector<int> camIds;
          camIds.push_back(pViewer->camId);
          pViewer->videoServer->startReceiveImages(pViewer->getComponentID(), camIds, 0, 0);
          pViewer->println("Started receiving images");
        }
        pViewer->receiving = !pViewer->receiving;
      }
    }
  }
}

std::string VideoViewer::CVvDisplayClient::getControlState(const std::string& ctrlId)
{
  if (ctrlId == "toggle.viewer.running") {
    return pViewer->receiving ? "1" : "0";
  }
  return "";
}
#endif

void VideoViewer::destroy()
{
#ifndef FEAT_VISUALIZATION
  cvDestroyWindow(getComponentID().c_str());
#endif
}

void VideoViewer::receiveImages(const std::vector<Video::Image>& images)
{
#ifdef FEAT_VISUALIZATION
  m_display.setImage(getComponentID(), images[0]);
#else
  IplImage *iplImage = convertImageToIpl(images[0]);
  cvShowImage(getComponentID().c_str(), iplImage);
  cvReleaseImage(&iplImage);
#endif
}

void VideoViewer::runComponent()
{
  sleepComponent(1000);

#ifndef FEAT_VISUALIZATION
  println("press <s> to stop/start receving images");
#endif

  while(isRunning())
  {
    // needed to make the window appear
    // (an odd behaviour of OpenCV windows!)
#ifdef FEAT_VISUALIZATION
    sleepComponent(100);
#else
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
#endif // FEAT_VISUALIZATION
  }
}

} // namespace

