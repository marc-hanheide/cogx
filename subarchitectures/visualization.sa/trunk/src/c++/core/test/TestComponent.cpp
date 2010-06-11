/**
 * @author Michael Zillich (original file: VideoViewer)
 * @date February 2009
 *
 * Adapted by Marko Mahnič.
 */

#include <highgui.h>
#include <VideoUtils.h>
#include "TestComponent.hpp"

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

  vector<int> camIds;
  camIds.push_back(camId);

#ifdef FEAT_VISUALIZATION
  m_bSendIplImage = false;
  m_display.connectIceClient(*this);
  m_display.installEventReceiver();
  m_display.setEventCallback(this, &VideoViewer::handleGuiEvent);
  m_display.setStateQueryCallback(this, &VideoViewer::getControlState);

  //m_display.addButton("toggle.viewer.running", "Start");
  m_display.addCheckBox(getComponentID(), "toggle.viewer.running", "&Streaming");
  m_display.addCheckBox(getComponentID(), "toggle.viewer.sendipl", "Send &IplImage");
  m_display.addButton(getComponentID(), "viewer.do.nothing", "Show &time");
  //m_display.addButton(getComponentID(), "viewer.do.nothing2", "&Test 2");
  //m_display.addButton(getComponentID(), "viewer.do.nothing3", "&Test 3");
#else
  cvNamedWindow(getComponentID().c_str(), 1);
#endif

  // start receiving images pushed by the video server
  receiving = false;
  if (receiving)
    videoServer->startReceiveImages(getComponentID().c_str(), camIds, 0, 0);
}

#ifdef FEAT_VISUALIZATION
void VideoViewer::handleGuiEvent(const Visualization::TEvent &event)
{
  debug(event.data + " (received by VideoViewer)");
  if (event.type == Visualization::evCheckBoxChange) {
    if (event.sourceId == "toggle.viewer.running") {
      debug(std::string("Time: ") + sfloat (fclocks()));
      bool newrcv = (event.data != "0");
      if (newrcv != receiving) {
        if(receiving) {
          videoServer->stopReceiveImages(getComponentID());
          println("Stopped receiving images");
        }
        else {
          vector<int> camIds;
          camIds.push_back(camId);
          videoServer->startReceiveImages(getComponentID(), camIds, 0, 0);
          println("Started receiving images");
        }
        receiving = !receiving;
      }
    }
    else if (event.sourceId == "toggle.viewer.sendipl") {
      debug(std::string("Time: ") + sfloat (fclocks()));
      bool newrcv = (event.data != "0");
      if (m_bSendIplImage != newrcv) {
        m_bSendIplImage = newrcv;
        if(m_bSendIplImage) {
          println("Sendind images of type IplImage.");
        }
        else {
          println("Sending images of type Video::Image.");
        }
      }
    }
  }
  else if (event.type == Visualization::evButtonClick) {
    if (event.sourceId == "viewer.do.nothing") {
      debug(std::string("Time: ") + sfloat (fclocks()));
      println("The button works.");
    }
  }
}

std::string VideoViewer::getControlState(const std::string& ctrlId)
{
  if (ctrlId == "toggle.viewer.running") {
    return receiving ? "1" : "0";
  }
  else if (ctrlId == "toggle.viewer.sendipl") {
    return m_bSendIplImage ? "1" : "0";
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
  if (! m_bSendIplImage)
    m_display.setImage(getComponentID(), images[0]);
  else {
    IplImage *iplImage = convertImageToIpl(images[0]);
    m_display.setImage(getComponentID(), iplImage);
    cvReleaseImage(&iplImage);
  }
  if (1) {
    static double iangl = 0;
    iangl += 10;
    if (iangl > 360) iangl = 0;
    double mdata[9];
    double scl = 0.5;
    double angl = iangl * 3.14 / 180;
    mdata[0] = scl*cos(angl);  mdata[1] = scl*sin(angl); mdata[2] = 0;
    mdata[3] = -scl*sin(angl); mdata[4] = scl*cos(angl); mdata[5] = 0;
    mdata[6] = 100;            mdata[7] = 100;           mdata[8] = 1;
    CvMat mat = cvMat(3, 3, CV_64FC1, mdata);
    m_display.setObjectTransform2D("Visualization.test.SVG", "little-lion", &mat);
  }
  {
    std::stringstream str;
    str << "The current CAST running time is " << sfloat (fclocks());
    m_display.setHtml("@info.TestComponent", "time", str.str());
  }
#else
  IplImage *iplImage = convertImageToIpl(images[0]);
  cvShowImage(getComponentID().c_str(), iplImage);
  cvReleaseImage(&iplImage);
#endif
}

void VideoViewer::runComponent()
{
  sleepComponent(1000);

#ifdef FEAT_VISUALIZATION
  {
    std::ifstream infile;
    infile.open("subarchitectures/visualization.sa/config/test/images/lion.svg", std::ifstream::in);
    std::stringstream str;
    str << infile.rdbuf();
    infile.close();
    m_display.setObject("Visualization.test.SVG", "lion", str.str());
    m_display.setObject("Visualization.test.SVG", "little-lion", str.str());
  }
  {
    std::stringstream str;
    str << "function initlists()\n";
    str <<   "if DispList:exists('myobject') then return end\n";
    str <<   "DispList:create('myobject')\n";
    str <<     "glBegin(GL_QUADS)\n";
    str <<     "glVertex(0.1, 0.1, 0.1)\n";
    str <<     "glVertex(0.3, 0.1, 0.2)\n";
    str <<     "glVertex(0.3, 0.4, 0.3)\n";
    str <<     "glVertex(0.0, 0.5, 0.4)\n";
    str <<     "glEnd()\n";
    str <<   "DispList:endList()\n";
    str << "end\n";

    str << "function render()\n";
    str << "initlists()\n";
    str << "N=36\n";
    str << "tabsin={0";
    for (int i = 0; i < 36; i++) str << "," << sin(10*i*3.14/180);
    str << "}\n";
    str << "tabcos={1.0";
    for (int i = 0; i < 36; i++) str << "," << cos(10*i*3.14/180);
    str << "}\n";
    str <<
         //"glPointSize(2)\n"
         "glBegin(GL_LINES)\n"
         "for i=1,10000 do\n"
            "glColor(i%23/23.0,i%17/17.0,i%11/11.0)\n"
            "glVertex(tabcos[i%N+1], tabsin[i%N+1], -i/100.0)\n"
         "end\n"
         "glEnd()\n"
         "glColor(0.5,0.1,0.1)\n"
         "DispList:draw('myobject')\n"
         "glPushMatrix()\n"
           "glColor(0.1,0.7,0.4)\n"
           "glTranslate(0.5,0.5,0.0)\n"
           "DispList:draw('myobject')\n"
         "glPopMatrix()\n"
      "end\n";
    m_display.setLuaGlObject("Visualization.sa.LuaGl", "Points", str.str());
  }
  {
    std::stringstream strA;
    strA << "This is the TestComponent for the Display Server<br>";
    m_display.setHtml("@info.TestComponent", "text", strA.str());
  }
#else
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

}

