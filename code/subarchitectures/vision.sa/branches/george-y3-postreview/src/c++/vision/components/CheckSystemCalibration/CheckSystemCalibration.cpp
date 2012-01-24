/**
 * @author Michael Zillich
 * @date February 2009
 */

// Convenience
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <ctime>

#include <highgui.h>
#include <VideoUtils.h>
#include "CheckSystemCalibration.h"

/**
 * distance between pattern corners in x and y
 */
static const double PATTERN_DX = 0.08;
static const double PATTERN_DY = 0.08;
/**
 * Offset in x and y (z = 0) of the top left corner of the calibration pattern
 * w.r.t. robot ego
 */
static const double PATTERN_X_OFFSET = 1.030;
static const double PATTERN_Y_OFFSET = 0.280; 
/**
 * Number of corners in x and y direction. Note: robot ego x points forward,
 * the pattern is viewed in landscape orientation. So NX < NY.
 */
static const int PATTERN_NX = 6;
static const int PATTERN_NY = 8;

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::CheckSystemCalibration();
  }
}


namespace cast
{

using namespace std;
using namespace VisionData;
using namespace cogx::Math;

CheckSystemCalibration::~CheckSystemCalibration()
{
#ifndef FEAT_VISUALIZATION
  cvDestroyWindow(getComponentID().c_str());
#endif
}

void CheckSystemCalibration::configure(const map<string,string> & _config)
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

void CheckSystemCalibration::start()
{
  // get connection to the video server
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  // register our client interface to allow the video server pushing images
  Video::VideoClientInterfacePtr servant = new VideoClientI(this);
  registerIceServer<Video::VideoClientInterface, Video::VideoClientInterface>(servant);

#ifdef FEAT_VISUALIZATION
  m_display.connectIceClient(*this);
  m_display.installEventReceiver();
  m_display.addCheckBox(getComponentID(), "toggle.checksyscalib.running", "&Streaming");
#else
  cvNamedWindow(getComponentID().c_str(), 1);
#endif

  vector<int> camIds;
  camIds.push_back(camId);
}

#ifdef FEAT_VISUALIZATION
void CheckSystemCalibration::CVvDisplayClient::handleEvent(const Visualization::TEvent &event)
{
  if (event.type == Visualization::evCheckBoxChange) {
    if (event.sourceId == "toggle.checksyscalib.running") {
      //debug(std::string("Time: ") + sfloat (fclocks()));
      bool newrcv = (event.data != "0");
      if (newrcv != pComp->receiving) {
        if(pComp->receiving) {
          pComp->videoServer->stopReceiveImages(pComp->getComponentID());
          pComp->log("Stopped receiving images");
        }
        else {
          vector<int> camIds;
          camIds.push_back(pComp->camId);
          pComp->videoServer->startReceiveImages(pComp->getComponentID(), camIds, 0, 0);
          pComp->log("Started receiving images");
        }
        pComp->receiving = !pComp->receiving;
      }
    }
  }
}

#endif

void CheckSystemCalibration::destroy()
{
#ifndef FEAT_VISUALIZATION
  cvDestroyWindow(getComponentID().c_str());
#endif
}

void CheckSystemCalibration::receiveImages(const std::vector<Video::Image>& images)
{
  IplImage *iplImg = convertImageToIpl(images[0]);
  drawCalibrationPattern(images[0].camPars, iplImg);
#ifdef FEAT_VISUALIZATION
  Video::Image img;
  convertImageFromIpl(iplImg, img);
  m_display.setImage(getComponentID(), img);
#else
  cvShowImage(getComponentID().c_str(), iplImg);
  cvWaitKey(10);
#endif
  cvReleaseImage(&iplImg);
}

void CheckSystemCalibration::drawCalibrationPattern(const Video::CameraParameters &cam, IplImage *iplImg)
{
  double dx = PATTERN_DX;
  double dy = PATTERN_DY;
  double xoffs = PATTERN_X_OFFSET;
  double yoffs = PATTERN_Y_OFFSET;
  int nx = PATTERN_NX;
  int ny = PATTERN_NY;
  double x = xoffs, y = yoffs;
  for(int i = 0; i < nx; i++, x -= dx)
  {
    Vector2 p1 = projectPoint(cam, vector3(x, yoffs, 0.));
    Vector2 p2 = projectPoint(cam, vector3(x, yoffs - (double)(ny - 1)*dy, 0.));
    cvLine(iplImg, cvPoint(p1.x, p1.y), cvPoint(p2.x, p2.y), cvScalar(0, 255, 0));
  }
  for(int i = 0; i < ny; i++, y -= dy)
  {
    Vector2 p1 = projectPoint(cam, vector3(xoffs, y, 0.));
    Vector2 p2 = projectPoint(cam, vector3(xoffs - (double)(nx - 1)*dx, y, 0.));
    cvLine(iplImg, cvPoint(p1.x, p1.y), cvPoint(p2.x, p2.y), cvScalar(0, 255, 0));
  }
  log("camera pose is:");
  log(toString(cam.pose));
}

}
