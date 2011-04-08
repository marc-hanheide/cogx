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
  if((it = _config.find("--pan")) != _config.end())
  {
    istringstream str(it->second);
    str >> pan;
  }

  if((it = _config.find("--tilt")) != _config.end())
  {
    istringstream str(it->second);
    str >> tilt;
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

  // register with the PTZ server
  Ice::CommunicatorPtr ic = getCommunicator();

  Ice::Identity id;
  id.name = "PTZServer";
  id.category = "PTZServer";

  std::ostringstream str;
  str << ic->identityToString(id) 
    << ":default"
    << " -h localhost"
    << " -p " << cast::cdl::CPPSERVERPORT;

  Ice::ObjectPrx base = ic->stringToProxy(str.str());    
  m_PTUServer = ptz::PTZInterfacePrx::uncheckedCast(base);

#ifdef FEAT_VISUALIZATION
  m_display.connectIceClient(*this);
  m_display.installEventReceiver();
  m_display.addCheckBox(getComponentID(), "toggle.viewer.running", "&Streaming");
#else
  cvNamedWindow(getComponentID().c_str(), 1);
#endif

  vector<int> camIds;
  camIds.push_back(camId);
}

#ifdef FEAT_VISUALIZATION
void CheckSystemCalibration::CVvDisplayClient::handleEvent(const Visualization::TEvent &event)
{
  //debug(event.data + " (received by CheckSystemCalibration)");
}

std::string CheckSystemCalibration::CVvDisplayClient::getControlState(const std::string& ctrlId)
{
  return "";
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
#ifdef FEAT_VISUALIZATION
  m_display.setImage(getComponentID(), images[0]);
#else
  IplImage *iplImage = Video::convertImageToIpl(images[0]);
  cvShowImage(getComponentID().c_str(), iplImage);
  cvReleaseImage(&iplImage);
#endif
}

void CheckSystemCalibration::MovePanTilt(double pan, double tilt, double tolerance)
{
	log("Moving pantilt to: %f %f with %f tolerance", pan, tilt, tolerance);
  ptz::PTZPose p;
  ptz::PTZReading ptuPose;
  p.pan = pan ;
  p.tilt = tilt;
  p.zoom = 0;
  m_PTUServer->setPose(p);
  bool run = true;
  ptuPose = m_PTUServer->getPose();
  double actualpose = ptuPose.pose.pan;
  while(run){
    m_PTUServer->setPose(p);
    ptuPose = m_PTUServer->getPose();
    actualpose = ptuPose.pose.pan;
    log("actualpose is: %f", actualpose);
    if (pan > actualpose){
      if (actualpose > abs(pan) - tolerance){
        log("false actualpose is: %f, %f", actualpose, abs(pan) + tolerance);
        run = false;
      }
        }
    if (actualpose > pan){
      if (actualpose < abs(pan) + tolerance)
        run = false;
        }
      if(pan == actualpose)
      run = false;
    
    usleep(10000);
  }
  log("Moved.");
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
}

void CheckSystemCalibration::runComponent()
{
  sleepComponent(1000);
	MovePanTilt(pan, tilt, 5*M_PI/180);
  sleepComponent(3000);
  Video::Image img;
  videoServer->getImage(camId, img);
#ifdef FEAT_VISUALIZATION
  m_display.setImage(getComponentID(), img);
#else
  IplImage *iplImg = convertImageToIpl(img);
  drawCalibrationPattern(img.camPars, iplImg);
  cvShowImage(getComponentID().c_str(), iplImg);
  cvWaitKey(100);
  cvShowImage(getComponentID().c_str(), iplImg);
  cvWaitKey(100);
  cvShowImage(getComponentID().c_str(), iplImg);
  cvWaitKey(100);
  cvReleaseImage(&iplImg);
#endif

}

} // namespace

