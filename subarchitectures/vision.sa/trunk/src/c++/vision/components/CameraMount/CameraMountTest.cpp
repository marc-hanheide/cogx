/**
 * Test camera mount component.
 * Set some pan tilt angles to the pan tilt head and receive camera poses.
 *
 * Michael Zillich
 * Oct 2009
 */

#include "cogxmath.h"
#include "Video.hpp"
#include "VideoUtils.h"
#include "CameraMountTest.hpp"

using namespace std;
using namespace cogx::Math;
using namespace Video;
using namespace ptz;

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new CameraMountTest();
  }
}

void CameraMountTest::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
 
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

  if((it = _config.find("--ptzserver")) != _config.end())
  {
    ptzServerComponent = it->second;
  }

}

void CameraMountTest::start()
{
  m_PTUServer = getIceServer<PTZInterface>(ptzServerComponent);
}

void CameraMountTest::runComponent()
{
	MovePanTilt(pan, tilt, 5*M_PI/180);
}

void CameraMountTest::MovePanTilt(double pan, double tilt, double tolerance)
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
