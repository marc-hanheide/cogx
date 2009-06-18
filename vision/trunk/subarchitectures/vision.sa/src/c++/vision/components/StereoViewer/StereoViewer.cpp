/**
 * @author Michael Zillich
 * @date June 2009
 */

#include "StereoViewer.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::StereoViewer();
  }
}

namespace cast
{

using namespace std;
using namespace Stereo;
using namespace cogx;
using namespace cogx::Math;

void StereoViewer::configure(const map<string,string> & _config)
{
  // first let the base classes configure themselves
  configureStereoCommunication(_config);
}
void StereoViewer::start()
{
  startStereoCommunication(*this);
}

void StereoViewer::runComponent()
{
  while(isRunning())
  {
    vector<Vector3> points;
    getPoints(points);
    println("have %d points", (int)points.size());

    // wait a bit so we don't hog the CPU
    sleepComponent(100);
  }
}

}

