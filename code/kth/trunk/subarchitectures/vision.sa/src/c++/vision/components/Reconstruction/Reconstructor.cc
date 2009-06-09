/**
 * @author Kai Zhou
 * @date April 2009
 */

#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <VideoUtils.h>
#include "Reconstructor.h"
#include "System.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::Reconstructor();
  }
}

namespace cast
{

using namespace std;
using namespace VisionData;

void Reconstructor::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
 
  // first let the base classes configure themselves
  configureVideoCommunication(_config);

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> camId;
  }
}

void Reconstructor::start()
{
  startVideoCommunication(*this);
}

void Reconstructor::runComponent()
{
	System s;
  while(isRunning())
  {
//////////////////////////////////////////////////////////////////////////
    Video::Image image;
    getImage(camId, image);
    IplImage *iplImage = convertImageToIpl(image);
    s.Run(iplImage);

    cvReleaseImage(&iplImage);
//////////////////////////////////////////////////////////////////////////
    // wait a bit so we don't hog the CPU
    sleepComponent(100);
  }
}

}

