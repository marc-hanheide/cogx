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
 
  // first let the base classes configure themselves
  configureVideoCommunication(_config);

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> camId;
  }
}

void VideoViewer::start()
{
  startVideoCommunication(*this);
  cvNamedWindow(getComponentID().c_str(), 1);
}

void VideoViewer::runComponent()
{
  while(isRunning())
  {
    Video::Image image;
    getImage(camId, image);
    IplImage *iplImage = convertImageToIpl(image);

    //vector<Video::Image> images;
    //getImages(images);
    //IplImage *iplImage = convertImageToIpl(images[0]);

    cvShowImage(getComponentID().c_str(), iplImage);

    // needed to make the window appear
    // (an odd behaviour of OpenCV windows!)
    cvWaitKey(10);
    cvReleaseImage(&iplImage);

    // wait a bit so we don't hog the CPU
    sleepComponent(100);
  }
  cvDestroyWindow(getComponentID().c_str());
}

}

