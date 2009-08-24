/**
 * @file VisionSystem3.h
 * @author Andreas Richtsfeld
 * @date April 2008
 * @version 0.1
 * @brief Management component for running simple object detector (vs3)
 */

#include <opencv/highgui.h>
#include <VideoUtils.h>
#include <VisionSystem3.h>

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::VisionSystem3();
  }
}

namespace cast
{

using namespace std;
using namespace VisionData;

void VisionSystem3::configure(const map<string,string> & _config)
{
	log("VisionSystem3::configure: Running configuration");
  map<string,string>::const_iterator it;
 
  // first let the base classes configure themselves
  configureVideoCommunication(_config);

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> camId;
  }

	vs3Interface = new Vs3Interface();
}

void VisionSystem3::start()
{
	log("VisionSystem3::start: Start Component");
  startVideoCommunication(*this);
  cvNamedWindow(getComponentID().c_str(), 1);
}

void VisionSystem3::runComponent()
{
	log("VisionSystem3::runComponent");
  while(isRunning())
  {
     Video::Image image;
     getImage(camId, image);
     sleepComponent(1000);			/// TODO 

    IplImage *iplImage = convertImageToIpl(image);

		// ----------------------------------
		// Process images with vision system3 
		// ----------------------------------
		log("ProcessSingleImage");
// 		Z::Image *zImage = convertImageToZImage(image);
//		zImage->SavePPM("zImage.ppm");
		vs3Interface->ProcessSingleImage(iplImage);
		// -------------------------------

    cvShowImage(getComponentID().c_str(), iplImage);

    // needed to make the window appear
    // (an odd behaviour of OpenCV windows!)
    cvWaitKey(10);
    cvReleaseImage(&iplImage);

    // wait a bit so we don't hog the CPU
    sleepComponent(100);
  }
}

}

