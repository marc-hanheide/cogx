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

SOIPtr Reconstructor::createObj(unsigned int i, const Video::Image &image, Vector<3> center, Vector<3> size, double radius)
{
	VisionData::SOIPtr obs = new VisionData::SOI;
	obs->boundingBox.pos.x = obs->boundingSphere.pos.x = center[0];
	obs->boundingBox.pos.y = obs->boundingSphere.pos.y = center[1];
	obs->boundingBox.pos.z = obs->boundingSphere.pos.z = center[2];
	obs->boundingBox.size.x = size[0];
	obs->boundingBox.size.y = size[1];
	obs->boundingBox.size.z = size[2];
	obs->boundingSphere.rad = radius;

	
	return obs;
}

void Reconstructor::postObjectToWM(const vector<string> & labels, const Video::Image &image, Vector<3> center, Vector<3> size, double radius)
{
  for(unsigned int i = 0; i < model_labels.size(); i++)
    if(find(labels.begin(), labels.end(), model_labels[i]) != labels.end())
      postObjectToWM_Internal(i, image, center,size,radius);
}

void Reconstructor::postObjectToWM_Internal(unsigned int i, const Video::Image &image, Vector<3> center, Vector<3> size, double radius)
{
  SOIPtr obj = createObj(i, image, center, size, radius);

  // if no WM ID yet for that object
  if(objWMIds[i] == "")
  {
    objWMIds[i] = newDataID();
    addToWorkingMemory(objWMIds[i], obj);
  }
  else
  {
    overwriteWorkingMemory(objWMIds[i], obj);
  }
}

}

