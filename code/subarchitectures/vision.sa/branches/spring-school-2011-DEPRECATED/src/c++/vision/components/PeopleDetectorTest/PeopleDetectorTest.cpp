
#include <highgui.h>
#include <VideoUtils.h>
#include "PeopleDetectorTest.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::PeopleDetectorTest();
  }
}

namespace cast
{

using namespace std;
using namespace VisionData;

void PeopleDetectorTest::configure(const map<string,string> & config)
{
  map<string,string>::const_iterator it;
  
  if((it = config.find("--videoname")) != config.end()) {
    videoServerName = it->second;
  }
  
  if ((it = config.find("--camid")) != config.end()) {
    std::istringstream str(it->second);
    str >> camId;
  }
  
  // sanity checks: Have all important things be configured? Is the
  // configuration consistent?
  if(videoServerName.empty()) {
    throw runtime_error(exceptionMessage(__HERE__, "no video server name given"));
  }
}
  
  std::string IntToStr(float i)
{
    std::ostringstream ss;
    ss << i;
    return ss.str();
}

void PeopleDetectorTest::start()
{

  // get connection to the video server
  m_videoServer = getIceServer<Video::VideoInterface>(videoServerName);

  cvNamedWindow(getComponentID().c_str(), 1);
  
  addChangeFilter(cast::createLocalTypeFilter<VisionData::Person>(cast::cdl::ADD),
    new cast::MemberFunctionChangeReceiver<PeopleDetectorTest>(this, &PeopleDetectorTest::newPerson));  
    
  addChangeFilter(cast::createLocalTypeFilter<VisionData::Person>(cast::cdl::OVERWRITE),
    new cast::MemberFunctionChangeReceiver<PeopleDetectorTest>(this, &PeopleDetectorTest::movePerson));  

  addChangeFilter(cast::createLocalTypeFilter<VisionData::Person>(cast::cdl::DELETE),
    new cast::MemberFunctionChangeReceiver<PeopleDetectorTest>(this, &PeopleDetectorTest::losePerson));          
}

void PeopleDetectorTest::newPerson(const cast::cdl::WorkingMemoryChange & _wmc)
{
    VisionData::PersonPtr person = getMemoryEntry<VisionData::Person>(_wmc.address);       
    people[_wmc.address.id] = TempPerson(person->locX, person->locZ);
    
    println("Got one");
}

void PeopleDetectorTest::movePerson(const cast::cdl::WorkingMemoryChange & _wmc)
{
    VisionData::PersonPtr person = getMemoryEntry<VisionData::Person>(_wmc.address);       
    TempPerson t = people[_wmc.address.id];
    t.X = person->locX;
    t.Z = person->locZ;    
    people[_wmc.address.id] = t;
    
    println("Moved");
}

void PeopleDetectorTest::losePerson(const cast::cdl::WorkingMemoryChange & _wmc)
{
    people.erase(_wmc.address.id);    
    println("Gone");
}


int count;
void PeopleDetectorTest::runComponent()
{
  count = 0;
  while(isRunning())
  {
    Video::Image image;
    m_videoServer->getImage(camId, image);
    IplImage *iplImage = convertImageToIpl(image);
    
    for (int y = 0; y < iplImage->height; y++)
    {
        for (int x = 0; x < iplImage->width; x++)
        {
            CvScalar s;
            s = cvGet2D(iplImage, y, x);
            int t = s.val[0];
            s.val[0] = s.val[2];
            s.val[2] = t;
            cvSet2D(iplImage, y, x, s);
        }
    }
    
    for (map<string, TempPerson>::iterator i = people.begin(); i != people.end(); i++)
    {
        int pos = int((i->second.Z * iplImage->width) / (i->second.X) + iplImage->width * 0.525);
        int hei = int((0 / (i->second.X)) + (iplImage->height) / (i->second.X) + iplImage->height * 0.625);       
        int l = hei - 2 * iplImage->height / i->second.X;        
        if (l < 0) l = 0;       

        if (pos > 0 && pos < iplImage->width)
            for (int y = l; y < hei && y < iplImage->height; y++)
            {
                CvScalar s2 = i->second.colour;

                cvSet2D(iplImage, y, pos, s2);
            }
    }

    cvShowImage(getComponentID().c_str(), iplImage);    
    //cvSaveImage(("./out/" + IntToStr((float)count++) +".bmp").c_str(), iplImage);

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

