

#ifndef PEOPLE_TEST_H
#define PEOPLE_TEST_H

#include <string>
#include <map>
#include <cmath>

#include <cast/architecture/ManagedComponent.hpp>
#include <cast/architecture.hpp>
#include <VideoClient.h>
#include <VisionData.hpp>

namespace cast
{

struct TempPerson
{
    double X, Z;
    CvScalar colour;
    
    TempPerson() {}
    
    TempPerson(double a, double b)
    {
        X = a;
        Z = b;
        colour = cvScalar(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
    }
};

class PeopleDetectorTest : public VideoClient,
                    public ManagedComponent
{
private:

  /**
   * component ID of the video server to connect to
   */
  std::string videoServerName;
  /**
   * Which camera to get images from
   */
  int camId;
  
  /**
   * our Ice proxy to the video server
   */
  Video::VideoInterfacePrx m_videoServer;
  
  
  std::map<std::string, TempPerson> people;
  
protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent();
  
  void newPerson(const cast::cdl::WorkingMemoryChange & _wmc);
  void movePerson(const cast::cdl::WorkingMemoryChange & _wmc);
  void losePerson(const cast::cdl::WorkingMemoryChange & _wmc);  

public:

  PeopleDetectorTest() : camId(0) {}
  virtual ~PeopleDetectorTest() {}
};

}

#endif



