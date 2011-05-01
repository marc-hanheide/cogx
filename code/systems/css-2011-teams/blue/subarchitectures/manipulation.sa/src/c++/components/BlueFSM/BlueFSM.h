#ifndef BLUE_FSM_H
#define BLUE_FSM_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include <manipulation.hpp>
#include <boost/thread/mutex.hpp>
#include <Navigation/LocalGridMap.hh>
#include <FrontierInterface.hpp>

namespace cogx
{
  typedef Cure::LocalGridMap<unsigned char> CureObstMap;
  
  using namespace std;
  using namespace cast;
  
  class BlueFSM : public ManagedComponent
  {
  private:
    
  protected:
    virtual void configure(const map<string, string> &_config);
    virtual void start();
    virtual void destroy();
    virtual void runComponent();
    
    void objectPoseCallback(const cdl::WorkingMemoryChange &_wmc);

    boost::mutex mutex_;
    cogx::Math::Pose3 pose_;
    
  private:
    bool findGraspPoses(double objX, double objY, double theta, double halfLength,
	double bestX, double bestY);

    FrontierInterface::PlaceInterfacePrx m_placeInterface;
    FrontierInterface::LocalMapInterfacePrx m_mapInterface;

  public:
    BlueFSM();
  };
  
}

#endif
