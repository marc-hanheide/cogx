#ifndef BLUE_FSM_H
#define BLUE_FSM_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include <boost/thread/mutex.hpp>

namespace cogx
{
  
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
    
  public:
    BlueFSM();
  };
  
}

#endif
