/**
 * @author Michael Zillich
 * @date April 2011
*/

#include <iostream>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "BlueFSM.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cogx::BlueFSM();
  }
}

namespace cogx
{

  using namespace std;
  using namespace cast;
  
  BlueFSM::BlueFSM()
  {
  }
  
  void BlueFSM::configure(const map<string, string> &_config)
  {
    map<string,string>::const_iterator it;
    
  }
  
  void BlueFSM::start()
  {
    addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
                    new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::objectPoseCallback));
    addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject>(cdl::ADD),
                    new MemberFunctionChangeReceiver<BlueFSM>(this, &BlueFSM::objectPoseCallback));
  }
  
  void BlueFSM::destroy()
  {
  }
  
  void BlueFSM::runComponent()
  {
    while (true)
    {
      std::ostringstream oss;
      oss << pose_.pos.x;
      log(oss.str());
      sleep(2);
    }
  }
  
  void BlueFSM::objectPoseCallback(const cdl::WorkingMemoryChange &_wmc)
  {
  //log("received objectPoseCallback");

  VisionData::VisualObjectPtr vo = getMemoryEntry<VisionData::VisualObject>(_wmc.address);

  unsigned m_idx = std::distance(vo->identDistrib.begin(), std::max_element(vo->identDistrib.begin(), vo->identDistrib.end()));
  if (vo->identLabels.at(m_idx) == "cereals1_model")
  {
    //boost::unique_lock<boost::mutex> lock(mutex_, boost::try_to_lock_t());
    //boost::unique_lock<boost::mutex> lock(mutex_);
    pose_ = vo->pose;
  }


  //log("finished objectPoseCallback");
}

  
}
