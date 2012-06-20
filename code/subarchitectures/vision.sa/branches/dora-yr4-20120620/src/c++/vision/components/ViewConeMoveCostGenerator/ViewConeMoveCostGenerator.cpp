#include "ViewConeMoveCostGenerator.hpp"

#include <cast/architecture/ChangeFilterFactory.hpp>

using namespace cast;
using namespace VisionData;
using namespace castutils;

extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new Vision::ViewConeMoveCostGenerator();
  }
}

namespace Vision {
  
  ViewConeMoveCostGenerator::ViewConeMoveCostGenerator()  {}
  

  void ViewConeMoveCostGenerator::start() {    
    m_viewCones = new WMView<ViewCone>(WorkingMemoryReaderComponentPtr::dynamicCast(getComponentPointer()));
    m_viewCones->registerChangeHandler(WMView<ViewCone>::ChangeHandlerPtr::dynamicCast(getComponentPointer()));
    m_viewCones->start();
  }
  
  void ViewConeMoveCostGenerator::entryChanged(const WMView<ViewCone> & _map,
					       const cdl::WorkingMemoryChange & _wmc,  
					       ChangeHandler<ViewCone>::StoredTypePtr _newEntry,
					       ChangeHandler<ViewCone>::StoredTypePtr _oldEntry) 
    throw (cast::CASTException) {
    switch (_wmc.operation) {
    case cdl::ADD:
      updateCostList();
      break;
    case cdl::OVERWRITE:
      //ignoring overwrites
      break;
    case cdl::DELETE:
      updateCostList();
      break;        
    default:
      break;
    }
  }
  
  void ViewConeMoveCostGenerator::updateCostList() {
    log("updating cost map on change to viewcones");
    
    m_costList = new ViewConeMoveCostList();
    
    //iterate through all pairs of viewcones
    for(WMView<ViewCone>::const_iterator from = m_viewCones->begin();
	from != m_viewCones->end(); ++from) {
      
      WMView<ViewCone>::const_iterator to = from;
      to++;
      
      while(to != m_viewCones->end()) {      
	println("cost between %s and %s",from->first.id.c_str(),to->first.id.c_str());
	double cost = calculateCost(from->second,to->second);
	m_costList->costs.push_back(new ViewConeMoveCost(new cdl::WorkingMemoryPointer(from->first,
										       cast::typeName<ViewCone>()),
							 new cdl::WorkingMemoryPointer(to->first,
										       cast::typeName<ViewCone>()),
							 cost));                                                                                         
	to++;      
      }
    }
    
    if(m_costListID.empty()) {
      m_costListID = newDataID();
      addToWorkingMemory(m_costListID,m_costList);
    }
    else {
      overwriteWorkingMemory(m_costListID,m_costList);
    }    
  }
  
  ///Calculate cost to transition between the two viewcones as follows:
  ///
  ///Costs are based on time.
  ///
  ///if the anchor points are different calculate cost
  double ViewConeMoveCostGenerator::calculateCost(ViewConePtr _from, ViewConePtr _to) const {    
    return 1;
  }
};
