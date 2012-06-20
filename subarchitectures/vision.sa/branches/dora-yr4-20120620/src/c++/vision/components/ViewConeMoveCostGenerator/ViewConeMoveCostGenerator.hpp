/**
 * Calculates the cost to move the robot and ptu from one viewcone to another
 * Oct 2009
 */

#ifndef VIEW_CONE_MOVE_COST_GENERATOR_HPP
#define VIEW_CONE_MOVE_COST_GENERATOR_HPP

#include "VisionData.hpp"

#include "castextensions/WMView.hpp"

#include <cast/architecture/ManagedComponent.hpp>




namespace Vision {


//  typedef cast::StringMap<ViewConePtr>::map StringViewConeMap;
//  typedef cast::StringMap<float>::map StringFloatMap;
//  typedef cast::StringMap<StringFloatMap>::map StringStringFloatMap;
  
class ViewConeMoveCostGenerator : 
  public cast::ManagedComponent, 
  public castutils::ChangeHandler<VisionData::ViewCone> {

public:
  ViewConeMoveCostGenerator();
  virtual ~ViewConeMoveCostGenerator() {}

    virtual void entryChanged(const castutils::WMView<VisionData::ViewCone> & _map,
                              const cast::cdl::WorkingMemoryChange & _wmc,
                              castutils::ChangeHandler<VisionData::ViewCone>::StoredTypePtr _newEntry,
                              castutils::ChangeHandler<VisionData::ViewCone>::StoredTypePtr _oldEntry) 
    throw (cast::CASTException);
    
protected:
  //  virtual void configure(const std::map<std::string, std::string>& _config) {}
  virtual void start();
  //virtual void runComponent() {}

private:
  
  ///Updates the cost map and writes it to WM
  void updateCostList();
  
  double calculateCost(VisionData::ViewConePtr _from, VisionData::ViewConePtr _to) const;
  
//  //list of ViewCones added to memory
//  std::list< cast::CASTData<VisionData::ViewCone> > m_viewCones;
//
//  
//  //wm id of from viewcone to id to target viewcone to cost
//  StringStringFloatMap m_costMap;  

  std::string m_costListID;

  //list of move costs created on major changes and written to wm
  VisionData::ViewConeMoveCostListPtr m_costList;

  //using a pointer to make delayed construction easier
  castutils::WMView<VisionData::ViewCone>::WMViewPtr m_viewCones;
  
};	
	
};
  
#endif
