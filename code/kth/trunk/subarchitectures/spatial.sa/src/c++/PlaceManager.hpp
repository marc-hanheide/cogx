//
// = Filename
//   PlaceManager.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef PlaceManager_hpp
#define PlaceManager_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <SpatialData.hpp>
#include <NavData.hpp>
#include <string>
#include <list>

namespace spatial {

/**
 * Component that maintains Place structs on WM, and fills them with
 * the necessary information.
 *
 * @author Patric Jensfelt
 * @see
 */
class PlaceManager : public cast::ManagedComponent
{
public:
  /**
   * Constructor
   */
  PlaceManager();

  /**
   * Destructor
   */
  virtual ~PlaceManager();

  virtual void start();
  virtual void stop();
  virtual void runComponent();

protected:

  // Call back functions for nodes 
  void newNavNode(const cast::cdl::WorkingMemoryChange &objID);
  void modifiedNavNode(const cast::cdl::WorkingMemoryChange &objID);
  void deletedNavNode(const cast::cdl::WorkingMemoryChange &objID);

  class PlaceHolder {
  public:
    SpatialData::PlacePtr m_data; 
    std::string m_WMid;
  };

  // A vector of places
  std::vector<PlaceHolder> m_Places;

}; // class PlaceManager

}; // namespace spatial

#endif // PlaceManager_hpp
