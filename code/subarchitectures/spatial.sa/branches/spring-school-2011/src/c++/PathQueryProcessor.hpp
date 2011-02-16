//
// = Filename
//   PathQueryProcessor.hpp
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

#ifndef PathQueryProcessor_hpp
#define PathQueryProcessor_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <SpatialData.hpp>
#include <NavData.hpp>
#include <string>
#include <list>
#include <Navigation/NavGraph.hh>
#include <FrontierInterface.hpp>

namespace spatial {

/**
 * This class processes query structs (and perhaps, eventually,
 * interface calls) to produce probability estimates and cost estimates
 * for paths.
 *
 * @author Patric Jensfelt
 * @see
 */
class PathQueryProcessor : public cast::ManagedComponent

{
public:
  /**
   * Constructor
   */
  PathQueryProcessor();

  /**
   * Destructor
   */
  virtual ~PathQueryProcessor();

  virtual void start();
  virtual void stop();
  virtual void runComponent();

protected:
  virtual void configure(const std::map<std::string, std::string>& _config);
  void newNavGraph(const cast::cdl::WorkingMemoryChange &objID);
  void newPathTransitionProbRequest(const cast::cdl::WorkingMemoryChange &objID);
  void newPathTransitionCostRequest(const cast::cdl::WorkingMemoryChange &objID);

  bool m_GotGraph;
  bool m_noIndirectPaths;
  IceUtil::Mutex m_GraphMutex;
  Cure::NavGraph m_NavGraph;

  FrontierInterface::PlaceInterfacePrx m_placeInterface;
}; // class PathQueryProcessor

}; // namespace spatial

#endif // PathQueryProcessor_hpp
