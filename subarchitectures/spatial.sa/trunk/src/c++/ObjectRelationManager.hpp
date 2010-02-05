//
// = FILENAME
//    ObjectRelationManager.hpp
//
// = FUNCTION
//    Evaluate spatial relations between objects and between objects and places,
//    and write appropriate Property structs to Spatial WM
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#ifndef ObjectRelationManager_hpp
#define ObjectRelationManager_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <SpatialProperties.hpp>
#include <FrontierInterface.hpp>
#include <map>
#include <peekabot.hh>
#include "RelationEvaluation.hpp"

using namespace SpatialProperties;
using namespace SpatialData;

namespace spatial {

/**
 * This class monitors the sensory layer for objects and their poses, and writes
 * spatial relation structs to WM as appropriate
 *
 * @author Kristoffer Sjöö
 * @see
 */
class ObjectRelationManager : public cast::ManagedComponent
{
public:

  ObjectRelationManager ();
  virtual ~ObjectRelationManager();

  virtual void runComponent();
  virtual void start();

protected:
  int m_maxObjectCounter;

  std::map<int, SpatialObjectPtr> m_objects;
  std::map<int, PlaceContainmentObjectPropertyPtr> m_containmentProperties;

  FrontierInterface::PlaceInterfacePrx m_placeInterface;

  bool m_bTestOnness;

  bool m_bDisplayPlaneObjectsInPB;
  peekabot::PeekabotClient m_PeekabotClient;  
  peekabot::GroupProxy m_planeProxies;
  peekabot::GroupProxy m_onnessTester;
  std::string m_PbHost;
  int m_PbPort;
  int m_RetryDelay; // Seconds to retry if cannot connect. -1 means dont retry

  void connectPeekabot();

  virtual void configure(const std::map<std::string, std::string>& _config);

  void newObject(const cast::cdl::WorkingMemoryChange &);
  void objectChanged(const cast::cdl::WorkingMemoryChange &);

  void newPlaneObject(const cast::cdl::WorkingMemoryChange &);

  void setContainmentProperty(int objectID, int placeID, double confidence);
}; 

std::vector<cogx::Math::Vector3>
findPolygonIntersection(const std::vector<cogx::Math::Vector3> &polygon1, 
    const std::vector<cogx::Math::Vector3> &polygon2);

double getPolygonArea(const std::vector<cogx::Math::Vector3> &polygon);


}; // namespace spatial

#endif // ObjectRelationManager_hpp
