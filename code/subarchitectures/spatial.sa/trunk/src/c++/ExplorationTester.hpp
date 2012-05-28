//
// = Filename
//   ExplorationTester.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#ifndef ExplorationTester_hpp
#define ExplorationTester_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <vector>
#include <queue>
#include <SpatialData.hpp>
#include <NavData.hpp>
#include <string>

using namespace std;

namespace spatial {
class ExplorationTester: public cast::ManagedComponent {
public:
  /**
   * Constructor
   */
  ExplorationTester();

  /**
   * Destructor
   */
  virtual ~ExplorationTester();

  virtual void start();
  virtual void stop();
  virtual void runComponent();
  virtual void configure(const std::map<std::string, std::string>& _config);

protected:
  map<string, SpatialData::PlacePtr> m_places;
  queue<pair<SpatialData::PlacePtr, SpatialData::PlacePtr> > m_explorationPairs;
  queue<SpatialData::PlacePtr> m_newPlaces;

  void newPlace(const cast::cdl::WorkingMemoryChange &);
  void placeDeleted(const cast::cdl::WorkingMemoryChange &);
  void PlaceChanged(const cast::cdl::WorkingMemoryChange &);

  bool firstPoseRead;

  NavData::FNodePtr getCurrentNavNode();

  SpatialData::PathTransitionProbRequestPtr
  doTransitionQuery(int startPlaceID, int goalPlaceID);
};
}
;

#endif //ExplorationTester_hpp
