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
#include <FrontierInterface.hpp>
#include <string>
#include <list>
#include <map>

namespace spatial {

/**
 * Component that maintains Place structs on WM, and fills them with
 * the necessary information.
 * 
 * Parameters: --max-frontier-dist x : Culling distance for frontiers around a certain node
 *		--min-frontier-length x : Minimum size of exploration frontiers to consider
 *		--min-node-separation x : Spread of new hypothetical targets around a node
 *
 * @author Patric Jensfelt
 * @see
 */
class PlaceManager : public cast::ManagedComponent
{
  private:
    class PlaceServer: public FrontierInterface::PlaceInterface {
      virtual SpatialData::PlacePtr getPlaceFromNodeID(int nodeID,
	  const Ice::Current &_context);
      virtual SpatialData::PlacePtr getPlaceFromHypID(int hypID,
	  const Ice::Current &_context);
      virtual FrontierInterface::NodeHypothesisPtr getHypFromPlaceID(int placeID,
	  const Ice::Current &_context);
      virtual NavData::FNodePtr getNodeFromPlaceID(int placeID,
	  const Ice::Current &_context);
      virtual void beginPlaceTransition(int goalPlaceID, const Ice::Current &_context);
      virtual void endPlaceTransition(int failed, const Ice::Current &_context);
      PlaceManager *m_pOwner;
      PlaceServer(PlaceManager *owner) : m_pOwner(owner)
      {}
      friend class PlaceManager;
    };
    friend class PlaceServer;
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
    virtual void configure(const std::map<std::string, std::string>& _config);

  protected:

    // Call back functions for nodes 
    void newNavNode(const cast::cdl::WorkingMemoryChange &objID);
    void modifiedNavNode(const cast::cdl::WorkingMemoryChange &objID);
    void deletedNavNode(const cast::cdl::WorkingMemoryChange &objID);

    // Call back functions for edges 
    void newEdge(const cast::cdl::WorkingMemoryChange &objID);
    void modifiedEdge(const cast::cdl::WorkingMemoryChange &objID);

    void evaluateUnexploredPaths();
    NavData::FNodePtr getCurrentNavNode();
    void beginPlaceTransition(int goalPlaceID);
    void endPlaceTransition(int failed);

    class PlaceHolder {
      public:
	SpatialData::PlacePtr m_data; 
	std::string m_WMid;
    };

    // A map of places and IDs
    std::map<int, PlaceHolder> m_Places;

    long m_placeIDCounter;

  private:
    SpatialData::PlacePtr getPlaceFromNodeID(int nodeID);
    SpatialData::PlacePtr getPlaceFromHypID(int hypID);
    FrontierInterface::NodeHypothesisPtr getHypFromPlaceID(int placeID);
    NavData::FNodePtr getNodeFromPlaceID(int placeID);

    // Callback function for metric movement
    void robotMoved(const cast::cdl::WorkingMemoryChange &objID);
    void processPlaceArrival(bool failed, NavData::FNodePtr newNavNode=0); 

    // Frontier based exploration path parameters
    double m_maxFrontierDist; // Culling distance for frontiers around a given node
    double m_minFrontierDist; // Culling distance for frontiers around a given node
    double m_minFrontierLength; // Minimum size of frontiers to consider
    double m_minNodeSeparation; // Min distance that has to exist between the new
    // hypothetical nodes generated at a place
    double m_hypPathLength;     // How far to try and move in the direction of the
    // frontier

    long m_hypIDCounter;
    std::map<int, NavData::FNodePtr> m_PlaceIDToNodeMap;
    std::map<int, FrontierInterface::NodeHypothesisPtr> m_PlaceIDToHypMap;
    std::map<int, std::string> m_HypIDToWMIDMap;
    bool m_isPathFollowing;
    int m_startNodeForCurrentPath; // During transitions, stores where the robot
    				   // came from last
    int m_goalPlaceForCurrentPath; // During transitions, stores where the robot
    				   // thought it was going
    std::map<int, std::set<int> > m_connectivities;
    FrontierInterface::FrontierReaderPrx frontierReader;
}; // class PlaceManager

}; // namespace spatial

#endif // PlaceManager_hpp
