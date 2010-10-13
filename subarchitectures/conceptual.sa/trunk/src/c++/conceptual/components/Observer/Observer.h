/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::Observer class.
 */

#ifndef CONCEPTUAL_OBSERVER_H
#define CONCEPTUAL_OBSERVER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>
#include "ComaData.hpp"
#include "SpatialData.hpp"
#include "SpatialProperties.hpp"

namespace conceptual
{

/**
 * @author Andrzej Pronobis
 *
 * Component observing changes on other working memories
 * which correspond to changes in spatial knowledge represented
 * by other layers of the representation.
 */
class Observer: public cast::ManagedComponent
{

public:

	/** Constructor. */
	Observer()
	{}

	/** Destructor. */
	virtual ~Observer()
	{}


protected:

	/** Called by the framework to configure the component. */
	virtual void configure(const std::map<std::string,std::string> & _config);

	/** Called by the framework after configuration, before run loop. */
	virtual void start();

	/** The main run loop. */
	virtual void runComponent();

	/** Called by the framework after the run loop finishes. */
	virtual void stop();


private:

	/** Initializes the world state. */
	void initializeWorldState();

	/** Adds the worldstate to the working memory. */
	void addInitialWorldState();

	/** Updates the world state on the WM. */
	void updateWorldState();

	/** Change event. */
	void comaRoomChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Change event. */
	void placeChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Change event. */
	void gatewayPlacePropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Change event. */
	void objectPlacePropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Change event. */
	void shapePlacePropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Change event. */
	void appearancePlacePropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Change event. */
	void connectivityPathPropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange);


private:

	/** Checks if the place is a true place based on the _placeWmAddressMap */
	bool isTruePlace(int placeId);

	/** Returns true if the place has a gateway property set. */
	bool isGatewayPlace(int placeId);

	/** Returns all object properties of the place. */
	void getObjectPlaceProperties(int placeId, std::vector<SpatialProperties::ObjectPlacePropertyPtr> &properties);

	/** Returns all object properties of the place. */
	void getShapePlaceProperties(int placeId, std::vector<SpatialProperties::RoomShapePlacePropertyPtr> &properties);

	/** Returns all object properties of the place. */
	void getAppearancePlaceProperties(int placeId, std::vector<SpatialProperties::RoomAppearancePlacePropertyPtr> &properties);

	/** Returns id of the room to which the given place belongs. Returns -1 if not found. */
	int getRoomForPlace(int placeId);

	/** Returns a list of Ids of places connected to the given place.
	 * traversedPlaces used only for recursion. */
	void getConnectedPlaces(int placeId, std::vector<int> *connectedPlaces,
			std::set<int> *traversedPlaces = 0);

	bool areWorldStatesDifferent(ConceptualData::WorldStatePtr ws1, ConceptualData::WorldStatePtr ws2);


private:


	/** Map of coma room wmAddress -> ComaRoom. */
	std::map<cast::cdl::WorkingMemoryAddress, comadata::ComaRoomPtr> _comaRoomWmAddressMap;

	/** Map of place wmAddress -> Place*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialData::PlacePtr> _placeWmAddressMap;

	/** Map of place wmAddress -> GatewayPlaceProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::GatewayPlacePropertyPtr> _gatewayPlacePropertyWmAddressMap;

	/** Map of place wmAddress -> ObjectPlaceProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::ObjectPlacePropertyPtr> _objectPlacePropertyWmAddressMap;

	/** Map of place wmAddress -> ShapePlaceProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::RoomShapePlacePropertyPtr> _shapePlacePropertyWmAddressMap;

	/** Map of place wmAddress -> AppearancePlaceProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::RoomAppearancePlacePropertyPtr> _appearancePlacePropertyWmAddressMap;

	/** Map of place wmAddress -> ConnectivityPathProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::ConnectivityPathPropertyPtr> _connectivityPathPropertyWmAddressMap;


	/** Current state of the world as much as
	 * the conceptual.sa is concerned. */
	ConceptualData::WorldStatePtr _worldStatePtr;

	/** Working memory ID of the world state struct. */
	std::string _worldStateId;


}; // class Observer
} // namespace

#endif // CONCEPTUAL_OBSERVER_H




