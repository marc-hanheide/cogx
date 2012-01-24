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
#include "SpatialProbabilities.hpp"

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
	Observer();

	/** Destructor. */
	virtual ~Observer();


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
	void sizePlacePropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Change event. */
	void appearancePlacePropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Change event. */
	void humanAssertionPlacePropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Change event. */
	void connectivityPathPropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Change event. */
	void objectSearchResultChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Change event. */
	void gatewayPlaceholderPropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Change event. */
	void associatedSpacePlaceholderPropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange);


private:

	/** Checks if the place is a true place based on the _placeWmAddressMap */
	bool isTruePlace(int placeId);

	/** Checks if the place is a placeholder based on the _placeWmAddressMap */
	bool isPlaceholder(int placeId);

	/** Returns true if the placeholder is already added to the ComaRoomInfo. */
	bool isPlaceholderPresent(const ConceptualData::ComaRoomInfo &cri, int placeholderId);

	/** Returns room index (in the list) of the room to which a placeholder is already added. */
	int isPlaceholderInRooms(const std::vector<ConceptualData::ComaRoomInfo> &cris, int placeholderId);

	/** Returns true if the place has a gateway property set. */
	bool isGatewayPlace(int placeId);

	/** Returns all object properties of the place. */
	void getObjectPlaceProperties(int placeId, std::vector<SpatialProperties::ObjectPlacePropertyPtr> &properties);

	/** Returns all object search results for a room. */
	void getObjectSearchResults(int roomId,
			std::vector<SpatialData::ObjectSearchResultPtr> &results);

	/** Returns all object properties of the place. */
	void getShapePlaceProperties(int placeId, std::vector<SpatialProperties::RoomShapePlacePropertyPtr> &properties);

	/** Returns all object properties of the place. */
	void getSizePlaceProperties(int placeId, std::vector<SpatialProperties::RoomSizePlacePropertyPtr> &properties);

	/** Returns all gateway properties of the placeholder. */
	void getGatewayPlaceholderProperties(int placeId, std::vector<SpatialProperties::GatewayPlaceholderPropertyPtr> &properties);

	void getAssociatedSpacePlaceholderProperties(int placeId, std::vector<SpatialProperties::AssociatedSpacePlaceholderPropertyPtr> &properties);

	/** Returns all object properties of the place. */
	void getAppearancePlaceProperties(int placeId, std::vector<SpatialProperties::RoomAppearancePlacePropertyPtr> &properties);

	/** Returns all object properties of the place. */
	void getHumanAssertionPlaceProperties(int placeId, std::vector<SpatialProperties::RoomHumanAssertionPlacePropertyPtr> &properties);

	/** Returns id of the room to which the given place belongs. Returns -1 if not found. */
	int getRoomForPlace(int placeId);

	/** Returns a list of Ids of places connected to the given place.
	 * traversedPlaces used only for recursion. */
	void getConnectedPlaces(int placeId, std::vector<int> *connectedPlaces,
			std::set<int> *traversedPlaces = 0);

	bool areWorldStatesDifferent(ConceptualData::WorldStatePtr ws1, ConceptualData::WorldStatePtr ws2);

	double calculateDistributionDifference(SpatialProperties::ProbabilityDistributionPtr dist1,
			SpatialProperties::ProbabilityDistributionPtr dist2);

	double calculateBinaryDistributionDifference(SpatialProperties::ProbabilityDistributionPtr dist1,
			SpatialProperties::ProbabilityDistributionPtr dist2);

	double castTimeToSeconds(const cast::cdl::CASTTime &time);

private:


	/** Map of coma room wmAddress -> ComaRoom. */
	std::map<cast::cdl::WorkingMemoryAddress, comadata::ComaRoomPtr> _comaRoomWmAddressMap;

	/** Map of place wmAddress -> Place*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialData::PlacePtr> _placeWmAddressMap;

	/** Map of place wmAddress -> GatewayPlaceProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::GatewayPlacePropertyPtr> _gatewayPlacePropertyWmAddressMap;

	/** Map of place wmAddress -> ObjectPlaceProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::ObjectPlacePropertyPtr> _objectPlacePropertyWmAddressMap;

	/** Map of place wmAddress -> ObjectSearchResult*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialData::ObjectSearchResultPtr> _objectSearchResultWmAddressMap;
	std::map<cast::cdl::WorkingMemoryAddress, SpatialData::ObjectSearchResultPtr> _acceptedObjectSearchResultWmAddressMap;

	/** Map of place wmAddress -> ShapePlaceProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::RoomShapePlacePropertyPtr> _shapePlacePropertyWmAddressMap;
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::RoomShapePlacePropertyPtr> _acceptedShapePlacePropertyWmAddressMap;

	/** Map of place wmAddress -> SizePlaceProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::RoomSizePlacePropertyPtr> _sizePlacePropertyWmAddressMap;
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::RoomSizePlacePropertyPtr> _acceptedSizePlacePropertyWmAddressMap;

	/** Map of place wmAddress -> AppearancePlaceProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::RoomAppearancePlacePropertyPtr> _appearancePlacePropertyWmAddressMap;
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::RoomAppearancePlacePropertyPtr> _acceptedAppearancePlacePropertyWmAddressMap;

	/** Map of place wmAddress -> HumanAssertionPlaceProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::RoomHumanAssertionPlacePropertyPtr> _humanAssertionPlacePropertyWmAddressMap;
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::RoomHumanAssertionPlacePropertyPtr> _acceptedHumanAssertionPlacePropertyWmAddressMap;

	/** Map of place wmAddress -> ConnectivityPathProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::ConnectivityPathPropertyPtr> _connectivityPathPropertyWmAddressMap;

	/** Map of place wmAddress -> GatewayPlaceholderProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::GatewayPlaceholderPropertyPtr> _gatewayPlaceholderPropertyWmAddressMap;
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::GatewayPlaceholderPropertyPtr> _acceptedGatewayPlaceholderPropertyWmAddressMap;

	/** Map of place wmAddress -> AssociatedSpacePlaceholderProperty*/
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::AssociatedSpacePlaceholderPropertyPtr> _associatedSpacePlaceholderPropertyWmAddressMap;
	std::map<cast::cdl::WorkingMemoryAddress, SpatialProperties::AssociatedSpacePlaceholderPropertyPtr> _acceptedAssociatedSpacePlaceholderPropertyWmAddressMap;

	/** Current state of the world as much as
	 * the conceptual.sa is concerned. */
	ConceptualData::WorldStatePtr _worldStatePtr;

	/** Working memory ID of the world state struct. */
	std::string _worldStateId;

	pthread_mutex_t _worldStateMutex;

	std::vector<ConceptualData::EventInfo> _accumulatedEvents;
	double _lastWsUpdateTime;

	double _shapeThreshold;
	double _sizeThreshold;
	double _appearanceThreshold;
	double _gatewayThreshold;
	double _associatedSpaceThreshold;
	double _betaThreshold;
	bool _includePlaceholderInfo;

}; // class Observer
} // namespace

#endif // CONCEPTUAL_OBSERVER_H




