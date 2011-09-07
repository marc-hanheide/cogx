#ifndef CONCEPTUALDATA_ICE
#define CONCEPTUALDATA_ICE

#include <cast/slice/CDL.ice>
#include <SpatialProbabilities.ice>
#include <SpatialData.ice>


/**
 * Data structures representing the knowledge stored in the conceptual layer
 * and interfaces used to access that knowledge.
 * @author Andrzej Pronobis
 */

module ConceptualData 
{
	// ---------------------------------------------------
	// Consts
	// ---------------------------------------------------

	const string EXISTS = "exists";
	const string NOTEXISTS = "not-exists";

	// ---------------------------------------------------
	// Testing
	// ---------------------------------------------------

	sequence<string> VariableValues;
	sequence<int> VariableIds;
	
	struct VariableInfo
	{
		string name;
		bool observed;
		VariableValues values;
	};
	
	dictionary<int, VariableInfo> VariableInfos;

	struct FactorInfo
	{
		string name;
		VariableIds variables;		
	};
	
	dictionary<int, FactorInfo> FactorInfos;
	
	/** Interface to the conceptual::ChainGraphInferencer::TestingServer. */
	interface ChainGraphTestingServerInterface
	{
		VariableInfos getVariables();
		FactorInfos getFactors();
	};


	// ---------------------------------------------------
	// Query stuff	
	// ---------------------------------------------------
	sequence<SpatialProbabilities::ProbabilityDistribution> ProbabilityDistributions;

	/** Interface to the conceptual::QueryHandler::Server. */
	interface QueryHandlerServerInterface
	{
		/** Query for a probability distribution extracted from the conceptual layer modeling real world. */
		ProbabilityDistributions query(string queryStr);
		/** Query for a probability distribution extracted from the conceptual layer modeling imaginary worlds. */
		ProbabilityDistributions imaginaryQuery(string queryStr);
		/** Query for a probability distribution of a single factor. */
		ProbabilityDistributions factorQuery(string factorStr);
	};

	enum QueryType {STANDARDQUERY, IMAGINARYQUERY, FACTORQUERY};

	/** Query sent to the chain graph inferencer. */ 
	class InferenceQuery
	{
		QueryType type; 
		string queryString;
	};
	
	/** Result of a single inference. */
	class InferenceResult
	{
		/** Query string used to generate the result. */
		string queryString;
		
		/** WM Id of the inference query to which that result corresponds. */
		string queryId;
		
		/** Result of the inference. */
		ProbabilityDistributions results;
	};

	// ---------------------------------------------------
	// WorldState Stuff	
	// ---------------------------------------------------

	/** Structure representing information about object place properties. */
	struct ObjectPlacePropertyInfo
	{
		/** The category of the object. */
		string category;
        /** Category of the possible support object. If the relation does not require a support object, it's empty. */
        string supportObjectCategory;
        /** Id of the possible support object. If the relation does not require a support object, it's empty. */
        string supportObjectId;
        /** Spatial relation for this object. */
        SpatialData::SpatialRelation relation;
        /** Number of objects of this type found. */
        int count;
        /** The percentage of the relation probability mass that was already explored for this object category. */
        double beta;
    };

	/** */
	sequence<ObjectPlacePropertyInfo> ObjectPlacePropertyInfos;

	struct ValuePotentialPair
	{
		string value;
		double potential;
	};
	
	sequence<ValuePotentialPair> ValuePotentialPairs;

	struct ShapePlacePropertyInfo
	{
		ValuePotentialPairs distribution;
	};
	
	sequence<ShapePlacePropertyInfo> ShapePlacePropertyInfos;

	struct SizePlacePropertyInfo
	{
		ValuePotentialPairs distribution;
	};
	
	sequence<SizePlacePropertyInfo> SizePlacePropertyInfos;

	struct AppearancePlacePropertyInfo
	{
		ValuePotentialPairs distribution;
	};
	
	sequence<AppearancePlacePropertyInfo> AppearancePlacePropertyInfos;

	struct HumanAssertionPlacePropertyInfo
	{
		string assertion;
	};
	
	sequence<HumanAssertionPlacePropertyInfo> HumanAssertionPlacePropertyInfos;

	/** Relevant information about a place. */
	struct PlaceInfo
	{
        int placeId;
		ObjectPlacePropertyInfos objectProperties;
		ShapePlacePropertyInfos shapeProperties;
		SizePlacePropertyInfos sizeProperties;
		AppearancePlacePropertyInfos appearanceProperties;
		HumanAssertionPlacePropertyInfos humanAssertionProperties;
	};

	/** Sequence of PlaceInfos. */
	sequence <PlaceInfo> PlaceInfos;

	struct GatewayPlaceholderPropertyInfo
	{
		double gatewayProbability;
	};
	
	sequence<GatewayPlaceholderPropertyInfo> GatewayPlaceholderPropertyInfos;

	struct AssociatedSpacePlaceholderPropertyInfo
	{
		double associatedSpace;
	};
	
	sequence<AssociatedSpacePlaceholderPropertyInfo> AssociatedSpacePlaceholderPropertyInfos;

	/** Relevant information about a placeholder. Currently only id. */
	struct PlaceholderInfo
	{
        int placeholderId;
        GatewayPlaceholderPropertyInfos gatewayProperties;
        AssociatedSpacePlaceholderPropertyInfos associatedSpaceProperties;
	};

	/** Sequence of placeholder infos. */
	sequence<PlaceholderInfo> PlaceholderInfos;

	/** Relevant information about a coma room. */
	struct ComaRoomInfo
	{
		/** Address required by the ComaRoomUpdater. */
		cast::cdl::WorkingMemoryAddress wmAddress; 
        int roomId;
		
		/** Places in that room. */
        PlaceInfos places;

		/** Placeholder infos. */
		PlaceholderInfos placeholders;

        /** Object properties not assigned to any places in this room. */
		ObjectPlacePropertyInfos objectProperties;
	};

	/** Sequence of ComaRoomInfos. */
	sequence<ComaRoomInfo> ComaRoomInfos;

	/** Room connectivity information. */
	struct RoomConnectivityInfo
	{
		/** This ID is always smaller than room2Id. */
		int room1Id;
		/** This ID is always larger than room1Id. */
		int room2Id;
	};

	/** List of room id pairs. */
	sequence<RoomConnectivityInfo> RoomConnectivityInfos;

	enum EventType 
	{
		// 0
		EventNothig, 
		// 1 2
		// Info: room id
		EventRoomAdded, EventRoomDeleted,  
		// 3 4
		// Info roomId, placeId
		EventRoomPlaceAdded, EventRoomPlaceDeleted, 
		// 5 6
		EventPlaceholderAdded, EventPlaceholderDeleted,
		// 7 
		// Info placeId
		EventGatewayPlacePropertyChanged, 
		// 8 9 10
		// placeId, propertyNo
		EventObjectPlacePropertyAdded, EventObjectPlacePropertyDeleted, EventObjectPlacePropertyChanged, 
		// 11 12 13
		// placeId, propertyNo
		EventObjectSearchResultAdded, EventObjectSearchResultDeleted, EventObjectSearchResultChanged, 
		// 14 15 16
		// placeid
		EventSizePlacePropertyAdded, EventSizePlacePropertyDeleted, EventSizePlacePropertyChanged, 
		// 17 18 19
		// placeid
		EventShapePlacePropertyAdded, EventShapePlacePropertyDeleted, EventShapePlacePropertyChanged, 
		// 20 21 22
		// placeid
		EventAppearancePlacePropertyAdded, EventAppearancePlacePropertyDeleted, EventAppearancePlacePropertyChanged, 
		// 23
		// place1id, place2id
		EventRoomConnectivityChanged, 
		// 24 25 26
		EventGatewayPlaceholderPropertyAdded, EventGatewayPlaceholderPropertyChanged, EventGatewayPlaceholderPropertyDeleted,
		// 27 28 29
		EventAssociatedSpacePlaceholderPropertyAdded, EventAssociatedSpacePlaceholderPropertyChanged, EventAssociatedSpacePlaceholderPropertyDeleted, 
		// 30 31 32
		EventHumanAssertionPlacePropertyAdded, EventHumanAssertionPlacePropertyDeleted, EventHumanAssertionPlacePropertyChanged
	};
	
	struct EventInfo
	{
		EventType type;
		double time;	
		int roomId;
		int place1Id;
		int place2Id;
		string propertyInfo;
	};
	
	sequence<EventInfo> EventInfos;
	
	/** State of the world obtained from other SAs. */
	class WorldState
	{
		ComaRoomInfos rooms;
		RoomConnectivityInfos roomConnections;
		EventInfos lastEvents;
	};

};

#endif // CONCEPTUALDATA_ICE
