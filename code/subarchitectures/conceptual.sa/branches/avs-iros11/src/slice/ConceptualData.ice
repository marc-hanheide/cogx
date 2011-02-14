#ifndef CONCEPTUALDATA_ICE
#define CONCEPTUALDATA_ICE

#include <cast/slice/CDL.ice>
#include <SpatialProbabilities.ice>


/**
 * Data structures representing the knowledge stored in the conceptual layer
 * and interfaces used to access that knowledge.
 * @author Andrzej Pronobis
 */

module ConceptualData 
{
	// ---------------------------------------------------
	// Query stuff	
	// ---------------------------------------------------

	/** Interface to the conceptual::QueryHandler::Server. */
	interface QueryHandlerServerInterface
	{
		/** Query for a probability distribution extracted from the conceptual layer modeling real world. */
		SpatialProbabilities::ProbabilityDistribution query(string queryStr);
		/** Query for a probability distribution extracted from the conceptual layer modeling imaginary worlds. */
		SpatialProbabilities::ProbabilityDistribution imaginaryQuery(string queryStr);
	};

	/** Query sent to the chain graph inferencer. */ 
	class InferenceQuery
	{
		bool imaginary; // If true, the query will refer to imaginary worlds
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
		SpatialProbabilities::ProbabilityDistribution result;
	};

	// ---------------------------------------------------
	// WorldState Stuff	
	// ---------------------------------------------------

	/** Structure representing information about object place properties. */
	struct ObjectPlacePropertyInfo
	{
		string category;
		/** True if the object is present in a place. False when it is not. */
		bool present; 
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

	struct AppearancePlacePropertyInfo
	{
		ValuePotentialPairs distribution;
	};
	
	sequence<AppearancePlacePropertyInfo> AppearancePlacePropertyInfos;


	/** Relevant information about a place. */
	struct PlaceInfo
	{
    	int placeId;
		ObjectPlacePropertyInfos objectProperties;
		ShapePlacePropertyInfos shapeProperties;
		AppearancePlacePropertyInfos appearanceProperties;
	};

	/** Sequence of PlaceInfos. */
	sequence <PlaceInfo> PlaceInfos;

	/** Relevant information about a placeholder. Currently only id. */
	struct PlaceholderInfo
	{
    	int placeholderId;
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

	/** State of the world obtained from other SAs. */
	class WorldState
	{
		ComaRoomInfos rooms;
		RoomConnectivityInfos roomConnections;
	};

};

#endif // CONCEPTUALDATA_ICE
