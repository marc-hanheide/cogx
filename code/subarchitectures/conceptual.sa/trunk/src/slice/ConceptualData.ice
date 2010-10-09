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
		SpatialProbabilities::ProbabilityDistribution query(string queryStr);
	};

	/** Query sent to the chain graph inferencer. */ 
	class InferenceQuery
	{
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

	/** Relevant information about a place. */
	struct PlaceInfo
	{
    	int placeId;
		ObjectPlacePropertyInfos objectProperties;
	};

	/** Sequence of PlaceInfos. */
	sequence <PlaceInfo> PlaceInfos;

	/** Relevant information about a coma room. */
	struct ComaRoomInfo
	{
		/** Address required by the ComaRoomUpdater. */
		cast::cdl::WorkingMemoryAddress wmAddress; 
    	int roomId;
		
		/** Places in that room. */
	    PlaceInfos places;
	};

	/** Sequence of ComaRoomInfos. */
	sequence<ComaRoomInfo> ComaRoomInfos;


	/** State of the world obtained from other SAs. */
	class WorldState
	{
		ComaRoomInfos rooms;
	};

};

#endif // CONCEPTUALDATA_ICE
