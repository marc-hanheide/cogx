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
	/** Interface to the conceptual::QueryHandler::Server. */
	interface QueryHandlerServerInterface
	{
		SpatialProbabilities::ProbabilityDistribution query(string queryStr);
	};

	/** Sequence of place IDs. */
	sequence<long> PlaceIdSeq;

	/** Relevant information about a coma room. */
	struct ComaRoomInfo
	{
		cast::cdl::WorkingMemoryAddress wmAddress;
    	int roomId;
	    PlaceIdSeq containedPlaceIds;
	};

	/** Map of room IDs to room structs. */
	dictionary<int, ComaRoomInfo> RoomSet;
	
	/** State of the world obtained from other SAs. */
	class WorldState
	{
		RoomSet rooms;
	};

};

#endif // CONCEPTUALDATA_ICE
