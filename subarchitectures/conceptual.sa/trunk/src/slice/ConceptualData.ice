#ifndef CONCEPTUALDATA_ICE
#define CONCEPTUALDATA_ICE

#include <cast/slice/CDL.ice>
#include <DefaultData.ice>

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
		DefaultData::DiscreteProbabilityDistribution query(string queryStr);
	};

	/** State of the world obtained from other SAs. */
	struct WorldState
	{
		int dummy;
	};

};

#endif // CONCEPTUALDATA_ICE
