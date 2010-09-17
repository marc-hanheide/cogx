#ifndef DEFAULTDATA_ICE
#define DEFAULTDATA_ICE

#include <cast/slice/CDL.ice>
#include <SpatialProbabilities.ice>

/**
 * Data structures representing the default knowledge maintained by default.sa
 * and interfaces used to access that knowledge.
 * @author Andrzej Pronobis
 */

module DefaultData 
{
	/** Interface to the def::QueryHandler::Server. */
	interface QueryHandlerServerInterface
	{
		SpatialProbabilities::ProbabilityDistribution query(string queryStr);
	};
};

#endif // DEFAULTDATA_ICE
