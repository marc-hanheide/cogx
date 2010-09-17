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
	sequence<string> BindingRow;
   	sequence<BindingRow> BindingTable;
	dictionary<string, int> StringToIntMap;

	/** Results returned by the forward chainer 
	    in response to a QDL query. */
	struct QdlQueryResults 
	{
		string query;
		StringToIntMap varPosMap;
		BindingTable bt;
	};

	/** Interface of the forward chainer. */
	interface HFCInterface 
	{
		QdlQueryResults querySelect(string q);
	};

	/** Interface to the def::QueryHandler::Server. */
	interface QueryHandlerServerInterface
	{
		SpatialProbabilities::ProbabilityDistribution query(string queryStr);
	};
};

#endif // DEFAULTDATA_ICE
