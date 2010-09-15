#ifndef CONCEPTUALDATA_ICE
#define CONCEPTUALDATA_ICE

#include <cast/slice/CDL.ice>
#include <default.sa/slice/DefaultData.ice>

/**
 * Data structures representing the knowledge stored in the conceptual layer
 * and interfaces used to access that knowledge.
 * @author Andrzej Pronobis
 */

module ConceptualData 
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

	/** Interface of the ConceptualQueryHandlerInterface */
	interface QueryHandlerInterface
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
