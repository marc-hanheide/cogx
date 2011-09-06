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
	// ---------------------------------------------------
	// Query stuff	
	// ---------------------------------------------------

	/** Interface to the def::QueryHandler::Server. */
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
	// Graph structure stuff	
	// ---------------------------------------------------

	sequence<string> StringSeq;

	/** Interface to the def::ChainGraphInferencer::Server. */
	interface ChainGraphInferencerServerInterface
	{
		/** Returns the names of the object-related variables. */
		StringSeq getObjectPropertyVariables();

		/** Returns the names of all the object categories. */
		StringSeq getObjectCategories();

		/** Returns the names of all the room categories. */
		StringSeq getRoomCategories();

		/** Returns the names of all the shapes. */
		StringSeq getShapes();

		/** Returns the names of all the sizes. */
		StringSeq getSizes();

		/** Returns the names of all the appearances. */
		StringSeq getAppearances();

		/** Returns the names of all the possible human assertions. */
		StringSeq getHumanAssertions();

		/** Returns the factor identified by a factor description string f(variables). 
		    We are using ProbabilityDistribution as a data container here for convenience. */
		SpatialProbabilities::ProbabilityDistribution getFactor(string factorStr);
	};

};

#endif // DEFAULTDATA_ICE
