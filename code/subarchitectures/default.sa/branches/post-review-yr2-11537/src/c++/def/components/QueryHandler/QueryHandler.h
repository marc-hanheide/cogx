/**
 * @author Andrzej Pronobis
 *
 * Declaration of the def::QueryQueryHandler class.
 */

#ifndef DEFAULT_QUERYHANDLER_H
#define DEFAULT_QUERYHANDLER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <DefaultData.hpp>


namespace def
{


/**
 * @author Andrzej Pronobis
 *
 * Component that handles probabilistic queries
 * sent to the Default.SA.
 */
class QueryHandler: public cast::ManagedComponent
{

	class Server: public DefaultData::QueryHandlerServerInterface
	{
	public:

		/** Constructor. */
		Server(QueryHandler *queryHandler) : _queryHandler(queryHandler)
		{}

		virtual SpatialProbabilities::ProbabilityDistribution
			query(const std::string &queryStr, const Ice::Current &);

	private:

		/** Pointer to the owner of the server. */
		QueryHandler *_queryHandler;
    };


public:

	/** Constructor. */
	QueryHandler();

	/** Destructor. */
	virtual ~QueryHandler();


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

	/** Change event. */
	void inferenceResultAdded(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Adds new InferenceQuery to WM. Returns the WM ID of the query. */
	std::string sendInferenceQuery(std::string queryString);

	/** Waits for the InferenceResult to a query with the given ID.
	 * Retuns pointer to the distribution in the result. */
	void retrieveInferenceResult(std::string queryId,
			SpatialProbabilities::ProbabilityDistribution *resultDistribution);


	pthread_cond_t _queryAddedSignalCond;
	pthread_mutex_t _queryAddedSignalMutex;

	/** Set of WM Ids of sent querries. */
	std::set<std::string> _sentQueryIds;

	/** Recently received results. */
	std::list<DefaultData::InferenceResultPtr> _receivedResults;

}; // class DefaultQueryHandler
} // namespace def

#endif // DEFAULT_QUERYHANDLER_H



