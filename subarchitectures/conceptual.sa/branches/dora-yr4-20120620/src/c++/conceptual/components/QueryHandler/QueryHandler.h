/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::QueryQueryHandler class.
 */

#ifndef CONCEPTUAL_QUERYHANDLER_H
#define CONCEPTUAL_QUERYHANDLER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>


namespace conceptual
{


/**
 * @author Andrzej Pronobis
 *
 * Component that handles probabilistic queries
 * sent to the Conceptual.SA.
 */
class QueryHandler: public cast::ManagedComponent
{

	class Server: public ConceptualData::QueryHandlerServerInterface
	{
	public:

		/** Constructor. */
		Server(QueryHandler *queryHandler) : _queryHandler(queryHandler)
		{}

		virtual ConceptualData::ProbabilityDistributions
			query(const std::string &queryStr, const Ice::Current &);

		virtual ConceptualData::ProbabilityDistributions
			imaginaryQuery(const std::string &queryStr, const Ice::Current &);

		virtual ConceptualData::ProbabilityDistributions
			factorQuery(const std::string &factorStr, const Ice::Current &);

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
	std::string sendInferenceQuery(std::string queryString, ConceptualData::QueryType type);

	/** Waits for the InferenceResult to a query with the given ID.
	 * Retuns pointer to the distribution in the result. */
	void retrieveInferenceResult(std::string queryId,
			ConceptualData::ProbabilityDistributions &resultDistributions);


	pthread_cond_t _queryAddedSignalCond;
	pthread_mutex_t _queryAddedSignalMutex;

	/** Set of WM Ids of sent querries. */
	std::set<std::string> _sentQueryIds;

	/** Recently received results. */
	std::list<ConceptualData::InferenceResultPtr> _receivedResults;

}; // class ConceptualQueryHandler
} // namespace def

#endif // CONCEPTUAL_QUERYHANDLER_H



