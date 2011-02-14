/**
 * @author Andrzej Pronobis
 *
 * Definition of the conceptual::QueryHandler class.
 */

// Conceptual.SA
#include "QueryHandler.h"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>

/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new conceptual::QueryHandler();
	}
}

namespace conceptual
{

using namespace std;
using namespace cast;


// -------------------------------------------------------
QueryHandler::QueryHandler()
{
	pthread_cond_init(&_queryAddedSignalCond, 0);
	pthread_mutex_init(&_queryAddedSignalMutex, 0);
}


// -------------------------------------------------------
QueryHandler::~QueryHandler()
{
	pthread_cond_destroy(&_queryAddedSignalCond);
	pthread_mutex_destroy(&_queryAddedSignalMutex);
}


// -------------------------------------------------------
void QueryHandler::configure(const map<string,string> & _config)
{
	// Register the ICE Server
	ConceptualData::QueryHandlerServerInterfacePtr queryHandlerServerInterfacePtr =
			new Server(this);
	registerIceServer<ConceptualData::QueryHandlerServerInterface,
			ConceptualData::QueryHandlerServerInterface>(queryHandlerServerInterfacePtr);
}


// -------------------------------------------------------
void QueryHandler::start()
{
	// Local filter on ConceptualData::InferenceResult
	addChangeFilter(createLocalTypeFilter<ConceptualData::InferenceResult>(cdl::ADD),
			new MemberFunctionChangeReceiver<QueryHandler>(this,
					&QueryHandler::inferenceResultAdded));
}


// -------------------------------------------------------
void QueryHandler::runComponent()
{
}


// -------------------------------------------------------
void QueryHandler::stop()
{
}

// -------------------------------------------------------
string QueryHandler::sendInferenceQuery(std::string queryString, bool imaginary)
{
	string inferenceQueryId = newDataID();
	ConceptualData::InferenceQueryPtr inferenceQueryPtr = new ConceptualData::InferenceQuery();
	inferenceQueryPtr->queryString = queryString;
	inferenceQueryPtr->imaginary = imaginary;

	_sentQueryIds.insert(inferenceQueryId);
	addToWorkingMemory<ConceptualData::InferenceQuery>(inferenceQueryId, inferenceQueryPtr);
	return inferenceQueryId;
}


// -------------------------------------------------------
void QueryHandler::retrieveInferenceResult(std::string queryId,
		 SpatialProbabilities::ProbabilityDistribution *resultDistribution)
{
	bool found = false;

	while (!found)
	{
		// Wait if necessary
		pthread_mutex_lock(&_queryAddedSignalMutex);

		// Any results received?
		if (_receivedResults.empty())
		{
			debug("retrieveInferenceResult waiting for query result!");

			// Nope, wait
			pthread_cond_wait(&_queryAddedSignalCond, &_queryAddedSignalMutex);
		}

		debug("retrieveInferenceResult received query result!");

		// Something must have arrived, check if that's what we want
		std::list<ConceptualData::InferenceResultPtr>::iterator rrIt;
		for (rrIt=_receivedResults.begin(); rrIt!=_receivedResults.end(); ++rrIt)
		{
			ConceptualData::InferenceResultPtr inferenceResultPtr = *rrIt;
			if (inferenceResultPtr->queryId == queryId)
			{ // Yes, we found what we want
				found = true;
				(*resultDistribution) = inferenceResultPtr->result;
				_receivedResults.erase(rrIt);
				break;
			}
		}

		pthread_mutex_unlock(&_queryAddedSignalMutex);
	}
}

// -------------------------------------------------------
void QueryHandler::inferenceResultAdded(const cast::cdl::WorkingMemoryChange &wmChange)
{
	// Get pointer to the queryresult
	ConceptualData::InferenceResultPtr inferenceResultPtr;
	try
	{
		inferenceResultPtr =
				getMemoryEntry<ConceptualData::InferenceResult>(wmChange.address);
	}
	catch(CASTException &e)
	{
		log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
		return;
	}

	debug("Received InferenceResult for query '"+inferenceResultPtr->queryString+"'!");

	// Lock the mutex to make the operations below atomic
	pthread_mutex_lock(&_queryAddedSignalMutex);

	// Check if the query result is on our waiting list.
	if (_sentQueryIds.find(inferenceResultPtr->queryId) == _sentQueryIds.end())
	{ // It's not on our list, skip it
		pthread_mutex_unlock(&_queryAddedSignalMutex);
		return;
	}

	debug("The InferenceResult is a response to a query that we've sent!");

	// Now we know it's on our list, removed from sent list, get a local copy and remove from WM
	_sentQueryIds.erase(inferenceResultPtr->queryId);
	_receivedResults.push_back(inferenceResultPtr);
	try
	{
		deleteFromWorkingMemory(wmChange.address);
	}
	catch(CASTException &e)
	{
		log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
	}

	// Signal that new result arrived
	pthread_cond_signal(&_queryAddedSignalCond);
	pthread_mutex_unlock(&_queryAddedSignalMutex);
}


// -------------------------------------------------------
SpatialProbabilities::ProbabilityDistribution QueryHandler::Server::query(
		const std::string &queryStr, const Ice::Current &)
{
	_queryHandler->debug("Received query: '"+queryStr+"'");

	// Send a new InferenceQuery
	string queryId = _queryHandler->sendInferenceQuery(queryStr, false);

	// Retrieve the inference result (blocking!)
	SpatialProbabilities::ProbabilityDistribution d;
	_queryHandler->retrieveInferenceResult(queryId, &d);

	_queryHandler->debug("Returning inference result for query: '"+queryStr+"'");

	// Return the distribution
	return d;
}


// -------------------------------------------------------
SpatialProbabilities::ProbabilityDistribution QueryHandler::Server::imaginaryQuery(
		const std::string &queryStr, const Ice::Current &)
{
	_queryHandler->debug("Received imaginary query: '"+queryStr+"'");

	// Send a new InferenceQuery
	string queryId = _queryHandler->sendInferenceQuery(queryStr, true);

	// Retrieve the inference result (blocking!)
	SpatialProbabilities::ProbabilityDistribution d;
	_queryHandler->retrieveInferenceResult(queryId, &d);

	_queryHandler->debug("Returning inference result for query: '"+queryStr+"'");

	// Return the distribution
	return d;
}


} // namespace def
