/**
 * @author Andrzej Pronobis
 *
 * Definition of the conceptual::ChainGraphInferencer class.
 */

// Conceptual.SA
#include "ChainGraphInferencer.h"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Boost
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new conceptual::ChainGraphInferencer();
	}
}

namespace conceptual
{

using namespace std;
using namespace cast;
using namespace boost;

// -------------------------------------------------------
ChainGraphInferencer::ChainGraphInferencer()
{
	pthread_cond_init(&_inferenceQueryAddedSignalCond, 0);
	pthread_mutex_init(&_inferenceQueryAddedSignalMutex, 0);
}


// -------------------------------------------------------
ChainGraphInferencer::~ChainGraphInferencer()
{
	pthread_cond_destroy(&_inferenceQueryAddedSignalCond);
	pthread_mutex_destroy(&_inferenceQueryAddedSignalMutex);
}


// -------------------------------------------------------
void ChainGraphInferencer::configure(const map<string,string> & _config)
{
/*	map<string,string>::const_iterator it;

	// QueryHandler name
	if((it = _config.find("--test-qh")) != _config.end())
	{
		_queryHandlerName = it->second;
	}

	log("Configuration parameters:");
	log("-> Test QueryHandler: %s", (_queryHandlerName.empty())?"No":"Yes");
	log("-> QueryHandler Name: %s", _queryHandlerName.c_str());*/
}


// -------------------------------------------------------
void ChainGraphInferencer::start()
{
	// Local filter on ConceptualData::InferenceResult
	addChangeFilter(createLocalTypeFilter<ConceptualData::InferenceQuery>(cdl::ADD),
			new MemberFunctionChangeReceiver<ChainGraphInferencer>(this,
					&ChainGraphInferencer::inferenceQueryAdded));
}


// -------------------------------------------------------
void ChainGraphInferencer::runComponent()
{
	// Run component
	while(isRunning())
	{
		// Get current time and add 1 sec
		timespec ts;
		timeval tv;
		gettimeofday(&tv, NULL);
		ts.tv_sec = tv.tv_sec + 1;
		ts.tv_nsec = 0;

		// Wait if necessary
		pthread_mutex_lock(&_inferenceQueryAddedSignalMutex);

		// Is there anything in the queue?
		if (_receivedQueries.empty())
			pthread_cond_timedwait(&_inferenceQueryAddedSignalCond, &_inferenceQueryAddedSignalMutex, &ts);

		// Handle signal if signal arrived
		if ((!isRunning()) || (_receivedQueries.empty()))
			pthread_mutex_unlock(&_inferenceQueryAddedSignalMutex);
		else
		{
			// Get the query
			Query q = _receivedQueries.front();
			_receivedQueries.pop_front();

			pthread_mutex_unlock(&_inferenceQueryAddedSignalMutex);

			debug("Processing inference query '"+q.queryPtr->queryString+"'");

			// Create empty result
			ConceptualData::InferenceResultPtr inferenceResultPtr = new ConceptualData::InferenceResult();
			inferenceResultPtr->queryId = q.wmAddress.id;
			inferenceResultPtr->queryString = q.queryPtr->queryString;

			// Process the query
			runInference(q.queryPtr->queryString, &(inferenceResultPtr->result));

			// Return result
			debug("Sending out inference result for query '"+q.queryPtr->queryString+"'");
			string inferenceResultId = newDataID();
			addToWorkingMemory<ConceptualData::InferenceResult>(inferenceResultId, inferenceResultPtr);

		} // if
	} // while*/
}


// -------------------------------------------------------
void ChainGraphInferencer::stop()
{
}


// -------------------------------------------------------
void ChainGraphInferencer::inferenceQueryAdded(const cast::cdl::WorkingMemoryChange &wmChange)
{
	// Get the inference query
	ConceptualData::InferenceQueryPtr inferenceQueryPtr;
	try
	{
		inferenceQueryPtr =
				getMemoryEntry<ConceptualData::InferenceQuery>(wmChange.address);
	}
	catch(CASTException &e)
	{
		log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
	}

	// Lock the mutex to make the operations below atomic
	pthread_mutex_lock(&_inferenceQueryAddedSignalMutex);

	Query q;
	q.queryPtr = inferenceQueryPtr;
	q.wmAddress = wmChange.address;
	_receivedQueries.push_back(q);

	debug("Received inference query '"+inferenceQueryPtr->queryString+"'");

	// Signal to make an inference and update coma room structs.
	pthread_cond_signal(&_inferenceQueryAddedSignalCond);
	pthread_mutex_unlock(&_inferenceQueryAddedSignalMutex);
}


// -------------------------------------------------------
void ChainGraphInferencer::runInference(std::string queryString,
		SpatialProbabilities::ProbabilityDistribution *resultDistribution)
{
	// Check the query string
	if ( (queryString.length()<4) || (queryString[0]!='p') ||
		 (queryString[1]!='(') || (queryString[queryString.length()-1]!=')') )
		throw CASTException("Malformed ChainGraphInferencer query string '"+queryString+"'");

	// Extract variable names
	string variableString=queryString;
	erase_head(variableString, 2);
	erase_tail(variableString, 1);
	vector<string> variables;
	split( variables, variableString, is_any_of(", ") );


	// Ignore the string for now, always return this:
	resultDistribution->description = queryString;
	resultDistribution->variableNameToPositionMap[variables[0]]=0;
	resultDistribution->massFunction.clear();

	SpatialProbabilities::StringRandomVariableValuePtr rvv1Ptr = new SpatialProbabilities::StringRandomVariableValue("kitchen");
	SpatialProbabilities::JointProbabilityValue jpv1;
	jpv1.probability=0.2;
	jpv1.variableValues.push_back(rvv1Ptr);
	resultDistribution->massFunction.push_back(jpv1);

	SpatialProbabilities::StringRandomVariableValuePtr rvv2Ptr = new SpatialProbabilities::StringRandomVariableValue("office");
	SpatialProbabilities::JointProbabilityValue jpv2;
	jpv2.probability=0.8;
	jpv2.variableValues.push_back(rvv2Ptr);
	resultDistribution->massFunction.push_back(jpv2);
}



} // namespace def
