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
#include <boost/assign/list_of.hpp>


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
using namespace boost::assign;


// -------------------------------------------------------
ChainGraphInferencer::ChainGraphInferencer() :
		_worldStateChanged(true) // We will process the world state initially
{
	pthread_cond_init(&_inferenceQueryAddedSignalCond, 0);
	pthread_mutex_init(&_inferenceQueryAddedSignalMutex, 0);
	pthread_mutex_init(&_worldStateMutex, 0);
}


// -------------------------------------------------------
ChainGraphInferencer::~ChainGraphInferencer()
{
	pthread_cond_destroy(&_inferenceQueryAddedSignalCond);
	pthread_mutex_destroy(&_inferenceQueryAddedSignalMutex);
	pthread_mutex_destroy(&_worldStateMutex);
}


// -------------------------------------------------------
void ChainGraphInferencer::configure(const map<string,string> & _config)
{
	map<string,string>::const_iterator it;

	// QueryHandler name
	if((it = _config.find("--defaultchaingraphinferencer")) != _config.end())
	{
		_defaultChainGraphInferencerName = it->second;
	}

	log("Configuration parameters:");
	log("-> DefaultChainGraphInferencer name: %s", _defaultChainGraphInferencerName.c_str());
}


// -------------------------------------------------------
void ChainGraphInferencer::start()
{
	// Local filter on ConceptualData::InferenceResult
	addChangeFilter(createLocalTypeFilter<ConceptualData::InferenceQuery>(cdl::ADD),
			new MemberFunctionChangeReceiver<ChainGraphInferencer>(this,
					&ChainGraphInferencer::inferenceQueryAdded));

	// Filter on WorldState
	addChangeFilter(createLocalTypeFilter<ConceptualData::WorldState>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<ChainGraphInferencer>(this,
					&ChainGraphInferencer::worldStateChanged));

	// Get the DefaultChainGraphInferencer interface proxy
	_defaultChainGraphInferencerServerInterfacePrx =
			getIceServer<DefaultData::ChainGraphInferencerServerInterface>(_defaultChainGraphInferencerName);
}


// -------------------------------------------------------
void ChainGraphInferencer::runComponent()
{
	// Get all the default knowledge from the Default.SA
	getDefaultKnowledge();

	// Print the default knowledge factors
	for( map<string, SpatialProbabilities::ProbabilityDistribution>::iterator it =
			_defaultKnowledgeFactors.begin(); it!=_defaultKnowledgeFactors.end(); ++it)
	{
		debug("----------- %s -----------", it->second.description.c_str());
		for (map<string, int>::iterator it2 = it->second.variableNameToPositionMap.begin();
				it2!=it->second.variableNameToPositionMap.end(); ++it2)
		{
			debug("%s->%d", it2->first.c_str(), it2->second);
		}
		for (unsigned int i=0; i<it->second.massFunction.size(); ++i)
		{
			SpatialProbabilities::JointProbabilityValue jpv = it->second.massFunction[i];
			string var1Val;
			string var2Val;
			//debug("%s", jpv.variableValues[0]->ice_id().c_str());
			if (jpv.variableValues[0]->ice_isA("::SpatialProbabilities::StringRandomVariableValue"))
				var1Val=SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(jpv.variableValues[0])->value;
			else if (jpv.variableValues[0]->ice_isA("::SpatialProbabilities::BoolRandomVariableValue"))
				var1Val= (SpatialProbabilities::BoolRandomVariableValuePtr::
						dynamicCast(jpv.variableValues[0])->value)?"true":"false";
			if (jpv.variableValues[1]->ice_isA("::SpatialProbabilities::StringRandomVariableValue"))
				var2Val=SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(jpv.variableValues[1])->value;
			else if (jpv.variableValues[1]->ice_isA("::SpatialProbabilities::BoolRandomVariableValue"))
				var2Val= (SpatialProbabilities::BoolRandomVariableValuePtr::
						dynamicCast(jpv.variableValues[1])->value)?"true":"false";
			debug("%s %s %f", var1Val.c_str(), var2Val.c_str(), jpv.probability);
		}
	}

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

			// Update the factor graph if necessary
			bool factorGraphChanged = updateFactorGraph();

			// If the factor graph changed, re-run the inferences
			if (factorGraphChanged)
				runAllInferences();

			// Prepare the result
			ConceptualData::InferenceResultPtr inferenceResultPtr = new ConceptualData::InferenceResult();
			inferenceResultPtr->queryId = q.wmAddress.id;
			inferenceResultPtr->queryString = q.queryPtr->queryString;
			prepareInferenceResult(q.queryPtr->queryString, &(inferenceResultPtr->result));

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
void ChainGraphInferencer::worldStateChanged(const cast::cdl::WorkingMemoryChange & wmChange)
{
	// Lock the mutex to make the operations below atomic
	pthread_mutex_lock(&_worldStateMutex);

	// Get the local copy of the information from the world state
	lockEntry(wmChange.address, cdl::LOCKEDOD);
	ConceptualData::WorldStatePtr worldStatePtr;
	try
	{
		worldStatePtr = getMemoryEntry<ConceptualData::WorldState>(wmChange.address);
	}
	catch(CASTException &e)
	{
		log("Caught exception at %s. Message: %s. Will ignore the world state change!", __HERE__, e.message.c_str());
		unlockEntry(wmChange.address);
		pthread_mutex_unlock(&_worldStateMutex);
		return;
	}

	_worldStateRooms = worldStatePtr->rooms;
	_worldStateRoomConnections = worldStatePtr->roomConnections;

	unlockEntry(wmChange.address);

	// Signal that it changed
	_worldStateChanged=true;

	pthread_mutex_unlock(&_worldStateMutex);
}


// -------------------------------------------------------
bool ChainGraphInferencer::updateFactorGraph()
{
	bool factorGraphChanged = false;

	// Lock world state
	pthread_mutex_lock(&_worldStateMutex);

	// If world state changed, re-create the factor graph
	if (_worldStateChanged)
	{
		log("Updating the graph since the world state has changed!");

		factorGraphChanged = true;
		_worldStateChanged = false;


		// TODO: HERE RECREATE THE FACTOR GRAPH FROM WORLD STATE AND DEFAULT KNOWLEDGE
	}

	// Unlock world state
	pthread_mutex_unlock(&_worldStateMutex);

	return factorGraphChanged;
}


// -------------------------------------------------------
void ChainGraphInferencer::runAllInferences()
{
	log("Running all inferences on the graph!");

	// TODO: HERE just run all the inferences on a factor graph that is already created!
}


// -------------------------------------------------------
void ChainGraphInferencer::prepareInferenceResult(string queryString,
		SpatialProbabilities::ProbabilityDistribution *resultDistribution)
{
	debug("Preparing inference results for inference query '"+queryString+"'");

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


// -------------------------------------------------------
void ChainGraphInferencer::getDefaultKnowledge()
{
	log("Obtaining default knowledge factors from DefaultChainGraphInferencer.");

	DefaultData::StringSeq objectPropertyVariables =
		_defaultChainGraphInferencerServerInterfacePrx->getObjectPropertyVariables();

	string factorStr;

	// Get the room_category1, room_category2 factor
	factorStr = "f(room_category1,room_category2)";
	_defaultKnowledgeFactors[factorStr] =
			_defaultChainGraphInferencerServerInterfacePrx->getFactor(factorStr);

	// Get the room1_category -> object_xxx_property factors
	for(unsigned int i=0; i<objectPropertyVariables.size(); ++i)
	{
		factorStr = "f(room_category1,"+objectPropertyVariables[i]+")";
		_defaultKnowledgeFactors[factorStr] =
				_defaultChainGraphInferencerServerInterfacePrx->getFactor(factorStr);
	}

	// Get the room1_category -> shape_property factor
}


} // namespace def
