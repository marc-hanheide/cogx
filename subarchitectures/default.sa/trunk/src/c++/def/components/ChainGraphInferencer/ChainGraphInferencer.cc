/**
 * @author Andrzej Pronobis
 *
 * Definition of the def::ChainGraphInferencer class.
 */

// Default.SA
#include "ChainGraphInferencer.h"
// ROCS
#include <rocs/core/Configuration.h>
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Boost
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new def::ChainGraphInferencer();
	}
}

namespace def
{

using namespace std;
using namespace cast;
using namespace boost;
using boost::lexical_cast;

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
	map<string,string>::const_iterator it;

	// Forward chainer server name
	if((it = _config.find("--hfcserver")) != _config.end())
	{
		_hfcServerName = it->second;
	}
	// Config file name
	string configFileName;
	if((it = _config.find("--config")) != _config.end())
	{
		configFileName = it->second;
	}

	// Read the config file
	rocs::core::Configuration config;
	config.addConfigFile(configFileName);
	_defaultRoomCategoryConnectivityPotential =
			config.getValue("default.default_room_category_connectivity_potential", 0.0);
	_defaultObjectExistenceProbability =
			config.getValue("default.default_object_existence_probability", 0.0);

	vector<string> roomCat1Vector;
	config.getValueList("default.room_category_connectivity", "room_category1", roomCat1Vector);
	vector<string> roomCat2Vector;
	config.getValueList("default.room_category_connectivity", "room_category2", roomCat2Vector);
	vector<double> potentialVector;
	config.getValueList("default.room_category_connectivity", "potential", potentialVector);

	if ( (roomCat1Vector.size()!=roomCat2Vector.size()) ||
		 (roomCat1Vector.size()!=potentialVector.size()) )
		throw CASTException("The numbers of room_category1, room_category2 and potential keys do not much.");

	for(unsigned int i=0; i<roomCat1Vector.size(); ++i)
	{
		RoomCategoryConnectivity rcc;
		rcc.roomCategory1=roomCat1Vector[i];
		rcc.roomCategory2=roomCat2Vector[i];
		rcc.potential=potentialVector[i];
		_roomCategoryConnectivity.push_back(rcc);
	}

	// Register the ICE Server
	DefaultData::ChainGraphInferencerServerInterfacePtr chainGraphInferencerServerInterfacePtr =
			new Server(this);
	registerIceServer<DefaultData::ChainGraphInferencerServerInterface,
			DefaultData::ChainGraphInferencerServerInterface>(chainGraphInferencerServerInterfacePtr);

	log("Configuration parameters:");
	log("-> HFCServer Name: %s", _hfcServerName.c_str());
	log("-> default_room_category_connectivity_potential: %f",
			_defaultRoomCategoryConnectivityPotential);
	log("-> default_object_existence_probability: %f",
			_defaultObjectExistenceProbability);

	log("-> Room category connectivity:");
	for(list<RoomCategoryConnectivity>::iterator it = _roomCategoryConnectivity.begin();
			it!=_roomCategoryConnectivity.end(); ++it)
	{
		log("----> rc1=%s rc2=%s p=%f",
				it->roomCategory1.c_str(), it->roomCategory2.c_str(), it->potential);
	}
}



// -------------------------------------------------------
void ChainGraphInferencer::start()
{
	// Get the HFCServer interface proxy
	_hfcInterfacePrx =
			getIceServer<comadata::HFCInterface>(_hfcServerName);

	// Local filter on DefaultData::InferenceResult
	addChangeFilter(createLocalTypeFilter<DefaultData::InferenceQuery>(cdl::ADD),
			new MemberFunctionChangeReceiver<ChainGraphInferencer>(this,
					&ChainGraphInferencer::inferenceQueryAdded));

	// Read the relevant information from the HFC of Default.SA
	_hfcQueryResults =
			_hfcInterfacePrx->querySelect("SELECT ?x ?y ?p where ?x <dora:in> ?y ?p");

	log ("Received default knowledge from the HFC Server");

	// Get variable columns
	int roomColumn = _hfcQueryResults.varPosMap["?y"];
	int objectColumn = _hfcQueryResults.varPosMap["?x"];
	int probabilityColumn = _hfcQueryResults.varPosMap["?p"];

	// Print the knowledge out
//	for (unsigned int i=0; i<_hfcQueryResults.bt.size(); ++i)
//	{
//		debug("room=%s object=%s p=%s", _hfcQueryResults.bt[i][roomColumn].c_str(),
//				_hfcQueryResults.bt[i][objectColumn].c_str(),
//				_hfcQueryResults.bt[i][probabilityColumn].c_str());
//	}

	// Convert the output to more efficient format
	// Typical string values:
	// room=<dora:box_of_nails> object=<dora:toilet> p="0.25800696"^^<xsd:float>
	debug("-> Default knowledge obtained from HFC:");
	for (unsigned int i=0; i<_hfcQueryResults.bt.size(); ++i)
	{
		std::string room = _hfcQueryResults.bt[i][roomColumn];
		std::string object = _hfcQueryResults.bt[i][objectColumn];
		std::string probability = _hfcQueryResults.bt[i][probabilityColumn];

		replace_first(room, "<dora:", "");
		replace_last(room, ">", "");
		replace_first(object, "<dora:", "");
		replace_last(object, ">", "");
		replace_first(probability, "\"", "");
		replace_last(probability, "\"^^<xsd:float>", "");

		HFCItem hfcItem;
		hfcItem.room = room;
		hfcItem.object = object;
		hfcItem.probability = lexical_cast<double>(probability);
		_hfcKnowledge.push_back(hfcItem);

		debug("---> room=%s object=%s p=%f", hfcItem.room.c_str(), hfcItem.object.c_str(), hfcItem.probability);
	}

	// Generate list of object variables
	set<string> objectPropertyVariables;
	for(list<HFCItem>::iterator it = _hfcKnowledge.begin();
			it!=_hfcKnowledge.end(); ++it)
		objectPropertyVariables.insert("object_"+it->object+"_property");
	_objectPropertyVariables = vector<string>(objectPropertyVariables.begin(), objectPropertyVariables.end());

	// Generate list of room categories
	set<string> roomCategories;
	for(list<HFCItem>::iterator it = _hfcKnowledge.begin();
			it!=_hfcKnowledge.end(); ++it)
		roomCategories.insert(it->room);
	_roomCategories = vector<string>(roomCategories.begin(), roomCategories.end());
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

			// Prepare the result
			DefaultData::InferenceResultPtr inferenceResultPtr = new DefaultData::InferenceResult();
			inferenceResultPtr->queryId = q.wmAddress.id;
			inferenceResultPtr->queryString = q.queryPtr->queryString;

			// Return result
			debug("Sending out inference result for query '"+q.queryPtr->queryString+"'");
			string inferenceResultId = newDataID();
			addToWorkingMemory<DefaultData::InferenceResult>(inferenceResultId, inferenceResultPtr);

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
	DefaultData::InferenceQueryPtr inferenceQueryPtr;
	try
	{
		inferenceQueryPtr =
				getMemoryEntry<DefaultData::InferenceQuery>(wmChange.address);
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
DefaultData::StringSeq
	ChainGraphInferencer::Server::getObjectPropertyVariables(const Ice::Current &)
{
	return _chainGraphInferencer->_objectPropertyVariables;
}


// -------------------------------------------------------
DefaultData::StringSeq
	ChainGraphInferencer::Server::getRoomCategories(const Ice::Current &)
{
	return _chainGraphInferencer->_roomCategories;
}


// -------------------------------------------------------
SpatialProbabilities::ProbabilityDistribution
	ChainGraphInferencer::Server::getFactor(const std::string &factorStr, const Ice::Current &)
{
	// Extract variable names
	string variableString=factorStr;
	erase_head(variableString, 2);
	erase_tail(variableString, 1);
	vector<string> variables;
	split( variables, variableString, is_any_of(", ") );

	// Output factor
	SpatialProbabilities::ProbabilityDistribution factor;
	factor.description=factorStr;
	for(unsigned int i=0; i<variables.size(); ++i)
		factor.variableNameToPositionMap[variables[i]]=i;

	// Connectivity factor
	if ((variables.size()==2) &&
		(variables[0]=="room_category1") && (variables[1]=="room_category2"))
	{
		// Set of all connectivities between all rooms
		set< pair<string, string> > catConnectivities;
		for ( vector<string>::iterator it = _chainGraphInferencer->_roomCategories.begin();
			  it!=_chainGraphInferencer->_roomCategories.end(); ++it )
		{
			for ( vector<string>::iterator it2 = _chainGraphInferencer->_roomCategories.begin();
				  it2!=_chainGraphInferencer->_roomCategories.end(); ++it2 )
			{
				catConnectivities.insert(pair<string,string>((*it), (*it2)));
			}
		}

		// Add the connectivities from the data file
		for(list<RoomCategoryConnectivity>::iterator it =
				_chainGraphInferencer->_roomCategoryConnectivity.begin();
				it!=_chainGraphInferencer->_roomCategoryConnectivity.end(); ++it)
		{
			catConnectivities.erase(pair<string, string>(it->roomCategory1, it->roomCategory2));

			SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->roomCategory1);
			SpatialProbabilities::StringRandomVariableValuePtr roomCategory2RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->roomCategory2);
			SpatialProbabilities::JointProbabilityValue jpv;
			jpv.probability=it->potential;
			jpv.variableValues.push_back(roomCategory1RVVPtr);
			jpv.variableValues.push_back(roomCategory2RVVPtr);
			factor.massFunction.push_back(jpv);
		}

		// Add default potential for those connectivities that we don't know
		for (set< pair<string, string> >::iterator it=catConnectivities.begin();
				it!=catConnectivities.end(); ++it)
		{
			SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->first);
			SpatialProbabilities::StringRandomVariableValuePtr roomCategory2RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->second);
			SpatialProbabilities::JointProbabilityValue jpv;
			jpv.probability=_chainGraphInferencer->_defaultRoomCategoryConnectivityPotential;
			jpv.variableValues.push_back(roomCategory1RVVPtr);
			jpv.variableValues.push_back(roomCategory2RVVPtr);
			factor.massFunction.push_back(jpv);
		}

		return factor;
	}


	// room_category1 -> object_xxx_property factors
	if ((variables.size()==2) && (variables[0]=="room_category1"))
	{
		// Identify which factor we want
		for(vector<string>::iterator it = _chainGraphInferencer->_objectPropertyVariables.begin();
				it!=_chainGraphInferencer->_objectPropertyVariables.end(); ++it)
		{
			if (variables[1] == (*it))
			{ // We found our factor!
				string objectName = (*it);
				replace_first(objectName, "object_", "");
				replace_last(objectName, "_property", "");

				// Set of room categories that we need to add value for
				set<string> roomCategories( _chainGraphInferencer->_roomCategories.begin(),
						_chainGraphInferencer->_roomCategories.end() );

				// Let's iterate over the knowledge and fill values of what we know
				for (std::list<HFCItem>::iterator it2=_chainGraphInferencer->_hfcKnowledge.begin();
						it2!=_chainGraphInferencer->_hfcKnowledge.end(); ++it2)
				{
					if (it2->object == objectName)
					{
						roomCategories.erase(it2->room); // Remove from the to-do list

						// Existance of object
						SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
								new SpatialProbabilities::StringRandomVariableValue(it2->room);
						SpatialProbabilities::BoolRandomVariableValuePtr objectRVVPtr =
								new SpatialProbabilities::BoolRandomVariableValue(true);
						SpatialProbabilities::JointProbabilityValue jpv1;
						jpv1.probability=it2->probability;
						jpv1.variableValues.push_back(roomCategory1RVVPtr);
						jpv1.variableValues.push_back(objectRVVPtr);
						factor.massFunction.push_back(jpv1);

						// Non-existance of object
						roomCategory1RVVPtr =
								new SpatialProbabilities::StringRandomVariableValue(it2->room);
						objectRVVPtr =
								new SpatialProbabilities::BoolRandomVariableValue(false);
						SpatialProbabilities::JointProbabilityValue jpv2;
						jpv2.probability = (1.0 - it2->probability);
						jpv2.variableValues.push_back(roomCategory1RVVPtr);
						jpv2.variableValues.push_back(objectRVVPtr);
						factor.massFunction.push_back(jpv2);
					}
				}

				// Let's see which room categories we missed
				for (set<string>::iterator it2=roomCategories.begin();
						it2!=roomCategories.end(); ++it2)
				{
					// Existence of object
					SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
							new SpatialProbabilities::StringRandomVariableValue(*it2);
					SpatialProbabilities::BoolRandomVariableValuePtr objectRVVPtr =
							new SpatialProbabilities::BoolRandomVariableValue(true);
					SpatialProbabilities::JointProbabilityValue jpv1;
					jpv1.probability=_chainGraphInferencer->_defaultObjectExistenceProbability;
					jpv1.variableValues.push_back(roomCategory1RVVPtr);
					jpv1.variableValues.push_back(objectRVVPtr);
					factor.massFunction.push_back(jpv1);

					// Non-existence of object
					roomCategory1RVVPtr =
							new SpatialProbabilities::StringRandomVariableValue(*it2);
					objectRVVPtr =
							new SpatialProbabilities::BoolRandomVariableValue(false);
					SpatialProbabilities::JointProbabilityValue jpv2;
					jpv2.probability = (1.0 - _chainGraphInferencer->_defaultObjectExistenceProbability);
					jpv2.variableValues.push_back(roomCategory1RVVPtr);
					jpv2.variableValues.push_back(objectRVVPtr);
					factor.massFunction.push_back(jpv2);
				}

				return factor;
			}
		}
	}


	// Factor not found!!
	throw CASTException("Factor '"+factorStr+"' not found!");
}


} // namespace def
