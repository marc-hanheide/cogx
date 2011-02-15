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
#include <fstream>


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
		_worldStateChanged(false) // We will process the world state initially
{
	pthread_cond_init(&_inferenceQueryAddedSignalCond, 0);
	pthread_mutex_init(&_inferenceQueryAddedSignalMutex, 0);
	pthread_mutex_init(&_worldStateMutex, 0);

    // Store the constants in a PropertySet object
	size_t maxiter = 10000;
    dai::Real   tol = 1e-9;
    size_t verb = 1;

     // Store the constants in a PropertySet object
     _daiOptions.set("maxiter",maxiter);  // Maximum number of iterations
     _daiOptions.set("tol",tol);          // Tolerance for convergence
     _daiOptions.set("verbose",verb);
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

	if((it = _config.find("--defaultchaingraphinferencer")) != _config.end())
		_defaultChainGraphInferencerName = it->second;
	if((it = _config.find("--save-graph")) != _config.end())
		_saveGraphFileName = it->second;
	if((it = _config.find("--save-graph-info")) != _config.end())
		_saveGraphInfoFileName = it->second;
	if((it = _config.find("--placeholder-in-current-room-prior")) != _config.end())
		_placeholderInCurrentRoomPrior = atof(it->second.c_str());
	else
		_placeholderInCurrentRoomPrior = 0.5;
	_inferPlaceholderProperties = (_config.find("--infer-placeholder-properties") != _config.end());

	log("Configuration parameters:");
	log("-> DefaultChainGraphInferencer name: %s", _defaultChainGraphInferencerName.c_str());
	log("-> Infer placeholder properties: %s", (_inferPlaceholderProperties)?"yes":"no");
	log("-> Prior probability that placeholder is in the current room: %f", _placeholderInCurrentRoomPrior);
}


// -------------------------------------------------------
void ChainGraphInferencer::start()
{
	// Local filter on ConceptualData::InferenceResult
	addChangeFilter(createLocalTypeFilter<ConceptualData::InferenceQuery>(cdl::ADD),
			new MemberFunctionChangeReceiver<ChainGraphInferencer>(this,
					&ChainGraphInferencer::inferenceQueryAdded));

	// Filter on WorldState
	addChangeFilter(createLocalTypeFilter<ConceptualData::WorldState>(cdl::ADD),
			new MemberFunctionChangeReceiver<ChainGraphInferencer>(this,
					&ChainGraphInferencer::worldStateChanged));
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

			debug("Processing inference query '%s' (imaginary = %s)",
							q.queryPtr->queryString.c_str(), (q.queryPtr->imaginary)?"true":"false");

			// Update the factor graph if necessary
			bool factorGraphChanged;
			bool worldStateValid = updateFactorGraph(factorGraphChanged);

			// Prepare some parts of the result
			ConceptualData::InferenceResultPtr inferenceResultPtr = new ConceptualData::InferenceResult();
			inferenceResultPtr->queryId = q.wmAddress.id;
			inferenceResultPtr->queryString = q.queryPtr->queryString;

			// Check if the world state is valid, otherwise return empty result
			if (worldStateValid)
			{
				// If the factor graph changed, re-run the inferences
				if (factorGraphChanged)
				{
					runAllInferences();
					_placeholderRoomCategoryExistance.clear(); // Clear also the placeholder stuff
				}

				// If this is a query about imaginary worlds, run imaginary world generation if the cache is empty
				if ((q.queryPtr->imaginary) && (_placeholderRoomCategoryExistance.empty()))
				{
					runImaginaryWorldsGeneration();
				}

				// Parse the query string
				vector<string> queryVariables;
				parseQuery(q.queryPtr->queryString, queryVariables);
				if (!queryVariables.empty())
				{
					// Is this a query about an imaginary world?
					if (q.queryPtr->imaginary)
					{
						// Prepare the result distribution
						prepareImaginaryInferenceResult(q.queryPtr->queryString,
								queryVariables, &(inferenceResultPtr->result));
					}
					else
					{
						// Prepare the result distribution
						prepareInferenceResult(q.queryPtr->queryString,
								queryVariables, &(inferenceResultPtr->result));
					}
				}

			}
			else
				log("World state is invalid. We will not run inference. Returning empty result.");

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
		return;
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
bool ChainGraphInferencer::updateFactorGraph(bool &factorGraphChanged)
{
	factorGraphChanged = false;

	// Lock world state
	pthread_mutex_lock(&_worldStateMutex);

	// If world state changed, re-create the factor graph
	if (_worldStateChanged)
	{
		log("Updating the graph since the world state has changed!");

		// Check if the world state is actaully valid
		if (_worldStateRooms.empty())
		{ // World state invalid
			log("World state is invalid.");
			pthread_mutex_unlock(&_worldStateMutex);
			return false;
		}

		factorGraphChanged = true;
		_worldStateChanged = false;

		// RECREATE THE FACTOR GRAPH FROM WORLD STATE AND DEFAULT KNOWLEDGE
		_factors.clear();
		_factorNames.clear();
		_variableNameToDai.clear();
		addDaiFactors();
		_factorGraph = dai::FactorGraph(_factors);

		// Save the generated factor graph
		if (!_saveGraphFileName.empty())
		{
			_factorGraph.WriteToFile(_saveGraphFileName.c_str());
		}
		if (!_saveGraphInfoFileName.empty())
		{
			ofstream file(_saveGraphInfoFileName.c_str());
			for (std::map<std::string, DaiVariable>::iterator i = _variableNameToDai.begin();
					i!=_variableNameToDai.end(); ++i)
			{
				file << i->second.var << " " << i->first.c_str() << endl;
				for (std::map<int, std::string>::iterator j = i->second.valueIdToName.begin();
									j!=i->second.valueIdToName.end(); ++j)
				{
					file << " " << j->first << " " << j->second << endl;
				}
				file <<endl;
			}
			for (unsigned int i = 0; i< _factors.size(); ++i)
			{
				file << "f" << i << " " << _factorNames[i] << endl;
			}
			file << endl;
			file.close();
		}
	}

	// Unlock world state
	pthread_mutex_unlock(&_worldStateMutex);

	return true;
}


// -------------------------------------------------------
void ChainGraphInferencer::createDaiVariable(string name, const vector<string> &values)
{
	if (_variableNameToDai.find(name) == _variableNameToDai.end())
	{
		DaiVariable &dv = _variableNameToDai[name];
		dv.var = dai::Var(_variableNameToDai.size(), values.size());
		for (unsigned int i=0; i<values.size(); ++i)
		{
			dv.valueIdToName[i] = values[i];
		}
	}
}

// -------------------------------------------------------
double ChainGraphInferencer::getProbabilityValue(
		const SpatialProbabilities::ProbabilityDistribution &pd,
		std::string var1Value, std::string var2value)
{
	for(unsigned int i=0; i<pd.massFunction.size(); ++i)
	{
		SpatialProbabilities::JointProbabilityValue jpv = pd.massFunction[i];
		string v1
			=SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(
					jpv.variableValues[0])->value;
		string v2
			=SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(
					jpv.variableValues[1])->value;

		if ( (v1 == var1Value) && (v2 == var2value) )
			return jpv.probability;
	}

	return -1;
}


// -------------------------------------------------------
double ChainGraphInferencer::getProbabilityValue(
		const SpatialProbabilities::ProbabilityDistribution &pd,
		std::string var1Value, bool var2value)
{
	for(unsigned int i=0; i<pd.massFunction.size(); ++i)
	{
		SpatialProbabilities::JointProbabilityValue jpv = pd.massFunction[i];
		string v1
			=SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(
					jpv.variableValues[0])->value;
		bool v2
			=SpatialProbabilities::BoolRandomVariableValuePtr::dynamicCast(
					jpv.variableValues[1])->value;

		if ( (v1 == var1Value) && (v2 == var2value) )
			return jpv.probability;
	}

	return -1;
}


// -------------------------------------------------------
void ChainGraphInferencer::createDaiConnectivityFactor(int room1Id, int room2Id)
{
	// Get the default connectivity factor
	string factorName = "f(room_category1,room_category2)";
	std::map<std::string, SpatialProbabilities::ProbabilityDistribution>::iterator dnfIt =
			_defaultKnowledgeFactors.find(factorName);
	if (dnfIt == _defaultKnowledgeFactors.end())
	{
		error("Factor \'%s\' not found. This indicates a serious implementatation error!", factorName.c_str());
		return;
	}
	const SpatialProbabilities::ProbabilityDistribution &factor = dnfIt->second;

	// Create variables
	string room1VarName = "room"+lexical_cast<string>(room1Id)+"_category";
	string room2VarName = "room"+lexical_cast<string>(room2Id)+"_category";

	debug("Creating DAI connectivity factor for variables '%s' and '%s'", room1VarName.c_str(), room2VarName.c_str() );
	createDaiVariable(room1VarName, _roomCategories);
	createDaiVariable(room2VarName, _roomCategories);
	DaiVariable &dv1 = _variableNameToDai[room1VarName];
	DaiVariable &dv2 = _variableNameToDai[room2VarName];

	// Create factor
	dai::Factor daiFactor( dai::VarSet( dv1.var, dv2.var ) );
	// Note: first fariable changes faster
	// Go over the second variable
	int roomCatCount = _roomCategories.size();
	int index=0;
	for (int i2 = 0; i2<roomCatCount; ++i2)
	{
		string var2ValueName = dv2.valueIdToName[i2];
		// Go over the first variable
		for (int i1 = 0; i1<roomCatCount; ++i1)
		{
			string var1ValueName = dv1.valueIdToName[i1];
			double potential = getProbabilityValue(factor, var1ValueName, var2ValueName);
			if (potential<0)
				throw CASTException("Potential not found for values '"+var1ValueName+"' and '"+var2ValueName+"'");
			daiFactor.set(index, potential);
			++index;
		}
	}

	// Add factor to the list
	_factors.push_back(daiFactor);
	_factorNames.push_back(factorName);
}



// -------------------------------------------------------
void ChainGraphInferencer::createDaiSingleRoomFactor(int room1Id)
{
	// Create variables
	string room1VarName = "room"+lexical_cast<string>(room1Id)+"_category";

	debug("Creating DAI single room factor for variable '%s'", room1VarName.c_str());
	createDaiVariable(room1VarName, _roomCategories);
	DaiVariable &dv1 = _variableNameToDai[room1VarName];

	// Create factor
	dai::Factor daiFactor( dv1.var );

	// Note: first fariable changes faster
	// Go over the second variable
	int roomCatCount = _roomCategories.size();
	int index=0;
	// Go over the first variable
	for (int i1 = 0; i1<roomCatCount; ++i1)
	{
		string var1ValueName = dv1.valueIdToName[i1];
		double potential = 0.1;
		daiFactor.set(index, potential);
		++index;
	}

	// Add factor to the list
	_factors.push_back(daiFactor);
	_factorNames.push_back("SingleRoomFactor");
}


// -------------------------------------------------------
void ChainGraphInferencer::createDaiObservedObjectPropertyFactor(int room1Id,
		string objectVariableName, bool objectExists)
{
	// Get the default object property factor
	string factorName = string("f(room_category1,")+objectVariableName+")";
	std::map<std::string, SpatialProbabilities::ProbabilityDistribution>::iterator dnfIt =
			_defaultKnowledgeFactors.find(factorName);
	if (dnfIt == _defaultKnowledgeFactors.end())
	{
		error ("Factor \'%s\' not found. This usually means that the object was not in the tuple store.",
				factorName.c_str());
		return;
	}
	const SpatialProbabilities::ProbabilityDistribution &factor = dnfIt->second;

	// Create variables
	string room1VarName = "room"+lexical_cast<string>(room1Id)+"_category";

	debug("Creating DAI observed object property factor for variable '%s' and object '%s'", room1VarName.c_str(),
			objectVariableName.c_str() );
	createDaiVariable(room1VarName, _roomCategories);
	DaiVariable &dv1 = _variableNameToDai[room1VarName];

	// Create factor
	dai::Factor daiFactor( dv1.var );
	// Note: first fariable changes faster
	// Go over the second variable
	int roomCatCount = _roomCategories.size();
	int index=0;
	for (int i1 = 0; i1<roomCatCount; ++i1)
	{
		string var1ValueName = dv1.valueIdToName[i1];
		double potential = getProbabilityValue(factor, var1ValueName, objectExists);
		if (potential<0)
			throw CASTException("Potential not found for object '"+objectVariableName+
					"'values '"+var1ValueName+"' and '"+((objectExists)?"true":"false")+"'");
		daiFactor.set(index, potential);
		++index;
	}

	// Add factor to the list
	_factors.push_back(daiFactor);
	_factorNames.push_back(factorName);

}


// -------------------------------------------------------
void ChainGraphInferencer::createDaiShapePropertyGivenRoomCategoryFactor(int room1Id, int placeId)
{
	// Get the default connectivity factor
	string factorName = "f(room_category1,shape_property)";
	std::map<std::string, SpatialProbabilities::ProbabilityDistribution>::iterator dnfIt =
			_defaultKnowledgeFactors.find(factorName);
	if (dnfIt == _defaultKnowledgeFactors.end())
	{
		error("Factor \'%s\' not found. This indicates a serious implementation error!", factorName.c_str());
		return;
	}
	const SpatialProbabilities::ProbabilityDistribution &factor = dnfIt->second;

	// Create variables
	string room1VarName = "room"+lexical_cast<string>(room1Id)+"_category";
	string shapePropVarName = "place"+lexical_cast<string>(placeId)+"_shape_property";

	debug("Creating DAI connectivity factor for variables '%s' and '%s'", room1VarName.c_str(), shapePropVarName.c_str() );
	createDaiVariable(room1VarName, _roomCategories);
	createDaiVariable(shapePropVarName, _shapes);
	DaiVariable &dv1 = _variableNameToDai[room1VarName];
	DaiVariable &dv2 = _variableNameToDai[shapePropVarName];

	// Create factor
	dai::Factor daiFactor( dai::VarSet( dv1.var, dv2.var ) );
	// Note: first fariable changes faster
	// Go over the second variable
	int roomCatCount = _roomCategories.size();
	int shapeCount = _shapes.size();
	int index=0;
	for (int i2 = 0; i2<shapeCount; ++i2)
	{
		string var2ValueName = dv2.valueIdToName[i2];
		// Go over the first variable
		for (int i1 = 0; i1<roomCatCount; ++i1)
		{
			string var1ValueName = dv1.valueIdToName[i1];
			double potential = getProbabilityValue(factor, var1ValueName, var2ValueName);
			if (potential<0)
				throw CASTException("Potential not found for values '"+var1ValueName+"' and '"+var2ValueName+"'");
			daiFactor.set(index, potential);
			++index;
		}
	}

	// Add factor to the list
	_factors.push_back(daiFactor);
	_factorNames.push_back(factorName);

}


// -------------------------------------------------------
void ChainGraphInferencer::createDaiObservedShapePropertyFactor(int placeId,
		ConceptualData::ValuePotentialPairs dist)
{
	string shapePropVarName = "place"+lexical_cast<string>(placeId)+"_shape_property";

	debug("Creating DAI observed shape property factor for variable '%s'",
			shapePropVarName.c_str());

	// Create variables
	createDaiVariable(shapePropVarName, _shapes);
	DaiVariable &dv = _variableNameToDai[shapePropVarName];

	// Create factor
	dai::Factor daiFactor( dv.var );

	int shapeCount = _shapes.size();
	int index=0;
	for (int i = 0; i<shapeCount; ++i)
	{
		// Find probability for the shape
		string shape = _shapes[i];
		double potential = -1;
		for (unsigned int j=0; j<dist.size(); ++j)
		{
			if (dist[j].value == shape)
			{
				potential = dist[j].potential;
				break;
			}
		}
		if (potential<0)
		{
			log("Warning: Can't find potential for a shape %s while building observed shape property DAI factor!"
					"This probably means that the shape value is not present in the model for this property.",
					shape.c_str());
			potential=0.01;
		}
		daiFactor.set(index, potential);
		++index;
	}

	// Add factor to the list
	_factors.push_back(daiFactor);
	_factorNames.push_back("ObservedShapePropertyFactor");

}


// -------------------------------------------------------
void ChainGraphInferencer::createDaiAppearancePropertyGivenRoomCategoryFactor(int room1Id, int placeId)
{
	// Get the default connectivity factor
	string factorName = "f(room_category1,appearance_property)";
	std::map<std::string, SpatialProbabilities::ProbabilityDistribution>::iterator dnfIt =
			_defaultKnowledgeFactors.find(factorName);
	if (dnfIt == _defaultKnowledgeFactors.end())
	{
		error("Factor \'%s\' not found. This indicates a serious implementation error!", factorName.c_str());
		return;
	}
	const SpatialProbabilities::ProbabilityDistribution &factor = dnfIt->second;

	// Create variables
	string room1VarName = "room"+lexical_cast<string>(room1Id)+"_category";
	string appearancePropVarName = "place"+lexical_cast<string>(placeId)+"_appearance_property";

	debug("Creating DAI connectivity factor for variables '%s' and '%s'", room1VarName.c_str(), appearancePropVarName.c_str() );
	createDaiVariable(room1VarName, _roomCategories);
	createDaiVariable(appearancePropVarName, _appearances);
	DaiVariable &dv1 = _variableNameToDai[room1VarName];
	DaiVariable &dv2 = _variableNameToDai[appearancePropVarName];

	// Create factor
	dai::Factor daiFactor( dai::VarSet( dv1.var, dv2.var ) );
	// Note: first fariable changes faster
	// Go over the second variable
	int roomCatCount = _roomCategories.size();
	int appearanceCount = _appearances.size();
	int index=0;
	for (int i2 = 0; i2<appearanceCount; ++i2)
	{
		string var2ValueName = dv2.valueIdToName[i2];
		// Go over the first variable
		for (int i1 = 0; i1<roomCatCount; ++i1)
		{
			string var1ValueName = dv1.valueIdToName[i1];
			double potential = getProbabilityValue(factor, var1ValueName, var2ValueName);
			if (potential<0)
				throw CASTException("Potential not found for values '"+var1ValueName+"' and '"+var2ValueName+"'");
			daiFactor.set(index, potential);
			++index;
		}
	}

	// Add factor to the list
	_factors.push_back(daiFactor);
	_factorNames.push_back(factorName);

}


// -------------------------------------------------------
void ChainGraphInferencer::createDaiObservedAppearancePropertyFactor(int placeId,
		ConceptualData::ValuePotentialPairs dist)
{
	string appearancePropVarName = "place"+lexical_cast<string>(placeId)+"_appearance_property";

	debug("Creating DAI observed appearance property factor for variable '%s'",
			appearancePropVarName.c_str());

	// Create variables
	createDaiVariable(appearancePropVarName, _appearances);
	DaiVariable &dv = _variableNameToDai[appearancePropVarName];

	// Create factor
	dai::Factor daiFactor( dv.var );

	int appearanceCount = _appearances.size();
	int index=0;
	for (int i = 0; i<appearanceCount; ++i)
	{
		// Find probability for the appearance
		string appearance = _appearances[i];
		double potential = -1;
		for (unsigned int j=0; j<dist.size(); ++j)
		{
			if (dist[j].value == appearance)
			{
				potential = dist[j].potential;
				break;
			}
		}
		if (potential<0)
		{
			log("Warning: Can't find potential for an appearance %s while building observed appearance property DAI factor!"
					"This probably means that the appearance value is not present in the model for this property.",
					appearance.c_str());

			potential=0.01;
		}
		daiFactor.set(index, potential);
		++index;
	}

	// Add factor to the list
	_factors.push_back(daiFactor);
	_factorNames.push_back("ObservedAppearancePropertyFactor");

}


// -------------------------------------------------------
void ChainGraphInferencer::addDaiFactors()
{
	// Let's go through all the room connections and add category connectivity factors
	for (unsigned int i=0; i<_worldStateRoomConnections.size(); ++i)
	{
		ConceptualData::RoomConnectivityInfo &rci = _worldStateRoomConnections[i];
		createDaiConnectivityFactor(rci.room1Id, rci.room2Id);
	}

	// Add single room factors. Should change nothing, but help if there are unconnected or
	// single rooms without any properties.
	for (unsigned int i=0; i<_worldStateRooms.size(); ++i)
		createDaiSingleRoomFactor(_worldStateRooms[i].roomId);

	// Property factors
	// Go over all rooms
	for (unsigned int r=0; r<_worldStateRooms.size(); ++r)
	{
		// No through all the places in the room
		const ConceptualData::ComaRoomInfo &cri =_worldStateRooms[r];
		for (unsigned int p=0; p<cri.places.size(); ++p)
		{
			const ConceptualData::PlaceInfo &pi = cri.places[p];

			// Object properties
			for (unsigned int o=0; o<pi.objectProperties.size(); ++o)
			{
				const ConceptualData::ObjectPlacePropertyInfo &oppi = pi.objectProperties[o];
				createDaiObservedObjectPropertyFactor(cri.roomId,
						"object_"+oppi.category+"_property", oppi.present);
			} // o

			// Shape properties
			for (unsigned int s=0; s<pi.shapeProperties.size(); ++s)
			{
				const ConceptualData::ShapePlacePropertyInfo &sppi = pi.shapeProperties[s];
				createDaiShapePropertyGivenRoomCategoryFactor(cri.roomId, pi.placeId);
				createDaiObservedShapePropertyFactor(pi.placeId, sppi.distribution);
			} // s

			// Appearance properties
			for (unsigned int a=0; a<pi.appearanceProperties.size(); ++a)
			{
				const ConceptualData::AppearancePlacePropertyInfo &appi = pi.appearanceProperties[a];
				createDaiAppearancePropertyGivenRoomCategoryFactor(cri.roomId, pi.placeId);
				createDaiObservedAppearancePropertyFactor(pi.placeId, appi.distribution);
			} // a


		} // p
	} // r
}


// -------------------------------------------------------
void ChainGraphInferencer::runAllInferences()
{
	log("Running all inferences on the graph!");

	_bp = dai::BP(_factorGraph, _daiOptions("updates",string("SEQRND"))("logdomain",false));
	_bp.init();
	_bp.run();

//	_junctionTree = dai::JTree(_factorGraph, _daiOptions("updates",string("HUGIN")));
//	_junctionTree.init();
//	_junctionTree.run();
}


// -------------------------------------------------------
void ChainGraphInferencer::runImaginaryWorldsGeneration()
{
	// Go over all rooms
	for (unsigned int r=0; r<_worldStateRooms.size(); ++r)
	{
		const ConceptualData::ComaRoomInfo &cri =_worldStateRooms[r];
		// Run imaginary world generation for that room
		vector<double> outputs;
		runImaginaryWorldsGenerationForPlaceholderInRoom(cri.roomId, outputs);

		// No through all the placeholders in the room and apply the results
		for (unsigned int p=0; p<cri.placeholders.size(); ++p)
		{
			const ConceptualData::PlaceholderInfo &phi = cri.placeholders[p];
			for (unsigned int i=0; i<_roomCategories.size(); ++i)
			{
				stringstream varName;
				varName << "placeholder" << phi.placeholderId << "_" << _roomCategories[i] << "_existance";
				_placeholderRoomCategoryExistance[varName.str()] = outputs[i];
			}
		}
	}
}


// -------------------------------------------------------
void ChainGraphInferencer::runImaginaryWorldsGenerationForPlaceholderInRoom(int roomId,
		std::vector<double> &outputs)
{
	outputs.clear();
	outputs.resize(_roomCategories.size());
	for (unsigned int i=0; i<outputs.size(); ++i)
		outputs[i] = 0.0;

	// World in which there are no other rooms
	// This one is not included as the properties contain only the probability of
	// categories of NEW rooms.
	// evaluateCurrentRoomImaginaryWorld(roomId, outputs, _placeholderInCurrentRoomPrior);

	vector<dai::Factor> factors = _factors;
	dai::VarSet newVars;
	int varId = _variableNameToDai.size();

	// World in which there is one additional room
	// -------------------------------------------
	// Create variables & factors
	string roomVarName = "room"+lexical_cast<string>(roomId)+"_category";
	dai::Var v1 = dai::Var(++varId, _roomCategories.size());
	newVars.insert(v1);
	// Create factor
	createDaiConnectivityFactor(factors, _variableNameToDai[roomVarName].var, v1);
	// Create factor graph
	dai::FactorGraph factorGraph = dai::FactorGraph(factors);

	// Run inference
	log("Running all inferences on the imaginary graph!");
	dai::BP bp = dai::BP(factorGraph, _daiOptions("updates",string("SEQRND"))("logdomain",false));
	bp.init();
	bp.run();
	// Update outputs using marginals
	updateOutputsUsingImaginaryVariables(bp, newVars, outputs, (1.0-_placeholderInCurrentRoomPrior)/2.0);

	// World in which there are 2 additional rooms
	// -------------------------------------------
	// Create variables & factors
	dai::Var v2 = dai::Var(++varId, _roomCategories.size());
	newVars.insert(v2);
	// Create factor
	createDaiConnectivityFactor(factors, v1, v2);
	// Create factor graph
	factorGraph = dai::FactorGraph(factors);

	// Run inference
	log("Running all inferences on the imaginary graph!");
	bp = dai::BP(factorGraph, _daiOptions("updates",string("SEQRND"))("logdomain",false));
	bp.init();
	bp.run();
	// Update outputs using marginals
	updateOutputsUsingImaginaryVariables(bp, newVars, outputs, (1.0-_placeholderInCurrentRoomPrior)/2.0);

}


// -------------------------------------------------------
void ChainGraphInferencer::updateOutputsUsingImaginaryVariables(dai::BP &bp, dai::VarSet &vars, std::vector<double> &outputs, double prior)
{
	// Retrieve marginal
	//        dai::Factor marginal = _junctionTree.belief(vars);
	dai::Factor marginal = bp.belief(vars);

	// Update outputs for 1 variable
	if (vars.size() == 1)
	{
		for (unsigned int i = 0; i < marginal.nrStates(); ++i)
		{
			double marginalProb = marginal.get(i);
			outputs[i]+=prior * marginalProb;
		}
	}
	else if (vars.size() == 2)
	{
		int i = 0;
		for (unsigned int v2 = 0; v2 < vars.elements()[1].states(); ++v2)
		{
			for (unsigned int v1 = 0; v1 < vars.elements()[0].states(); ++v1)
			{
				double marginalProb = marginal.get(i);
				if (v1==v2)
					outputs[v1]+=prior * marginalProb;
				else
				{
					outputs[v1]+=prior * marginalProb;
					outputs[v2]+=prior * marginalProb;
				}
				i++;
			}
		}
	}
	else
		throw CASTException("Incorrect number of variables in updateOutputsUsingImaginaryVariables");
}

// -------------------------------------------------------
void ChainGraphInferencer::createDaiConnectivityFactor(vector<dai::Factor> &factors, dai::Var &var1, dai::Var &var2)
{
	// Get the default connectivity factor
	string factorName = "f(room_category1,room_category2)";
	std::map<std::string, SpatialProbabilities::ProbabilityDistribution>::iterator dnfIt =
			_defaultKnowledgeFactors.find(factorName);
	if (dnfIt == _defaultKnowledgeFactors.end())
	{
		error("Factor \'%s\' not found. This indicates a serious implementatation error!", factorName.c_str());
		return;
	}
	const SpatialProbabilities::ProbabilityDistribution &factor = dnfIt->second;

	// Create factor
	dai::Factor daiFactor( dai::VarSet( var1, var2 ) );
	// Note: first fariable changes faster
	// Go over the second variable
	int roomCatCount = _roomCategories.size();
	int index=0;
	for (int i2 = 0; i2<roomCatCount; ++i2)
	{
		string var2ValueName = _roomCategories[i2];
		// Go over the first variable
		for (int i1 = 0; i1<roomCatCount; ++i1)
		{
			string var1ValueName = _roomCategories[i1];
			double potential = getProbabilityValue(factor, var1ValueName, var2ValueName);
			if (potential<0)
				throw CASTException("Potential not found for values '"+var1ValueName+"' and '"+var2ValueName+"'");
			daiFactor.set(index, potential);
			++index;
		}
	}

	// Add factor to the list
	factors.push_back(daiFactor);
}



// -------------------------------------------------------
void ChainGraphInferencer::evaluateCurrentRoomImaginaryWorld(int roomId, std::vector<double> &outputs, double prior)
{
	// Find the room category variable
	stringstream varName;
	varName <<"room" << roomId << "_category";
	map<string, DaiVariable>::iterator varIter = _variableNameToDai.find(varName.str());
	if (varIter == _variableNameToDai.end())
	{
		string msg;
		msg = "Variable '" + varName.str() + "' not found while evaluating imaginary world! Variables we know:";
		for (varIter = _variableNameToDai.begin(); varIter
				!= _variableNameToDai.end(); ++varIter)
			msg += varIter->first + " ";
		log(msg.c_str());
		return;
	}

	// Retrieve marginal
	// dai::Factor marginal = _junctionTree.belief(varIter->second.var);
	dai::Factor marginal = _bp.belief(varIter->second.var);

	// Convert to output
	// We assume that the order is the same as in _roomCategories here!!
	for (unsigned int i = 0; i < marginal.nrStates(); ++i)
		outputs[i]+= prior * marginal.get(i);
}



// -------------------------------------------------------
void ChainGraphInferencer::prepareInferenceResult(std::string queryString,
		std::vector<std::string> queryVariables,
		SpatialProbabilities::ProbabilityDistribution *resultDistribution)
{
	debug("Preparing inference results for inference query '"+queryString+"'");

	// Prepare the resulting distribution
	resultDistribution->description = queryString;
	for(unsigned int i=0; i<queryVariables.size(); ++i)
		resultDistribution->variableNameToPositionMap[queryVariables[i]]=i;
	resultDistribution->massFunction.clear();

	// Check
	if (queryVariables.size()!=1)
	{
		error("Unhandled query \'%s\'. This indicates serious implementation error.", queryString.c_str());
		return;
	}
	string varName = queryVariables[0];

	// Find the variable
	map<string, DaiVariable>::iterator varIter = _variableNameToDai.find(
			varName);
	if (varIter == _variableNameToDai.end())
	{
		string msg;
		msg = "Variable '" + varName + "' not found! Variables we know:";
		for (varIter = _variableNameToDai.begin(); varIter
				!= _variableNameToDai.end(); ++varIter)
			msg += varIter->first + " ";
		log(msg.c_str());
		return;
	}
	// Retrieve marginal
	//        dai::Factor marginal = _junctionTree.belief(varIter->second.var);
	dai::Factor marginal = _bp.belief(varIter->second.var);
	// Convert to probability distribution
	for (unsigned int i = 0; i < marginal.nrStates(); ++i)
	{
		double marginalProb = marginal.get(i);
		SpatialProbabilities::StringRandomVariableValuePtr rvvPtr =
				new SpatialProbabilities::StringRandomVariableValue(
						varIter->second.valueIdToName[i]);
		SpatialProbabilities::JointProbabilityValue jpv;
		jpv.probability = marginalProb;
		jpv.variableValues.push_back(rvvPtr);
		resultDistribution->massFunction.push_back(jpv);
	}
}


// -------------------------------------------------------
void ChainGraphInferencer::prepareImaginaryInferenceResult(std::string queryString,
		std::vector<std::string> queryVariables,
		SpatialProbabilities::ProbabilityDistribution *resultDistribution)
{
	debug("Preparing imaginary inference results for inference query '"+queryString+"'");

	// Prepare the resulting distribution
	resultDistribution->description = queryString;
	for(unsigned int i=0; i<queryVariables.size(); ++i)
		resultDistribution->variableNameToPositionMap[queryVariables[i]]=i;
	resultDistribution->massFunction.clear();

	// Check
	if (queryVariables.size()!=1)
	{
		error("Unhandled query \'%s\'. This indicates serious implementation error.", queryString.c_str());
		return;
	}
	string varName = queryVariables[0];
	if (_placeholderRoomCategoryExistance.find(varName)==_placeholderRoomCategoryExistance.end())
	{
		error("Varianble name \'%s\' not found in the inferred variable set.", varName.c_str());
		return;
	}

	// Find the variable and set the values of probability
	double prob = _placeholderRoomCategoryExistance[varName];
	SpatialProbabilities::IntRandomVariableValuePtr rvv1Ptr =
			new SpatialProbabilities::IntRandomVariableValue(1);
	SpatialProbabilities::JointProbabilityValue jpv1;
	jpv1.probability=prob;
	jpv1.variableValues.push_back(rvv1Ptr);
	resultDistribution->massFunction.push_back(jpv1);

	SpatialProbabilities::IntRandomVariableValuePtr rvv2Ptr =
			new SpatialProbabilities::IntRandomVariableValue(0);
	SpatialProbabilities::JointProbabilityValue jpv2;
	jpv2.probability=1.0 - prob;
	jpv2.variableValues.push_back(rvv2Ptr);
	resultDistribution->massFunction.push_back(jpv2);
}


// -------------------------------------------------------
void ChainGraphInferencer::parseQuery(string queryString, vector<string> &variables)
{
	// Check the query string
	if ( (queryString.length()<4) || (queryString[0]!='p') ||
		 (queryString[1]!='(') || (queryString[queryString.length()-1]!=')') )
	{
		error("Malformed ChainGraphInferencer query string \'%s\'! This indicates a serious implementation error!",
				queryString.c_str());
		return;
	}

	// Extract variable names
	string variableString=queryString;
	erase_head(variableString, 2);
	erase_tail(variableString, 1);
	split( variables, variableString, is_any_of(", ") );
}


// -------------------------------------------------------
void ChainGraphInferencer::getDefaultKnowledge()
{
	log("Obtaining default knowledge factors from DefaultChainGraphInferencer.");

	_objectPropertyVariables =
		_defaultChainGraphInferencerServerInterfacePrx->getObjectPropertyVariables();
	_roomCategories =
		_defaultChainGraphInferencerServerInterfacePrx->getRoomCategories();
	_shapes =
		_defaultChainGraphInferencerServerInterfacePrx->getShapes();
	_appearances =
		_defaultChainGraphInferencerServerInterfacePrx->getAppearances();


	// Error checking
	if ( (_objectPropertyVariables.empty()) || (_roomCategories.empty()) )
		throw CASTException("Did not receive information from Default.SA. Is everything started?");

	string factorStr;

	// Get the room_category1, room_category2 factor
	factorStr = "f(room_category1,room_category2)";
	_defaultKnowledgeFactors[factorStr] =
			_defaultChainGraphInferencerServerInterfacePrx->getFactor(factorStr);
	if (_defaultKnowledgeFactors[factorStr].massFunction.empty())
		throw CASTException("Did not receive information from Default.SA. Is everything started?");


	// Get the room1_category -> object_xxx_property factors
	for(unsigned int i=0; i<_objectPropertyVariables.size(); ++i)
	{
		factorStr = "f(room_category1,"+_objectPropertyVariables[i]+")";
		_defaultKnowledgeFactors[factorStr] =
				_defaultChainGraphInferencerServerInterfacePrx->getFactor(factorStr);
		if (_defaultKnowledgeFactors[factorStr].massFunction.empty())
			throw CASTException("Did not receive information from Default.SA. Is everything started?");
	}

	// Get the room1_category -> shape_property factor
	factorStr = "f(room_category1,shape_property)";
	_defaultKnowledgeFactors[factorStr] =
			_defaultChainGraphInferencerServerInterfacePrx->getFactor(factorStr);
	if (_defaultKnowledgeFactors[factorStr].massFunction.empty())
		throw CASTException("Did not receive information from Default.SA. Is everything started?");

	// Get the room1_category -> appearance_property factor
	factorStr = "f(room_category1,appearance_property)";
	_defaultKnowledgeFactors[factorStr] =
			_defaultChainGraphInferencerServerInterfacePrx->getFactor(factorStr);
	if (_defaultKnowledgeFactors[factorStr].massFunction.empty())
		throw CASTException("Did not receive information from Default.SA. Is everything started?");
}





} // namespace def
