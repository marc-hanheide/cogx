/**
 * @author Andrzej Pronobis
 *
 * Definition of the conceptual::ChainGraphInferencer class.
 */

// Conceptual.SA
#include "ChainGraphInferencer.h"
#include "VariableNameGenerator.h"
#include "SpatialProbabilities.hpp"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Boost
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/assign/list_of.hpp>
// Std
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
using namespace boost::algorithm;


// -------------------------------------------------------
ChainGraphInferencer::ChainGraphInferencer() :
		_worldStateChanged(false) // We will process the world state initially
{
	pthread_cond_init(&_inferenceQueryAddedSignalCond, 0);
	pthread_mutex_init(&_inferenceQueryAddedSignalMutex, 0);
	pthread_mutex_init(&_worldStateMutex, 0);
	pthread_mutex_init(&_graphMutex, 0);

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
	pthread_mutex_destroy(&_graphMutex);
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
	if((it = _config.find("--avs-default-knowledge")) != _config.end())
		_avsDefaultKnowledgeFileName = it->second;
	if((it = _config.find("--placeholder-in-current-room-prior")) != _config.end())
		_placeholderInCurrentRoomPrior = atof(it->second.c_str());
	else
		_placeholderInCurrentRoomPrior = 0.5;
	_inferPlaceholderProperties = (_config.find("--infer-placeholder-properties") != _config.end());

	// Check parameters
	if (_avsDefaultKnowledgeFileName.empty())
	{
		throw CASTException(exceptionMessage(__HERE__,"Provide the AVS default knowledge file!"));
	}

	log("Configuration parameters:");
	log("-> DefaultChainGraphInferencer name: %s", _defaultChainGraphInferencerName.c_str());
	log("-> Infer placeholder properties: %s", (_inferPlaceholderProperties)?"yes":"no");
	log("-> Prior probability that placeholder is in the current room: %f", _placeholderInCurrentRoomPrior);

	// Register the ICE Server
	ConceptualData::ChainGraphTestingServerInterfacePtr chainGraphTestingServerInterfacePtr =
			new TestingServer(this);
	registerIceServer<ConceptualData::ChainGraphTestingServerInterface,
			ConceptualData::ChainGraphTestingServerInterface>(chainGraphTestingServerInterfacePtr);

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

			debug("Processing inference query '%s' (type = %s)",
							q.queryPtr->queryString.c_str(),
							(q.queryPtr->type==ConceptualData::STANDARDQUERY)?"standard":
							((q.queryPtr->type==ConceptualData::IMAGINARYQUERY)?"imaginary":"factor"));

			// Prepare some parts of the result
			ConceptualData::InferenceResultPtr inferenceResultPtr = new ConceptualData::InferenceResult();
			inferenceResultPtr->queryId = q.wmAddress.id;
			inferenceResultPtr->queryString = q.queryPtr->queryString;

			// Parse the query string
			vector<string> queryVariables;
			parseQuery(q.queryPtr->queryString, q.queryPtr->type, queryVariables);
			if (!queryVariables.empty())
			{
				// Lock the graph mutex
				pthread_mutex_lock(&_graphMutex);

				if (q.queryPtr->type == ConceptualData::STANDARDQUERY)
				{
					// Get the "additional" variables that should be added from the query if missing
					_additionalVariables.clear();
					for(unsigned int i=0; i<queryVariables.size(); ++i)
					{
						if (queryVariables[i][0]=='+')
						{
							_additionalVariables.insert(trim_left_copy_if(queryVariables[i],is_any_of("+")));
							erase_first(queryVariables[i], "+");
							// log("Adding additional variable %s.", queryVariables[i].c_str());
						}
					}
				}

				// Update the factor graph if necessary
				bool factorGraphChanged;
				bool worldStateValid = updateFactorGraph(factorGraphChanged);

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
					if ((q.queryPtr->type==ConceptualData::IMAGINARYQUERY) && (_placeholderRoomCategoryExistance.empty()))
					{
						runImaginaryWorldsGeneration();
					}

					// Is this a query about an imaginary world?
					if (q.queryPtr->type == ConceptualData::IMAGINARYQUERY)
					{
						// Prepare the result distribution
						prepareImaginaryInferenceResult(q.queryPtr->queryString,
								queryVariables, inferenceResultPtr->results);
					}
					else if (q.queryPtr->type == ConceptualData::FACTORQUERY)
					{
						// Prepare the result distribution
						prepareFactorResult(q.queryPtr->queryString,
								queryVariables, inferenceResultPtr->results);
					}
					else
					{
						// Prepare the result distribution
						prepareInferenceResult(q.queryPtr->queryString,
								queryVariables, inferenceResultPtr->results);
					}
				} // if (worldStateValid)
				else
					log("World state is invalid. We will not run inference. Returning empty result.");

				// No more operations on the graph
				pthread_mutex_unlock(&_graphMutex);

			} // if (!queryVariables.empty())

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

	if (_worldStateChanged)
	{
		factorGraphChanged=true;
		log("Updating the graph since the world state has changed!");
	}
	else
	{
		// Check if we have new "additional variables"
		for(set<string>::iterator it = _additionalVariables.begin(); it!=_additionalVariables.end(); ++it)
		{
			if (_variableNameToDai.find(*it) == _variableNameToDai.end())
			{
				factorGraphChanged = true;
				log("Updating the graph since the new variables are requested.");
				break;
			}
		}
	}

	// If world state changed, re-create the factor graph
	if (factorGraphChanged)
	{
		// Check if the world state is actaully valid
		if (_worldStateRooms.empty())
		{ // World state invalid
			log("World state is invalid.");
			pthread_mutex_unlock(&_worldStateMutex);
			return false;
		}

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
	// No need to check if the variables are in right order, the factor is symmetric

	// Note: first fariable changes faster
	// Go over the second variable
	int index=0;
	for (unsigned int i2 = 0; i2<dv2.var.states(); ++i2)
	{
		string var2ValueName = dv2.valueIdToName[i2];
		// Go over the first variable
		for (unsigned int i1 = 0; i1<dv1.var.states(); ++i1)
		{
			string var1ValueName = dv1.valueIdToName[i1];
			double potential = getProbabilityValue(factor, var1ValueName, var2ValueName);
			if (potential<0)
				throw CASTException(exceptionMessage(__HERE__,
						"Potential not found for values '%s' and '%s'", var1ValueName.c_str(), var2ValueName.c_str()));
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
	int index=0;
	// Go over the first variable
	for (unsigned int i1 = 0; i1<dv1.var.states(); ++i1)
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
void ChainGraphInferencer::createDaiObservedObjectPropertyFactor(int room1Id, const std::string &objectCategory,
		SpatialData::SpatialRelation relation, const std::string &supportObjectCategory, const std::string &supportObjectId,
		unsigned int objectCount, double beta)
{
	// Create variables
	string room1VarName = "room"+lexical_cast<string>(room1Id)+"_category";
	string objectVarName = VariableNameGenerator::getDefaultObjectPropertyVarName(
			objectCategory, relation, supportObjectCategory);

	debug("Creating DAI observed object property factor for variable '%s' and object '%s'", room1VarName.c_str(),
			objectVarName.c_str() );
	createDaiVariable(room1VarName, _roomCategories);
	DaiVariable &dv1 = _variableNameToDai[room1VarName];

	// Create factor
	dai::Factor daiFactor( dv1.var );

	// Go over room categories
	int index=0;
	for (unsigned int i1 = 0; i1<dv1.var.states(); ++i1)
	{
		string var1ValueName = dv1.valueIdToName[i1]; // Room category
		// Get the lambda value for the poisson distribution
		double lambda = getPoissonLambda(var1ValueName, objectCategory, relation, supportObjectCategory);
		// Check for errors
		if (lambda<0)
			return;
		// Get the poisson prob distribution
		double probability = getPoissonProabability(beta*lambda, objectCount);
		daiFactor.set(index, probability);
		++index;
	}

	// Add factor to the list
	_factors.push_back(daiFactor);
	_factorNames.push_back("ObservedObjectPropertyFactor");
}


// -------------------------------------------------------
void ChainGraphInferencer::createDaiObjectUnexploredFactor(int room1Id, const std::string &objectCategory,
		SpatialData::SpatialRelation relation, const std::string &supportObjectCategory,
		const std::string &supportObjectId, double beta)
{
	// Check if we have default knowledge for this factor, if not do nothing
	string objectVariableName = VariableNameGenerator::getDefaultObjectPropertyVarName(
			objectCategory, relation, supportObjectCategory);
	string defFactorName = string("f(room_category1,")+objectVariableName+")";
	std::map<std::string, SpatialProbabilities::ProbabilityDistribution>::iterator dnfIt =
			_defaultKnowledgeFactors.find(defFactorName);
	if (dnfIt == _defaultKnowledgeFactors.end())
	{
		error("Factor \'%s\' not found. This usually means that the object was not in the tuple store.",
				defFactorName.c_str());
		return;
	}

	// Create variables
	string room1VarName = "room"+lexical_cast<string>(room1Id)+"_category";
	string objectUnexploredVarName = VariableNameGenerator::getUnexploredObjectVarName(
			room1Id, objectCategory, relation, supportObjectCategory, supportObjectId);
	string factorName = "f("+room1VarName+","+objectUnexploredVarName+")";

	debug("Creating DAI factor for variables '%s' and '%s'", room1VarName.c_str(), objectUnexploredVarName.c_str() );
	createDaiVariable(room1VarName, _roomCategories);
	vector<string> values(2);
	values[0] = ConceptualData::NOTEXISTS;
	values[1] = ConceptualData::EXISTS;
	createDaiVariable(objectUnexploredVarName, values);
	DaiVariable &dv1 = _variableNameToDai[room1VarName];
	DaiVariable &dv2 = _variableNameToDai[objectUnexploredVarName];

	// Create factor
	dai::Factor daiFactor( dai::VarSet( dv1.var, dv2.var ) );
	// Check if the variables in the factor are order the way they should
	bool switchVars=false;
	if (daiFactor.vars().elements()[0].label() != dv1.var.label())
	{
		switchVars=true;
		DaiVariable &tmp = dv1;
		dv1=dv2;
		dv2=tmp;
	}

	// Note: first variable changes faster
	// Go over the second variable
	int index=0;
	for (unsigned int i2 = 0; i2<dv2.var.states(); ++i2)
	{
		// Go over the first variable
		for (unsigned int i1 = 0; i1<dv1.var.states(); ++i1)
		{
			string roomCategory;
			string exist;
			if (switchVars)
			{
				roomCategory = dv2.valueIdToName[i2];
				exist = dv1.valueIdToName[i1];
			}
			else
			{
				roomCategory = dv1.valueIdToName[i1];
				exist = dv2.valueIdToName[i2];
			}

			// Get the lambda value for the poisson distribution
			double lambda = getPoissonLambda(roomCategory, objectCategory, relation, supportObjectCategory);
			// Check for errors
			if (lambda<0)
				return;
			// Get the poisson prob distribution
			double probability;
			if (exist == ConceptualData::EXISTS)
				probability = 1.0 - getPoissonProabability((1.0-beta)*lambda, 0);
			else
				probability = getPoissonProabability((1.0-beta)*lambda, 0);

			daiFactor.set(index, probability);
			++index;
		}
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
	// Check if the variables in the factor are order the way they should
	bool switchVars=false;
	if (daiFactor.vars().elements()[0].label() != dv1.var.label())
	{
		switchVars=true;
		DaiVariable &tmp = dv1;
		dv1=dv2;
		dv2=tmp;
	}

	// Note: first variable changes faster
	// Go over the second variable
	int index=0;
	for (unsigned int i2 = 0; i2<dv2.var.states(); ++i2)
	{
		// Go over the first variable
		for (unsigned int i1 = 0; i1<dv1.var.states(); ++i1)
		{
			string roomCategory;
			string shape;
			if (switchVars)
			{
				roomCategory = dv2.valueIdToName[i2];
				shape = dv1.valueIdToName[i1];
			}
			else
			{
				roomCategory = dv1.valueIdToName[i1];
				shape = dv2.valueIdToName[i2];
			}

			double potential = getProbabilityValue(factor, roomCategory, shape);
			if (potential<0)
				throw CASTException(exceptionMessage(__HERE__,
						"Potential not found for values '%s' and '%s'", roomCategory.c_str(), shape.c_str()));
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

	int index=0;
	for (unsigned int i = 0; i<dv.var.states(); ++i)
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
	// Check if the variables in the factor are order the way they should
	bool switchVars=false;
	if (daiFactor.vars().elements()[0].label() != dv1.var.label())
	{
		switchVars=true;
		DaiVariable &tmp = dv1;
		dv1=dv2;
		dv2=tmp;
	}

	// Note: first variable changes faster
	// Go over the second variable
	int index=0;
	for (unsigned int i2 = 0; i2<dv2.var.states(); ++i2)
	{
		// Go over the first variable
		for (unsigned int i1 = 0; i1<dv1.var.states(); ++i1)
		{
			string roomCategory;
			string appearance;
			if (switchVars)
			{
				roomCategory = dv2.valueIdToName[i2];
				appearance = dv1.valueIdToName[i1];
			}
			else
			{
				roomCategory = dv1.valueIdToName[i1];
				appearance = dv2.valueIdToName[i2];
			}

			double potential = getProbabilityValue(factor, roomCategory, appearance);
			if (potential<0)
				throw CASTException(exceptionMessage(__HERE__,
						"Potential not found for values '%s' and '%s'", roomCategory.c_str(), appearance.c_str()));
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

	int index=0;
	for (unsigned int i = 0; i<dv.var.states(); ++i)
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
				createDaiObservedObjectPropertyFactor(cri.roomId, oppi.category, oppi.relation,
													  oppi.supportObjectCategory, oppi.supportObjectId,
													  oppi.count, oppi.beta);
				createDaiObjectUnexploredFactor(cri.roomId, oppi.category, oppi.relation,
						  	  	  	  	  	    oppi.supportObjectCategory, oppi.supportObjectId,
						  	  	  	  	  	    oppi.beta);
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

		// Place independent Object properties
		for (unsigned int o=0; o<cri.objectProperties.size(); ++o)
		{
			const ConceptualData::ObjectPlacePropertyInfo &oppi = cri.objectProperties[o];
			createDaiObservedObjectPropertyFactor(cri.roomId, oppi.category, oppi.relation,
												  oppi.supportObjectCategory, oppi.supportObjectId,
												  oppi.count, oppi.beta);
			createDaiObjectUnexploredFactor(cri.roomId, oppi.category, oppi.relation,
					  	  	  	  	  	    oppi.supportObjectCategory, oppi.supportObjectId,
					  	  	  	  	  	    oppi.beta);
		} // o
	} // r

	// Add factors for additional variables requested in the queries
	for(set<string>::iterator it = _additionalVariables.begin(); it!=_additionalVariables.end(); ++it)
	{
		log("Adding factors for additional variables requested in the queries.");

		// Check if the variable was already added.
		if (_variableNameToDai.find(*it) == _variableNameToDai.end())
		{ // Nope, we have to add it now
			log("Adding factors for additional variable requested in the queries: " + *it);
			// Check if this is one of the variables that we can reason about
			vector<string> elements;
			parseVariable(*it, elements);
			if ((elements.size()==4) && (starts_with(elements[0], "room")) &&
				(elements[1]=="object") && (elements[3]=="unexplored"))
			{
				int roomId = 0;
				try
				{
					roomId = boost::lexical_cast<unsigned int>(erase_first_copy(elements[0], "room"));
				}
				catch(...)
				{
					error("Incorrect room ID in the variable name (%s)!", elements[0].c_str());
				}

				createDaiObjectUnexploredFactor(roomId, elements[2], SpatialData::INROOM, "", "", 0.0);
			}
			else if ((elements.size()==6) && (starts_with(elements[0], "room"))
					&& (elements[1]=="object") && (elements[5]=="unexplored"))
			{
				int roomId = 0;
				try
				{
					roomId = boost::lexical_cast<unsigned int>(erase_first_copy(elements[0], "room"));
				}
				catch(...)
				{
					error("Incorrect room ID in the variable name (%s)!", elements[0].c_str());
				}

				// Get object ID and category
				vector<string> objElems;
				split(objElems, elements[4], is_any_of("-") );
				if (objElems.size()!=2)
				{
					error("Incorrect object category and/or id in the variable name (%s)!", elements[4].c_str());
				}

				createDaiObjectUnexploredFactor(roomId, elements[2], VariableNameGenerator::stringToRelation(elements[3]),
						  	  	  	  	  	    objElems[0], objElems[1], 0.0);
			}
			else
			{
				error("Inference requested for incorrect variable name (%s)!", it->c_str());
			}
		} // if
	} // for
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
		throw CASTException(exceptionMessage(__HERE__,
				"Incorrect number of variables in updateOutputsUsingImaginaryVariables"));
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
	// No need to check if the variables are in the right order, the factor is symmetric

	// Note: first fariable changes faster
	// Go over the second variable
	int index=0;
	for (unsigned int i2 = 0; i2<var2.states(); ++i2)
	{
		string var2ValueName = _roomCategories[i2];
		// Go over the first variable
		for (unsigned int i1 = 0; i1<var1.states(); ++i1)
		{
			string var1ValueName = _roomCategories[i1];
			double potential = getProbabilityValue(factor, var1ValueName, var2ValueName);
			if (potential<0)
				throw CASTException(exceptionMessage(__HERE__,
						"Potential not found for values '%s' and '%s'", var1ValueName.c_str(), var2ValueName.c_str()));
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
		ConceptualData::ProbabilityDistributions &resultDistributions)
{
	debug("Preparing inference results for inference query '"+queryString+"'");

	// Check
	if (queryVariables.size()!=1)
	{
		error("Unhandled query \'%s\'. This indicates serious implementation error.", queryString.c_str());
		return;
	}
	string varName = queryVariables[0];

	// Check if there is '=' in the name and if yes, extract the components.
	vector<string> elems;
	split(elems, varName, is_any_of("=") );
	varName=elems[0];
	string valueName;
	if (elems.size()>1)
		valueName=elems[1];

	// Find all variables that match
	vector< map<string, DaiVariable>::iterator > varIters;
	for (map<string, DaiVariable>::iterator it = _variableNameToDai.begin();
			it != _variableNameToDai.end(); ++it)
	{
		if ( wildcmp(varName.c_str(), it->first.c_str()))
				varIters.push_back(it);
	}
	if (varIters.empty())
	{
		string msg;
		msg = "Variable '" + varName + "' not found! Variables we know:";
		for (map<string, DaiVariable>::iterator varIter = _variableNameToDai.begin(); varIter
				!= _variableNameToDai.end(); ++varIter)
			msg += varIter->first + " ";
		log(msg.c_str());
		return;
	}

	for (unsigned int v=0; v<varIters.size(); ++v)
	{
		map<string, DaiVariable>::iterator varIter = varIters[v];

		// Prepare the resulting distribution
		SpatialProbabilities::ProbabilityDistribution resultDistribution;
		resultDistribution.description = queryString;
		resultDistribution.variableNameToPositionMap[varIter->first]=0;
		resultDistribution.massFunction.clear();

		// Retrieve marginal
		//        dai::Factor marginal = _junctionTree.belief(varIter->second.var);
		dai::Factor marginal = _bp.belief(varIter->second.var);
		// Convert to probability distribution
		for (unsigned int i = 0; i < marginal.nrStates(); ++i)
		{
			double marginalProb = marginal.get(i);
			string vn = varIter->second.valueIdToName[i];
			if ((valueName.empty()) || (vn==valueName))
			{
				SpatialProbabilities::StringRandomVariableValuePtr rvvPtr =
						new SpatialProbabilities::StringRandomVariableValue(
								vn);
				SpatialProbabilities::JointProbabilityValue jpv;
				jpv.probability = marginalProb;
				jpv.variableValues.push_back(rvvPtr);
				resultDistribution.massFunction.push_back(jpv);
			}
		}

		resultDistributions.push_back(resultDistribution);
	}
}


// -------------------------------------------------------
void ChainGraphInferencer::prepareFactorResult(std::string queryString,
		std::vector<std::string> queryVariables,
		ConceptualData::ProbabilityDistributions &resultDistributions)
{
	debug("Preparing results for factor query '"+queryString+"'");

	// Prepare the resulting distribution
	SpatialProbabilities::ProbabilityDistribution resultDistribution;
	resultDistribution.description = queryString;
	resultDistribution.massFunction.clear();

	// Check
	if (queryVariables.size()!=1)
	{
		error("Unhandled query \'%s\'.", queryString.c_str());
		return;
	}
	string varName = queryVariables[0];

	// Find the factor
	const dai::Factor *factor = 0;
	for(unsigned int i=0; i<_factorNames.size(); ++i)
	{
		if (_factorNames[i]==varName)
		{
			factor = &_factors[i];
		}
	}
	if (!factor)
	{
		string msg;
		msg = "Factor '" + varName + "' not found! Factors we know:";
		for (unsigned int i=0; i <_factorNames.size(); i++)
			msg += _factorNames[i] + " ";
		log(msg.c_str());
		return;
	}

	// Fill in variable names
	const vector<dai::Var> &daiVars = factor->vars().elements();
	vector<DaiVariable> vars;
	for(unsigned int i=0; i<daiVars.size(); ++i)
	{
		string varName;
		// Find the variable name
		for(std::map<std::string, DaiVariable>::iterator it = _variableNameToDai.begin();
				it!=_variableNameToDai.end(); ++it)
		{
			if (daiVars[i].label() == it->second.var.label())
			{
				varName = it->first;
				vars.push_back(it->second);
			}
		}
		resultDistribution.variableNameToPositionMap[varName]=i;
	}

	// Fill in the distribution
	for (unsigned int i = 0; i < factor->nrStates(); ++i)
	{
		SpatialProbabilities::JointProbabilityValue jpv;
		jpv.probability = factor->get(i);
		// Get variable values for this probability
		int modulo=1;
		for (unsigned int j=0; j<vars.size(); ++j)
		{
			// Calculate value number for this variable
			int valNo= (i % (modulo*daiVars[j].states()))/modulo;
			modulo*=daiVars[j].states();
			string valName = vars[j].valueIdToName[valNo];
			SpatialProbabilities::StringRandomVariableValuePtr rvvPtr =
					new SpatialProbabilities::StringRandomVariableValue(valName);
			jpv.variableValues.push_back(rvvPtr);
		}
		resultDistribution.massFunction.push_back(jpv);
	}

	resultDistributions.push_back(resultDistribution);
}


// -------------------------------------------------------
void ChainGraphInferencer::prepareImaginaryInferenceResult(std::string queryString,
		std::vector<std::string> queryVariables,
		ConceptualData::ProbabilityDistributions &resultDistributions)
{
	debug("Preparing imaginary inference results for inference query '"+queryString+"'");

	// Prepare the resulting distribution
	SpatialProbabilities::ProbabilityDistribution resultDistribution;
	resultDistribution.description = queryString;
	for(unsigned int i=0; i<queryVariables.size(); ++i)
		resultDistribution.variableNameToPositionMap[queryVariables[i]]=i;
	resultDistribution.massFunction.clear();

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
	resultDistribution.massFunction.push_back(jpv1);

	SpatialProbabilities::IntRandomVariableValuePtr rvv2Ptr =
			new SpatialProbabilities::IntRandomVariableValue(0);
	SpatialProbabilities::JointProbabilityValue jpv2;
	jpv2.probability=1.0 - prob;
	jpv2.variableValues.push_back(rvv2Ptr);
	resultDistribution.massFunction.push_back(jpv2);

	resultDistributions.push_back(resultDistribution);
}


// -------------------------------------------------------
void ChainGraphInferencer::parseVariable(string variableName, vector<string> &elements)
{
	// Extract variable names
	trim(variableName);
	split(elements, variableName, is_any_of("_") );
}


// -------------------------------------------------------
void ChainGraphInferencer::parseQuery(string queryString, ConceptualData::QueryType type, vector<string> &variables)
{
	// Check the query string
	if (type!=ConceptualData::FACTORQUERY)
	{
		if ( (queryString.length()<4) || (queryString[0]!='p') ||
			 (queryString[1]!='(') || (queryString[queryString.length()-1]!=')') )
		{
			error("Malformed ChainGraphInferencer query string \'%s\' of type %i! This indicates a serious implementation error!",
					queryString.c_str(), type);
			return;
		}
		erase_head(queryString, 2);
		erase_tail(queryString, 1);
		// Extract variable names
		split( variables, queryString, is_any_of(", ") );
	}
	else
		variables.push_back(queryString);

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
		throw CASTException(exceptionMessage(__HERE__,
				"Did not receive information from Default.SA. Is everything started?"));

	string factorStr;

	// Get the room_category1, room_category2 factor
	factorStr = "f(room_category1,room_category2)";
	_defaultKnowledgeFactors[factorStr] =
			_defaultChainGraphInferencerServerInterfacePrx->getFactor(factorStr);
	if (_defaultKnowledgeFactors[factorStr].massFunction.empty())
		throw CASTException(exceptionMessage(__HERE__,
				"Did not receive information from Default.SA. Is everything started?"));


	// Get the room1_category -> object_xxx_property factors
	// THIS CODE IS TEMPORARILY DISABLED AND PROBABILITIES ARE INSTEAD TAKEN FROM THE FLAT FILE
	// THIS IS ONLY A HACK FOR THE AVS PAPER
//	for(unsigned int i=0; i<_objectPropertyVariables.size(); ++i)
//	{
//		factorStr = "f(room_category1,"+_objectPropertyVariables[i]+")";
//		_defaultKnowledgeFactors[factorStr] =
//				_defaultChainGraphInferencerServerInterfacePrx->getFactor(factorStr);
//		if (_defaultKnowledgeFactors[factorStr].massFunction.empty())
//			throw CASTException("Did not receive information from Default.SA. Is everything started?");
//	}
	// Open the AVS default knowledge file
	loadAvsDefaultKnowledge();

	// Get the room1_category -> shape_property factor
	factorStr = "f(room_category1,shape_property)";
	_defaultKnowledgeFactors[factorStr] =
			_defaultChainGraphInferencerServerInterfacePrx->getFactor(factorStr);
	if (_defaultKnowledgeFactors[factorStr].massFunction.empty())
		throw CASTException(exceptionMessage(__HERE__,
				"Did not receive information from Default.SA. Is everything started?"));

	// Get the room1_category -> appearance_property factor
	factorStr = "f(room_category1,appearance_property)";
	_defaultKnowledgeFactors[factorStr] =
			_defaultChainGraphInferencerServerInterfacePrx->getFactor(factorStr);
	if (_defaultKnowledgeFactors[factorStr].massFunction.empty())
		throw CASTException(exceptionMessage(__HERE__, "Did not receive information from Default.SA. Is everything started?"));
}


// -------------------------------------------------------
void ChainGraphInferencer::loadAvsDefaultKnowledge()
{
	map<string, map<string, double> > defKn;
	ifstream avsFile(_avsDefaultKnowledgeFileName.c_str());
	if (!avsFile.is_open())
		throw CASTException(exceptionMessage(__HERE__, "Cannot open the AVS default knowledge file!"));

	char *line = new char[256];
	while (avsFile.good())
	{
		string targetObjectCategory;
		string supportObjectCategory;
		SpatialData::SpatialRelation relation;
		string roomCategory;
		double probability;

		try
		{
			avsFile.getline(line, 256);
			vector<string> splitLine;
			split(splitLine, line, is_any_of(" ") );
			if (splitLine.size()==4)
			{
				if (splitLine[0]=="INROOM")
				{
					targetObjectCategory = splitLine[1];
					roomCategory = splitLine[2];
					probability = boost::lexical_cast<double>(splitLine[3]);
					relation = SpatialData::INROOM;
				}
				else continue;
			}
			else if (splitLine.size()==5)
			{
				if (splitLine[0]=="ON")
				{
					targetObjectCategory = splitLine[1];
					supportObjectCategory = splitLine[2];
					roomCategory = splitLine[3];
					probability = boost::lexical_cast<double>(splitLine[4]);
					relation = SpatialData::ON;
				}
				else if (splitLine[0]=="INOBJECT")
				{
					targetObjectCategory = splitLine[1];
					supportObjectCategory = splitLine[2];
					roomCategory = splitLine[3];
					probability = boost::lexical_cast<double>(splitLine[4]);
					relation = SpatialData::INOBJECT;
				}
				else continue;
			}
			else continue;
		}
		catch(...)
		{
			throw CASTException(exceptionMessage(__HERE__, "Incorrect AVS default knowledge file!"));
		}

		// For object property variable name
		string objVarName = VariableNameGenerator::getDefaultObjectPropertyVarName(targetObjectCategory,
												relation, supportObjectCategory);

		defKn[objVarName][roomCategory] = probability;
	}
	delete [] line;
	avsFile.close();


	// Form all the factors
	for (map<string, map<string, double> >::iterator it = defKn.begin(); it != defKn.end(); ++it)
	{
		// Create a factor
		string objVarName = it->first;
		string factorStr = "f(room_category1," + objVarName + ")";
		SpatialProbabilities::ProbabilityDistribution factor;
		factor.description=factorStr;
		factor.variableNameToPositionMap["room_category1"]=0;
		factor.variableNameToPositionMap[objVarName]=1;

		// Set of room categories that we need to add value for
		set<string> roomCategories( _roomCategories.begin(), _roomCategories.end() );

		// Let's iterate over the knowledge and fill values of what we know
		for(map<string, double>::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2)
		{
			roomCategories.erase(it2->first); // Remove from the to-do list

			// Existance of object
			SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it2->first);
			SpatialProbabilities::BoolRandomVariableValuePtr objectRVVPtr =
					new SpatialProbabilities::BoolRandomVariableValue(true);
			SpatialProbabilities::JointProbabilityValue jpv1;
			jpv1.probability=it2->second;
			jpv1.variableValues.push_back(roomCategory1RVVPtr);
			jpv1.variableValues.push_back(objectRVVPtr);
			factor.massFunction.push_back(jpv1);

			// Non-existance of object
			roomCategory1RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it2->first);
			objectRVVPtr =
					new SpatialProbabilities::BoolRandomVariableValue(false);
			SpatialProbabilities::JointProbabilityValue jpv2;
			jpv2.probability = (1.0 - it2->second);
			jpv2.variableValues.push_back(roomCategory1RVVPtr);
			jpv2.variableValues.push_back(objectRVVPtr);
			factor.massFunction.push_back(jpv2);
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
			jpv1.probability=0; // TODO: SHOULD BE THE DEFAULT.SA VALUE WHICH IS A PARAMETER THERE!
			jpv1.variableValues.push_back(roomCategory1RVVPtr);
			jpv1.variableValues.push_back(objectRVVPtr);
			factor.massFunction.push_back(jpv1);

			// Non-existence of object
			roomCategory1RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(*it2);
			objectRVVPtr =
					new SpatialProbabilities::BoolRandomVariableValue(false);
			SpatialProbabilities::JointProbabilityValue jpv2;
			jpv2.probability = 1.0;  // TODO: SHOULD BE THE DEFAULT.SA VALUE WHICH IS A PARAMETER THERE!
			jpv2.variableValues.push_back(roomCategory1RVVPtr);
			jpv2.variableValues.push_back(objectRVVPtr);
			factor.massFunction.push_back(jpv2);
		}

		// Add the factor to the list
		_defaultKnowledgeFactors[factorStr] = factor;
	}
}


// -------------------------------------------------------
double ChainGraphInferencer::getPoissonLambda(const std::string &roomCategory,
		const std::string &objectCategory, SpatialData::SpatialRelation relation,
		const std::string &supportObjectCategory)
{
	// Check if we have this value in cache
	string cacheString = roomCategory + "_" + objectCategory + "_" + VariableNameGenerator::relationToString(relation) + "_"+
						 supportObjectCategory;
	if (_poissonLambdaCache.find(cacheString) != _poissonLambdaCache.end())
		return _poissonLambdaCache[cacheString];

	// Get the default object property factor
	string objectVariableName = VariableNameGenerator::getDefaultObjectPropertyVarName(
			objectCategory, relation, supportObjectCategory);
	string factorName = string("f(room_category1,")+objectVariableName+")";
	std::map<std::string, SpatialProbabilities::ProbabilityDistribution>::iterator dnfIt =
			_defaultKnowledgeFactors.find(factorName);
	if (dnfIt == _defaultKnowledgeFactors.end())
	{
		error("Factor \'%s\' not found. This usually means that the object was not in the tuple store.",
				factorName.c_str());
		return -1;
	}
	const SpatialProbabilities::ProbabilityDistribution &factor = dnfIt->second;

	// Get the probability & calculate lambda
	double probability = getProbabilityValue(factor, roomCategory, true);
	double lambda = -::log(1.0 - probability);

	// Save to cache
	_poissonLambdaCache[cacheString] = lambda;

	return lambda;
}


// -------------------------------------------------------
ConceptualData::VariableInfos ChainGraphInferencer::TestingServer::getVariables(const Ice::Current &)
{
	pthread_mutex_lock(&_chainGraphInferencer->_graphMutex);

	ConceptualData::VariableInfos vis;

	for (std::map<std::string, DaiVariable>::iterator i = _chainGraphInferencer->_variableNameToDai.begin();
			i!=_chainGraphInferencer->_variableNameToDai.end(); ++i)
	{
		ConceptualData::VariableInfo vi;
		vi.name = i->first;
		for (unsigned int j=0; j<i->second.valueIdToName.size(); ++j)
			vi.values.push_back(i->second.valueIdToName[j]);
		vis[i->second.var.label()] = vi;
	}

	pthread_mutex_unlock(&_chainGraphInferencer->_graphMutex);

	return vis;
}


// -------------------------------------------------------
ConceptualData::FactorInfos ChainGraphInferencer::TestingServer::getFactors(const Ice::Current &)
{
	pthread_mutex_lock(&_chainGraphInferencer->_graphMutex);

	ConceptualData::FactorInfos fis;

	for (unsigned int i = 0; i< _chainGraphInferencer->_factors.size(); ++i)
	{
		ConceptualData::FactorInfo fi;
		fi.name = _chainGraphInferencer->_factorNames[i];
		vector<dai::Var> vars = _chainGraphInferencer->_factors[i].vars().elements();
		for(unsigned int j=0; j<vars.size(); j++)
			fi.variables.push_back(vars[j].label());
		fis[i] = fi;
	}

	pthread_mutex_unlock(&_chainGraphInferencer->_graphMutex);

	return fis;
}


// -------------------------------------------------------
int ChainGraphInferencer::wildcmp(const char *wild, const char *string)
{
  // Written by Jack Handy - jakkhandy@hotmail.com

  const char *cp = NULL, *mp = NULL;

  while ((*string) && (*wild != '*')) {
    if ((*wild != *string) && (*wild != '?')) {
      return 0;
    }
    wild++;
    string++;
  }

  while (*string) {
    if (*wild == '*') {
      if (!*++wild) {
        return 1;
      }
      mp = wild;
      cp = string+1;
    } else if ((*wild == *string) || (*wild == '?')) {
      wild++;
      string++;
    } else {
      wild = mp;
      string = cp++;
    }
  }

  while (*wild == '*') {
    wild++;
  }
  return !*wild;
}


} // namespace def
