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

	// QueryHandler name
	if((it = _config.find("--defaultchaingraphinferencer")) != _config.end())
	{
		_defaultChainGraphInferencerName = it->second;
	}
	if((it = _config.find("--save-graph")) != _config.end())
	{
		_saveGraphFileName = it->second;
	}
	if((it = _config.find("--save-graph-info")) != _config.end())
	{
		_saveGraphInfoFileName = it->second;
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

			debug("Processing inference query '"+q.queryPtr->queryString+"'");

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
					runAllInferences();

				// Prepare the result distribution
				prepareInferenceResult(q.queryPtr->queryString, &(inferenceResultPtr->result));
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
ChainGraphInferencer::DaiVariable&
ChainGraphInferencer::createDaiVariable(const string &name, const vector<string> &values)
{
	if (_variableNameToDai.find(name) == _variableNameToDai.end())
	{
		DaiVariable &dv = _variableNameToDai[name];
		dv.name = name;
		dv.var = dai::Var(_variableNameToDai.size(), values.size());
		for (unsigned int i=0; i<values.size(); ++i)
		{
			dv.valueIdToName[i] = values[i];
		}
	}
    return _variableNameToDai[name];
}

ChainGraphInferencer::DaiVariable& ChainGraphInferencer::RoomCategoryVar(int roomId)
{
	string varname = "room"+lexical_cast<string>(roomId)+"_category";
	return createDaiVariable(varname, _roomCategories);
}

ChainGraphInferencer::DaiVariable& ChainGraphInferencer::PlaceShapePropertyVar(int placeId)
{
	string varname = "place"+lexical_cast<string>(placeId)+"_shape_property";
	return createDaiVariable(varname, _shapes);
}

ChainGraphInferencer::DaiVariable& ChainGraphInferencer::PlaceAppearancePropertyVar(int placeId)
{
	string varname = "place"+lexical_cast<string>(placeId)+"_appearance_property";
	return createDaiVariable(varname, _appearances);
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
void ChainGraphInferencer::createDaiSingleRoomFactor(int room1Id)
{
	// Create variables
	DaiVariable &dv1 = RoomCategoryVar(room1Id);
	debug("Creating DAI single room factor for variable '%s'", dv1.name.c_str());

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
		const string& objectVariableName, bool objectExists)
{
	string factorName = "f(room_category1,"+objectVariableName+")";
	const SpatialProbabilities::ProbabilityDistribution *factor =
		getDefaultProbabilityDistribution(factorName);
	if (!factor) return;

	// Create variables
	DaiVariable &dv1 = RoomCategoryVar(room1Id);
	debug("Creating DAI observed object property factor for variable '%s' and object '%s'", dv1.name.c_str(),
			objectVariableName.c_str() );

	// Create factor
	dai::Factor daiFactor( dv1.var );
	// Note: first fariable changes faster
	// Go over the second variable
	int roomCatCount = _roomCategories.size();
	int index=0;
	for (int i1 = 0; i1<roomCatCount; ++i1)
	{
		string var1ValueName = dv1.valueIdToName[i1];
		double potential = getProbabilityValue(*factor, var1ValueName, objectExists);
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
void ChainGraphInferencer::createDaiObservedShapePropertyFactor(int placeId,
		ConceptualData::ValuePotentialPairs dist)
{
	// Create variables
	DaiVariable &dv = PlaceShapePropertyVar(placeId);
	debug("Creating DAI observed shape property factor for variable '%s'", dv.name.c_str());

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
void ChainGraphInferencer::createDaiObservedAppearancePropertyFactor(int placeId,
		ConceptualData::ValuePotentialPairs dist)
{
	// Create variables
	DaiVariable &dv = PlaceAppearancePropertyVar(placeId);
	debug("Creating DAI observed shape property factor for variable '%s'", dv.name.c_str());

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
void ChainGraphInferencer::createDaiFactor(const string &name, DaiVariable &dv1, DaiVariable &dv2)
{
	debug("Creating DAI factor '%s' for variables '%s' and '%s'", name.c_str(), dv1.name.c_str(), dv2.name.c_str() );
	const SpatialProbabilities::ProbabilityDistribution *factor = getDefaultProbabilityDistribution(name);
	if(!factor) return;

	// Create factor
	dai::Factor daiFactor( dai::VarSet( dv1.var, dv2.var ) );

	// Note: first fariable changes faster
	// Go over the second variable
	int index=0;
	typedef pair<int,string> item_type;
	foreach(const item_type &var2ValueName, dv2.valueIdToName)
	{
		// Go over the first variable
		foreach(const item_type &var1ValueName, dv1.valueIdToName)
		{
			double potential = getProbabilityValue(*factor, var1ValueName.second, var2ValueName.second);
			if (potential<0)
				throw CASTException("Potential not found for values '"+var1ValueName.second+"' and '"+var2ValueName.second+"'");
			daiFactor.set(index, potential);
			++index;
		}
	}

	// Add factor to the list
	_factors.push_back(daiFactor);
	_factorNames.push_back(name);
}

// -------------------------------------------------------
void ChainGraphInferencer::addDaiFactors()
{
	// Let's go through all the room connections and add category connectivity factors
	foreach(const ConceptualData::RoomConnectivityInfo &rci, _worldStateRoomConnections)
		createDaiFactor("f(room_category1,room_category2)", RoomCategoryVar(rci.room1Id), RoomCategoryVar(rci.room2Id) );

	// Add single room factors. Should change nothing, but help if there are unconnected or
	// single rooms without any properties.
	for (unsigned int i=0; i<_worldStateRooms.size(); ++i)
		createDaiSingleRoomFactor(_worldStateRooms[i].roomId);

	// Property factors
	// Go over all rooms
	foreach(const ConceptualData::ComaRoomInfo &cri, _worldStateRooms)
	{
		// No through all the places in the room
		foreach(const ConceptualData::PlaceInfo &pi, cri.places)
		{
			// Object properties
			foreach(const ConceptualData::ObjectPlacePropertyInfo &oppi, pi.objectProperties)
				createDaiObservedObjectPropertyFactor(cri.roomId,
						"object_"+oppi.category+"_property", oppi.present);

			// Shape properties
			foreach(const ConceptualData::ShapePlacePropertyInfo &sppi, pi.shapeProperties)
			{
				createDaiFactor("f(room_category1,shape_property)", RoomCategoryVar(cri.roomId), PlaceShapePropertyVar(pi.placeId));
				createDaiObservedShapePropertyFactor(pi.placeId, sppi.distribution);
			}

			// Appearance properties
			foreach(const ConceptualData::AppearancePlacePropertyInfo &appi, pi.appearanceProperties)
			{
				createDaiFactor("f(room_category1,appearance_property)", RoomCategoryVar(cri.roomId), PlaceAppearancePropertyVar(pi.placeId));
				createDaiObservedAppearancePropertyFactor(pi.placeId, appi.distribution);
			}
		}
	}
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
void ChainGraphInferencer::prepareInferenceResult(string queryString,
		SpatialProbabilities::ProbabilityDistribution *resultDistribution)
{
	debug("Preparing inference results for inference query '"+queryString+"'");

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
	vector<string> variables;
	split( variables, variableString, is_any_of(", ") );

	// Prepare the resulting distribution
	resultDistribution->description = queryString;
	for(unsigned int i=0; i<variables.size(); ++i)
		resultDistribution->variableNameToPositionMap[variables[i]]=i;
	resultDistribution->massFunction.clear();

	// Check what we need to return
	if (variables.size()==1)
	{
		string varName = variables[0];
		// Find the variable
		map<string, DaiVariable>::iterator varIter = _variableNameToDai.find(varName);
		if (varIter==_variableNameToDai.end())
		{
			string msg;
			msg = "Variable '"+varName+"' not found! Variables we know:";
			for (varIter = _variableNameToDai.begin(); varIter!=_variableNameToDai.end(); ++varIter)
				msg+=varIter->first+" ";
			log(msg.c_str());
			return;
		}
		// Retrieve marginal
//        dai::Factor marginal = _junctionTree.belief(varIter->second.var);
        dai::Factor marginal = _bp.belief(varIter->second.var);
        // Convert to probability distribution
        for(unsigned int i=0; i<marginal.nrStates(); ++i)
        {
        	double marginalProb = marginal.get(i);
    		SpatialProbabilities::StringRandomVariableValuePtr rvvPtr =
    				new SpatialProbabilities::StringRandomVariableValue(
    						varIter->second.valueIdToName[i]);
    		SpatialProbabilities::JointProbabilityValue jpv;
    		jpv.probability=marginalProb;
    		jpv.variableValues.push_back(rvvPtr);
    		resultDistribution->massFunction.push_back(jpv);
			resultDistribution->massFunction.push_back(jpv);
        }
	}
	else
	{
		error("Unhandled query \'%s\'. This indicates serious implementation error.", queryString.c_str());
		return;
	}
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

	vector<string> factors;

	// Get the room_category1, room_category2 factor
	factors.push_back("f(room_category1,room_category2)");

	// Get the room1_category -> object_xxx_property factors
	for(unsigned int i=0; i<_objectPropertyVariables.size(); ++i)
		factors.push_back("f(room_category1,"+_objectPropertyVariables[i]+")");

	// Get the room1_category -> appearance_property factor
	factors.push_back("f(room_category1,appearance_property)");

	// Get the room1_category -> shape_property factor
	factors.push_back("f(room_category1,shape_property)");

	// Fetch all the default factors
	foreach(const string &factorStr, factors)
	{
		_defaultKnowledgeFactors[factorStr] =
				_defaultChainGraphInferencerServerInterfacePrx->getFactor(factorStr);
		if (_defaultKnowledgeFactors[factorStr].massFunction.empty())
			throw CASTException("Did not receive information from Default.SA. Is everything started?");
	}
}

// -------------------------------------------------------
const SpatialProbabilities::ProbabilityDistribution*
ChainGraphInferencer::getDefaultProbabilityDistribution(const string& factorName) const
{
	std::map<std::string, SpatialProbabilities::ProbabilityDistribution>::const_iterator dnfIt =
			_defaultKnowledgeFactors.find(factorName);
	if (dnfIt == _defaultKnowledgeFactors.end())
	{
		error("Factor \'%s\' not found. This indicates a serious implementatation error!", factorName.c_str());
		return 0;
	}
	return &dnfIt->second;
}

} // namespace def
