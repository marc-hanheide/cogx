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
	// Where to load object information from?
	if((it = _config.find("--objects-from-hfc")) != _config.end())
	{ // Get object info from the HFC
		_loadObjectsFrom = LOF_HFC;
	}
	else if((it = _config.find("--objects-from-defaultprob")) != _config.end())
	{ // Get object info from the defaultprob file used by AVS
		_loadObjectsFrom = LOF_DEFAULTPROB;
		_objectsFileName = it->second;

		if (_objectsFileName.empty())
			throw CASTException(exceptionMessage(__HERE__, "Provide the AVS default knowledge file!"));
	}
	else
	{
		throw CASTException(exceptionMessage(__HERE__, "Please select the source of object information (--objects-from-hfc or --objects-from-defaultprob)."));
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

	// Connectivity
	vector<string> roomCat1Vector;
	config.getValueList("default.room_category_connectivity", "room_category1", roomCat1Vector);
	vector<string> roomCat2Vector;
	config.getValueList("default.room_category_connectivity", "room_category2", roomCat2Vector);
	vector<double> potentialVector;
	config.getValueList("default.room_category_connectivity", "potential", potentialVector);

	if ( (roomCat1Vector.size()!=roomCat2Vector.size()) ||
		 (roomCat1Vector.size()!=potentialVector.size()) )
		throw CASTException("The numbers of room_category1, room_category2 and potential keys do not match.");

	for(unsigned int i=0; i<roomCat1Vector.size(); ++i)
	{
		RoomCategoryConnectivity rcc;
		rcc.roomCategory1=roomCat1Vector[i];
		rcc.roomCategory2=roomCat2Vector[i];
		rcc.potential=potentialVector[i];
		_roomCategoryConnectivity.push_back(rcc);
	}

	// Object observation model
	_defaultObjectTruePositiveProbability =
			config.getValue("default.default_object_observation_model.true_positive_probability", 1.0);
	_defaultObjectTrueNegativeProbability =
			config.getValue("default.default_object_observation_model.true_negative_probability", 1.0);

	vector<string> objCatVector;
	config.getValueList("default.object_observation_models", "object_category", objCatVector);
	vector<string> relationVector;
	config.getValueList("default.object_observation_models", "relation", relationVector);
	vector<string> supportObjCatVector;
	config.getValueList("default.object_observation_models", "support_object_category", supportObjCatVector);
	vector<double> truePosProbVector;
	config.getValueList("default.object_observation_models", "true_positive_probability", truePosProbVector);
	vector<double> trueNegProbVector;
	config.getValueList("default.object_observation_models", "true_negative_probability", trueNegProbVector);

	if ( (objCatVector.size()!=relationVector.size()) ||
		 (objCatVector.size()!=supportObjCatVector.size()) ||
		 (objCatVector.size()!=truePosProbVector.size()) ||
		 (objCatVector.size()!=trueNegProbVector.size()) )
		throw CASTException("The numbers of object_category, relation, support_object_category, true_positive_probability and true_negative_probability keys do not match.");

	for(unsigned int i=0; i<objCatVector.size(); ++i)
	{
		ObjectObservationModel oom;
		oom.objectCategory = objCatVector[i];
		oom.relation = (relationVector[i]=="in")?SpatialData::INOBJECT:((relationVector[i]=="on")?SpatialData::ON:SpatialData::INROOM);
		oom.supportObjectCategory = supportObjCatVector[i];
		oom.truePositiveProbability = truePosProbVector[i];
		oom.trueNegativeProbability = trueNegProbVector[i];
		_objectObservationModel.push_back(oom);
	}

	// Properties (shape)
	roomCat1Vector.clear();
	config.getValueList("default.shape_property_given_room_category", "room_category1", roomCat1Vector);
	vector<string> shapePropVector;
	config.getValueList("default.shape_property_given_room_category", "shape_property", shapePropVector);
	potentialVector.clear();
	config.getValueList("default.shape_property_given_room_category", "probability", potentialVector);

	if ( (roomCat1Vector.size()!=shapePropVector.size()) ||
		 (roomCat1Vector.size()!=potentialVector.size()) )
		throw CASTException("The numbers of room_category1, shape_property and probability keys do not match.");

	for(unsigned int i=0; i<roomCat1Vector.size(); ++i)
	{
		ShapePropertyGivenRoomCategory spgrc;
		spgrc.roomCategory1=roomCat1Vector[i];
		spgrc.shapeProperty=shapePropVector[i];
		spgrc.probability=potentialVector[i];
		_shapePropertyGivenRoomCategory.push_back(spgrc);
	}

	// Properties (shape) default
	shapePropVector.clear();
	config.getValueList("default.default_shape_property_given_room_category", "shape_property", shapePropVector);
	potentialVector.clear();
	config.getValueList("default.default_shape_property_given_room_category", "probability", potentialVector);

	if ( shapePropVector.size()!=potentialVector.size() )
		throw CASTException("The numbers of shape_property and probability keys do not match.");

	for(unsigned int i=0; i<shapePropVector.size(); ++i)
	{
		DefaultShapePropertyGivenRoomCategory dspgrc;
		dspgrc.shapeProperty=shapePropVector[i];
		dspgrc.probability=potentialVector[i];
		_defaultShapePropertyGivenRoomCategory.push_back(dspgrc);
	}


	// Properties (size)
	roomCat1Vector.clear();
	config.getValueList("default.size_property_given_room_category", "room_category1", roomCat1Vector);
	vector<string> sizePropVector;
	config.getValueList("default.size_property_given_room_category", "size_property", sizePropVector);
	potentialVector.clear();
	config.getValueList("default.size_property_given_room_category", "probability", potentialVector);

	if ( (roomCat1Vector.size()!=sizePropVector.size()) ||
		 (roomCat1Vector.size()!=potentialVector.size()) )
		throw CASTException("The numbers of room_category1, size_property and probability keys do not match.");

	for(unsigned int i=0; i<roomCat1Vector.size(); ++i)
	{
		SizePropertyGivenRoomCategory spgrc;
		spgrc.roomCategory1=roomCat1Vector[i];
		spgrc.sizeProperty=sizePropVector[i];
		spgrc.probability=potentialVector[i];
		_sizePropertyGivenRoomCategory.push_back(spgrc);
	}

	// Properties (size) default
	sizePropVector.clear();
	config.getValueList("default.default_size_property_given_room_category", "size_property", sizePropVector);
	potentialVector.clear();
	config.getValueList("default.default_size_property_given_room_category", "probability", potentialVector);

	if ( sizePropVector.size()!=potentialVector.size() )
		throw CASTException("The numbers of size_property and probability keys do not match.");

	for(unsigned int i=0; i<sizePropVector.size(); ++i)
	{
		DefaultSizePropertyGivenRoomCategory dspgrc;
		dspgrc.sizeProperty=sizePropVector[i];
		dspgrc.probability=potentialVector[i];
		_defaultSizePropertyGivenRoomCategory.push_back(dspgrc);
	}

	// Properties (appearance)
	roomCat1Vector.clear();
	config.getValueList("default.appearance_property_given_room_category", "room_category1", roomCat1Vector);
	vector<string> appearancePropVector;
	config.getValueList("default.appearance_property_given_room_category", "appearance_property", appearancePropVector);
	potentialVector.clear();
	config.getValueList("default.appearance_property_given_room_category", "probability", potentialVector);

	if ( (roomCat1Vector.size()!=appearancePropVector.size()) ||
		 (roomCat1Vector.size()!=potentialVector.size()) )
		throw CASTException(exceptionMessage(__HERE__, "The numbers of room_category1, appearance_property and probability keys do not match. %d %d %d",
				roomCat1Vector.size(), appearancePropVector.size(), potentialVector.size()));

	for(unsigned int i=0; i<roomCat1Vector.size(); ++i)
	{
		AppearancePropertyGivenRoomCategory spgrc;
		spgrc.roomCategory1=roomCat1Vector[i];
		spgrc.appearanceProperty=appearancePropVector[i];
		spgrc.probability=potentialVector[i];
		_appearancePropertyGivenRoomCategory.push_back(spgrc);
	}

	// Properties (appearance) default
	appearancePropVector.clear();
	config.getValueList("default.default_appearance_property_given_room_category", "appearance_property", appearancePropVector);
	potentialVector.clear();
	config.getValueList("default.default_appearance_property_given_room_category", "probability", potentialVector);

	if ( appearancePropVector.size()!=potentialVector.size() )
		throw CASTException("The numbers of appearance_property and probability keys do not much.");

	for(unsigned int i=0; i<appearancePropVector.size(); ++i)
	{
		DefaultAppearancePropertyGivenRoomCategory dspgrc;
		dspgrc.appearanceProperty=appearancePropVector[i];
		dspgrc.probability=potentialVector[i];
		_defaultAppearancePropertyGivenRoomCategory.push_back(dspgrc);
	}


	// Properties (human assertion)
	roomCat1Vector.clear();
	config.getValueList("default.human_asserted_knowledge_property_given_room_category", "room_category1", roomCat1Vector);
	vector<string> humanAssertionPropVector;
	config.getValueList("default.human_asserted_knowledge_property_given_room_category", "human_assertion_property", humanAssertionPropVector);
	potentialVector.clear();
	config.getValueList("default.human_asserted_knowledge_property_given_room_category", "probability", potentialVector);

	if ( (roomCat1Vector.size()!=humanAssertionPropVector.size()) ||
		 (roomCat1Vector.size()!=potentialVector.size()) )
		throw CASTException(exceptionMessage(__HERE__, "The numbers of room_category1, human_assertion_property and probability keys do not match. %d %d %d",
				roomCat1Vector.size(), humanAssertionPropVector.size(), potentialVector.size()));

	for(unsigned int i=0; i<roomCat1Vector.size(); ++i)
	{
		HumanAssertionPropertyGivenRoomCategory hapgrc;
		hapgrc.roomCategory1=roomCat1Vector[i];
		hapgrc.humanAssertionProperty=humanAssertionPropVector[i];
		hapgrc.probability=potentialVector[i];
		_humanAssertionPropertyGivenRoomCategory.push_back(hapgrc);
	}

	// Properties (human assertion) default
	_defaultMatchingHumanAssertionProbability =
			config.getValue("default.default_human_asserted_knowledge_property_given_room_category.matching_probability", 0.95);


	// Load object information from AVS DefaultProb file
	if (_loadObjectsFrom == LOF_DEFAULTPROB)
	{
		loadAvsDefaultKnowledge();
		generateLists(); // If we load knowledge from the AVS data file, we can generate the lists already now so that they are available in start()
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

	log("-> Shape property given room category:");
	for(list<ShapePropertyGivenRoomCategory>::iterator it = _shapePropertyGivenRoomCategory.begin();
			it!=_shapePropertyGivenRoomCategory.end(); ++it)
	{
		log("----> rc1=%s sp=%s p=%f",
				it->roomCategory1.c_str(), it->shapeProperty.c_str(), it->probability);
	}

	log("-> Default Shape property given room category:");
	for(list<DefaultShapePropertyGivenRoomCategory>::iterator it = _defaultShapePropertyGivenRoomCategory.begin();
			it!=_defaultShapePropertyGivenRoomCategory.end(); ++it)
	{
		log("----> sp=%s p=%f",
				it->shapeProperty.c_str(), it->probability);
	}


	log("-> Size property given room category:");
	for(list<SizePropertyGivenRoomCategory>::iterator it = _sizePropertyGivenRoomCategory.begin();
			it!=_sizePropertyGivenRoomCategory.end(); ++it)
	{
		log("----> rc1=%s sp=%s p=%f",
				it->roomCategory1.c_str(), it->sizeProperty.c_str(), it->probability);
	}

	log("-> Default Size property given room category:");
	for(list<DefaultSizePropertyGivenRoomCategory>::iterator it = _defaultSizePropertyGivenRoomCategory.begin();
			it!=_defaultSizePropertyGivenRoomCategory.end(); ++it)
	{
		log("----> sp=%s p=%f",
				it->sizeProperty.c_str(), it->probability);
	}


	log("-> Appearance property given room category:");
	for(list<AppearancePropertyGivenRoomCategory>::iterator it = _appearancePropertyGivenRoomCategory.begin();
			it!=_appearancePropertyGivenRoomCategory.end(); ++it)
	{
		log("----> rc1=%s ap=%s p=%f",
				it->roomCategory1.c_str(), it->appearanceProperty.c_str(), it->probability);
	}

	log("-> Default Appearance property given room category:");
	for(list<DefaultAppearancePropertyGivenRoomCategory>::iterator it = _defaultAppearancePropertyGivenRoomCategory.begin();
			it!=_defaultAppearancePropertyGivenRoomCategory.end(); ++it)
	{
		log("----> ap=%s p=%f",
				it->appearanceProperty.c_str(), it->probability);
	}


	log("-> Human assertion property given room category:");
	for(list<HumanAssertionPropertyGivenRoomCategory>::iterator it = _humanAssertionPropertyGivenRoomCategory.begin();
			it!=_humanAssertionPropertyGivenRoomCategory.end(); ++it)
	{
		log("----> rc1=%s hap=%s p=%f",
				it->roomCategory1.c_str(), it->humanAssertionProperty.c_str(), it->probability);
	}
	log("-> Default matching human assertion probability: %g", _defaultMatchingHumanAssertionProbability);

}



// -------------------------------------------------------
void ChainGraphInferencer::start()
{
	// Load object information from HFC
	if (_loadObjectsFrom == LOF_HFC)
	{
		// Get the HFCServer interface proxy
		_hfcInterfacePrx =
			getIceServer<comadata::HFCInterface>(_hfcServerName);

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

			ObjectPropertyGivenRoomCategory hfcItem;
			hfcItem.roomCategory = room;
			hfcItem.objectCategory = object;
			hfcItem.supportObjectCategory = "";
			hfcItem.relation = SpatialData::INROOM;
			hfcItem.probability = lexical_cast<double>(probability);
			_objectPropertyGivenRoomCategory.push_back(hfcItem);

			debug("---> room=%s object=%s p=%f", hfcItem.roomCategory.c_str(), hfcItem.objectCategory.c_str(), hfcItem.probability);
		}

		generateLists(); // In case when we load from HFC, we can generate lists only now.
	} // if (_loadObjectsFrom = LOF_HFC)


	// Local filter on DefaultData::InferenceQuery
	addChangeFilter(createLocalTypeFilter<DefaultData::InferenceQuery>(cdl::ADD),
			new MemberFunctionChangeReceiver<ChainGraphInferencer>(this,
					&ChainGraphInferencer::inferenceQueryAdded));
}


// -------------------------------------------------------
void ChainGraphInferencer::generateLists()
{
	// Generate list of object categories and variables
	set<string> objectPropertyVariables;
	set<string> objectCategories;
	for(list<ObjectPropertyGivenRoomCategory>::iterator it = _objectPropertyGivenRoomCategory.begin();
			it!=_objectPropertyGivenRoomCategory.end(); ++it)
	{
		objectPropertyVariables.insert(
				VariableNameGenerator::getDefaultObjectPropertyVarName(
						it->objectCategory, it->relation, it->supportObjectCategory));
		objectCategories.insert(it->objectCategory);
	}
	_objectPropertyVariables = vector<string>(objectPropertyVariables.begin(), objectPropertyVariables.end());
	_objectCategories = vector<string>(objectCategories.begin(), objectCategories.end());

	// Generate list of room categories mentioned ANYWHERE
	set<string> roomCategories;
	for(list<ObjectPropertyGivenRoomCategory>::iterator it = _objectPropertyGivenRoomCategory.begin();
			it!=_objectPropertyGivenRoomCategory.end(); ++it)
		roomCategories.insert(it->roomCategory);
	for(std::list<ShapePropertyGivenRoomCategory>::iterator it =
			_shapePropertyGivenRoomCategory.begin();
			it!=_shapePropertyGivenRoomCategory.end(); ++it)
		roomCategories.insert(it->roomCategory1);
	for(std::list<SizePropertyGivenRoomCategory>::iterator it =
			_sizePropertyGivenRoomCategory.begin();
			it!=_sizePropertyGivenRoomCategory.end(); ++it)
		roomCategories.insert(it->roomCategory1);
	for(std::list<AppearancePropertyGivenRoomCategory>::iterator it =
			_appearancePropertyGivenRoomCategory.begin();
			it!=_appearancePropertyGivenRoomCategory.end(); ++it)
		roomCategories.insert(it->roomCategory1);
	_roomCategories = vector<string>(roomCategories.begin(), roomCategories.end());

	// Generate list of shapes
	set<string> shapes;
	for(std::list<ShapePropertyGivenRoomCategory>::iterator it =
			_shapePropertyGivenRoomCategory.begin();
			it!=_shapePropertyGivenRoomCategory.end(); ++it)
		shapes.insert(it->shapeProperty);
	_shapes = vector<string>(shapes.begin(), shapes.end());

	// Generate list of sizes
	set<string> sizes;
	for(std::list<SizePropertyGivenRoomCategory>::iterator it =
			_sizePropertyGivenRoomCategory.begin();
			it!=_sizePropertyGivenRoomCategory.end(); ++it)
		sizes.insert(it->sizeProperty);
	_sizes = vector<string>(sizes.begin(), sizes.end());

	// Generate list of appearances
	set<string> appearances;
	for(std::list<AppearancePropertyGivenRoomCategory>::iterator it =
			_appearancePropertyGivenRoomCategory.begin();
			it!=_appearancePropertyGivenRoomCategory.end(); ++it)
		appearances.insert(it->appearanceProperty);
	_appearances = vector<string>(appearances.begin(), appearances.end());

	// Generate list of human assertions. Include room categories into those as well.
	set<string> humanAssertions;
	humanAssertions.insert(roomCategories.begin(), roomCategories.end()); // Include room categories into those as well.
	for(std::list<HumanAssertionPropertyGivenRoomCategory>::iterator it =
			_humanAssertionPropertyGivenRoomCategory.begin();
			it!=_humanAssertionPropertyGivenRoomCategory.end(); ++it)
		humanAssertions.insert(it->humanAssertionProperty);
	_humanAssertions = vector<string>(humanAssertions.begin(), humanAssertions.end());

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
void ChainGraphInferencer::loadAvsDefaultKnowledge()
{
	ifstream avsFile(_objectsFileName.c_str());
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

		ObjectPropertyGivenRoomCategory item;
		item.roomCategory = roomCategory;
		item.objectCategory = targetObjectCategory;
		item.supportObjectCategory = supportObjectCategory;
		item.relation = relation;
		item.probability = probability;
		_objectPropertyGivenRoomCategory.push_back(item);

		debug("---> room=%s object=%s relation=%d supportObject=%s p=%f",
				item.roomCategory.c_str(), item.objectCategory.c_str(),
				item.relation, item.supportObjectCategory.c_str(),
				item.probability);
	}
	delete [] line;
	avsFile.close();
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
DefaultData::StringSeq
	ChainGraphInferencer::Server::getObjectCategories(const Ice::Current &)
{
	return _chainGraphInferencer->_objectCategories;
}


// -------------------------------------------------------
DefaultData::StringSeq
	ChainGraphInferencer::Server::getShapes(const Ice::Current &)
{
	return _chainGraphInferencer->_shapes;
}


// -------------------------------------------------------
DefaultData::StringSeq
	ChainGraphInferencer::Server::getSizes(const Ice::Current &)
{
	return _chainGraphInferencer->_sizes;
}


// -------------------------------------------------------
DefaultData::StringSeq
	ChainGraphInferencer::Server::getAppearances(const Ice::Current &)
{
	return _chainGraphInferencer->_appearances;
}


// -------------------------------------------------------
DefaultData::StringSeq
	ChainGraphInferencer::Server::getHumanAssertions(const Ice::Current &)
{
	return _chainGraphInferencer->_humanAssertions;
}


// -------------------------------------------------------
SpatialProbabilities::ProbabilityDistribution
	ChainGraphInferencer::Server::getFactor(const std::string &factorStr, const Ice::Current &)
{
	_chainGraphInferencer->debug("Received factor query '%s'",
			factorStr.c_str());


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
				string objectCategory;
				string supportObjectCategory;
				SpatialData::SpatialRelation relation;
				VariableNameGenerator::parseDefaultObjectPropertyVar(*it, objectCategory, relation, supportObjectCategory);

				_chainGraphInferencer->debug("Received query for object property factor for object=%s, relation=%d, support=%s",
						objectCategory.c_str(), relation, supportObjectCategory.c_str());

				// Set of room categories that we need to add value for
				set<string> roomCategories( _chainGraphInferencer->_roomCategories.begin(),
						_chainGraphInferencer->_roomCategories.end() );


				// Let's iterate over the knowledge and fill values of what we know
				for (std::list<ObjectPropertyGivenRoomCategory>::iterator
						it2=_chainGraphInferencer->_objectPropertyGivenRoomCategory.begin();
						it2!=_chainGraphInferencer->_objectPropertyGivenRoomCategory.end(); ++it2)
				{
					if ( (it2->objectCategory == objectCategory) && (it2->relation == relation) &&
						 ( (relation == SpatialData::INROOM) || (it2->supportObjectCategory == supportObjectCategory) )
					   )
					{
						roomCategories.erase(it2->roomCategory); // Remove from the to-do list

						// Existance of object
						SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
								new SpatialProbabilities::StringRandomVariableValue(it2->roomCategory);
						SpatialProbabilities::BoolRandomVariableValuePtr objectRVVPtr =
								new SpatialProbabilities::BoolRandomVariableValue(true);
						SpatialProbabilities::JointProbabilityValue jpv1;
						jpv1.probability=it2->probability;
						jpv1.variableValues.push_back(roomCategory1RVVPtr);
						jpv1.variableValues.push_back(objectRVVPtr);
						factor.massFunction.push_back(jpv1);

						// Non-existance of object
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



	// object_xxx_property -> object_xxx_observation factors
	if ((variables.size()==2) && (starts_with(variables[0],"object_")) &&
		 (starts_with(variables[1],"object_")) && (ends_with(variables[0],"_property")) &&
		 (ends_with(variables[1],"_observation")))
	{
		// Identify which factor we want
		for(vector<string>::iterator it = _chainGraphInferencer->_objectPropertyVariables.begin();
				it!=_chainGraphInferencer->_objectPropertyVariables.end(); ++it)
		{
			if (variables[0] == (*it))
			{ // We found our factor!
				string objectCategory;
				string supportObjectCategory;
				SpatialData::SpatialRelation relation;
				VariableNameGenerator::parseDefaultObjectPropertyVar(*it, objectCategory, relation, supportObjectCategory);
				// We assume that the observation variable matches the property variable, so we don't have to parse it.

				_chainGraphInferencer->debug("Received query for object observation factor for object=%s, relation=%d, support=%s",
						objectCategory.c_str(), relation, supportObjectCategory.c_str());

				// Let's try to find the observation model
				for (std::list<ObjectObservationModel>::iterator it2 = _chainGraphInferencer->_objectObservationModel.begin();
						it2 != _chainGraphInferencer->_objectObservationModel.end(); ++it2)
				{
					if ( (it2->objectCategory == objectCategory) && (it2->relation == relation) &&
							( (relation == SpatialData::INROOM) || (it2->supportObjectCategory == supportObjectCategory) )
					   )
					{
						// True positive (real=true, detected=true)
						SpatialProbabilities::BoolRandomVariableValuePtr realRVVPtr =
								new SpatialProbabilities::BoolRandomVariableValue(true);
						SpatialProbabilities::BoolRandomVariableValuePtr detectedRVVPtr =
								new SpatialProbabilities::BoolRandomVariableValue(true);
						SpatialProbabilities::JointProbabilityValue jpv1;
						jpv1.probability=it2->truePositiveProbability;
						jpv1.variableValues.push_back(realRVVPtr);
						jpv1.variableValues.push_back(detectedRVVPtr);
						factor.massFunction.push_back(jpv1);

						// False negative (real=true, detected=false)
						realRVVPtr =
								new SpatialProbabilities::BoolRandomVariableValue(true);
						detectedRVVPtr =
								new SpatialProbabilities::BoolRandomVariableValue(false);
						SpatialProbabilities::JointProbabilityValue jpv4;
						jpv4.probability=1.0 - it2->truePositiveProbability;
						jpv4.variableValues.push_back(realRVVPtr);
						jpv4.variableValues.push_back(detectedRVVPtr);
						factor.massFunction.push_back(jpv4);

						// True negative (real=false, detected=false)
						realRVVPtr =
								new SpatialProbabilities::BoolRandomVariableValue(false);
						detectedRVVPtr =
								new SpatialProbabilities::BoolRandomVariableValue(false);
						SpatialProbabilities::JointProbabilityValue jpv3;
						jpv3.probability=it2->trueNegativeProbability;
						jpv3.variableValues.push_back(realRVVPtr);
						jpv3.variableValues.push_back(detectedRVVPtr);
						factor.massFunction.push_back(jpv3);


						// False positive (real=false, detected=true)
						realRVVPtr =
								new SpatialProbabilities::BoolRandomVariableValue(false);
						detectedRVVPtr =
								new SpatialProbabilities::BoolRandomVariableValue(true);
						SpatialProbabilities::JointProbabilityValue jpv2;
						jpv2.probability=1.0-it2->trueNegativeProbability;
						jpv2.variableValues.push_back(realRVVPtr);
						jpv2.variableValues.push_back(detectedRVVPtr);
						factor.massFunction.push_back(jpv2);

						return factor;
					}
				}

				// If it's missing, we used default

				// True positive (real=true, detected=true)
				SpatialProbabilities::BoolRandomVariableValuePtr realRVVPtr =
						new SpatialProbabilities::BoolRandomVariableValue(true);
				SpatialProbabilities::BoolRandomVariableValuePtr detectedRVVPtr =
						new SpatialProbabilities::BoolRandomVariableValue(true);
				SpatialProbabilities::JointProbabilityValue jpv1;
				jpv1.probability=_chainGraphInferencer->_defaultObjectTruePositiveProbability;
				jpv1.variableValues.push_back(realRVVPtr);
				jpv1.variableValues.push_back(detectedRVVPtr);
				factor.massFunction.push_back(jpv1);

				// False negative (real=true, detected=false)
				realRVVPtr =
						new SpatialProbabilities::BoolRandomVariableValue(true);
				detectedRVVPtr =
						new SpatialProbabilities::BoolRandomVariableValue(false);
				SpatialProbabilities::JointProbabilityValue jpv4;
				jpv4.probability=1.0 - _chainGraphInferencer->_defaultObjectTruePositiveProbability;
				jpv4.variableValues.push_back(realRVVPtr);
				jpv4.variableValues.push_back(detectedRVVPtr);
				factor.massFunction.push_back(jpv4);

				// True negative (real=false, detected=false)
				realRVVPtr =
						new SpatialProbabilities::BoolRandomVariableValue(false);
				detectedRVVPtr =
						new SpatialProbabilities::BoolRandomVariableValue(false);
				SpatialProbabilities::JointProbabilityValue jpv3;
				jpv3.probability=_chainGraphInferencer->_defaultObjectTrueNegativeProbability;
				jpv3.variableValues.push_back(realRVVPtr);
				jpv3.variableValues.push_back(detectedRVVPtr);
				factor.massFunction.push_back(jpv3);

				// False positive (real=false, detected=true)
				realRVVPtr =
						new SpatialProbabilities::BoolRandomVariableValue(false);
				detectedRVVPtr =
						new SpatialProbabilities::BoolRandomVariableValue(true);
				SpatialProbabilities::JointProbabilityValue jpv2;
				jpv2.probability=1.0-_chainGraphInferencer->_defaultObjectTrueNegativeProbability;
				jpv2.variableValues.push_back(realRVVPtr);
				jpv2.variableValues.push_back(detectedRVVPtr);
				factor.massFunction.push_back(jpv2);



				return factor;
			}
		}
	}


	// room_category1 -> _shape_property
	if ((variables.size()==2) &&
		(variables[0]=="room_category1") && (variables[1]=="shape_property"))
	{
		// Set of all pairs category -> shape
		set< pair<string, string> > catShape;
		for ( vector<string>::iterator it = _chainGraphInferencer->_roomCategories.begin();
			  it!=_chainGraphInferencer->_roomCategories.end(); ++it )
		{
			for ( vector<string>::iterator it2 = _chainGraphInferencer->_shapes.begin();
				  it2!=_chainGraphInferencer->_shapes.end(); ++it2 )
			{
				catShape.insert(pair<string,string>((*it), (*it2)));
			}
		}

		// Add the connectivities from the data file
		for(list<ShapePropertyGivenRoomCategory>::iterator it =
				_chainGraphInferencer->_shapePropertyGivenRoomCategory.begin();
				it!=_chainGraphInferencer->_shapePropertyGivenRoomCategory.end(); ++it)
		{
			catShape.erase(pair<string, string>(it->roomCategory1, it->shapeProperty));

			SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->roomCategory1);
			SpatialProbabilities::StringRandomVariableValuePtr shapePropertyRVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->shapeProperty);
			SpatialProbabilities::JointProbabilityValue jpv;
			jpv.probability=it->probability;
			jpv.variableValues.push_back(roomCategory1RVVPtr);
			jpv.variableValues.push_back(shapePropertyRVVPtr);
			factor.massFunction.push_back(jpv);
		}

		// Add default potential for those connectivities that we don't know
		for (set< pair<string, string> >::iterator it=catShape.begin();
				it!=catShape.end(); ++it)
		{
			SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->first);
			SpatialProbabilities::StringRandomVariableValuePtr shapePropertyRVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->second);
			SpatialProbabilities::JointProbabilityValue jpv;
			for(std::list<DefaultShapePropertyGivenRoomCategory>::iterator it2 =
					_chainGraphInferencer->_defaultShapePropertyGivenRoomCategory.begin();
					it2!=_chainGraphInferencer->_defaultShapePropertyGivenRoomCategory.end(); ++it2)
			{
				if (it2->shapeProperty == it->second)
				{
					jpv.probability=it2->probability;
					break;
				}
			}
			jpv.variableValues.push_back(roomCategory1RVVPtr);
			jpv.variableValues.push_back(shapePropertyRVVPtr);
			factor.massFunction.push_back(jpv);
		}

		return factor;
	}


	// room_category1 -> _size_property
	if ((variables.size()==2) &&
		(variables[0]=="room_category1") && (variables[1]=="size_property"))
	{
		// Set of all pairs category -> size
		set< pair<string, string> > catSize;
		for ( vector<string>::iterator it = _chainGraphInferencer->_roomCategories.begin();
			  it!=_chainGraphInferencer->_roomCategories.end(); ++it )
		{
			for ( vector<string>::iterator it2 = _chainGraphInferencer->_sizes.begin();
				  it2!=_chainGraphInferencer->_sizes.end(); ++it2 )
			{
				catSize.insert(pair<string,string>((*it), (*it2)));
			}
		}

		// Add the connectivities from the data file
		for(list<SizePropertyGivenRoomCategory>::iterator it =
				_chainGraphInferencer->_sizePropertyGivenRoomCategory.begin();
				it!=_chainGraphInferencer->_sizePropertyGivenRoomCategory.end(); ++it)
		{
			catSize.erase(pair<string, string>(it->roomCategory1, it->sizeProperty));

			SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->roomCategory1);
			SpatialProbabilities::StringRandomVariableValuePtr sizePropertyRVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->sizeProperty);
			SpatialProbabilities::JointProbabilityValue jpv;
			jpv.probability=it->probability;
			jpv.variableValues.push_back(roomCategory1RVVPtr);
			jpv.variableValues.push_back(sizePropertyRVVPtr);
			factor.massFunction.push_back(jpv);
		}

		// Add default potential for those connectivities that we don't know
		for (set< pair<string, string> >::iterator it=catSize.begin();
				it!=catSize.end(); ++it)
		{
			SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->first);
			SpatialProbabilities::StringRandomVariableValuePtr sizePropertyRVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->second);
			SpatialProbabilities::JointProbabilityValue jpv;
			for(std::list<DefaultSizePropertyGivenRoomCategory>::iterator it2 =
					_chainGraphInferencer->_defaultSizePropertyGivenRoomCategory.begin();
					it2!=_chainGraphInferencer->_defaultSizePropertyGivenRoomCategory.end(); ++it2)
			{
				if (it2->sizeProperty == it->second)
				{
					jpv.probability=it2->probability;
					break;
				}
			}
			jpv.variableValues.push_back(roomCategory1RVVPtr);
			jpv.variableValues.push_back(sizePropertyRVVPtr);
			factor.massFunction.push_back(jpv);
		}

		return factor;
	}



	// room_category1 -> appearance_property
	if ((variables.size()==2) &&
		(variables[0]=="room_category1") && (variables[1]=="appearance_property"))
	{
		// Set of all pairs category -> appearance
		set< pair<string, string> > catAppearance;
		for ( vector<string>::iterator it = _chainGraphInferencer->_roomCategories.begin();
			  it!=_chainGraphInferencer->_roomCategories.end(); ++it )
		{
			for ( vector<string>::iterator it2 = _chainGraphInferencer->_appearances.begin();
				  it2!=_chainGraphInferencer->_appearances.end(); ++it2 )
			{
				catAppearance.insert(pair<string,string>((*it), (*it2)));
			}
		}

		// Add the connectivities from the data file
		for(list<AppearancePropertyGivenRoomCategory>::iterator it =
				_chainGraphInferencer->_appearancePropertyGivenRoomCategory.begin();
				it!=_chainGraphInferencer->_appearancePropertyGivenRoomCategory.end(); ++it)
		{
			catAppearance.erase(pair<string, string>(it->roomCategory1, it->appearanceProperty));

			SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->roomCategory1);
			SpatialProbabilities::StringRandomVariableValuePtr appearancePropertyRVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->appearanceProperty);
			SpatialProbabilities::JointProbabilityValue jpv;
			jpv.probability=it->probability;
			jpv.variableValues.push_back(roomCategory1RVVPtr);
			jpv.variableValues.push_back(appearancePropertyRVVPtr);
			factor.massFunction.push_back(jpv);
		}

		// Add default potential for those connectivities that we don't know
		for (set< pair<string, string> >::iterator it=catAppearance.begin();
				it!=catAppearance.end(); ++it)
		{
			SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->first);
			SpatialProbabilities::StringRandomVariableValuePtr appearancePropertyRVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->second);
			SpatialProbabilities::JointProbabilityValue jpv;
			for(std::list<DefaultAppearancePropertyGivenRoomCategory>::iterator it2 =
					_chainGraphInferencer->_defaultAppearancePropertyGivenRoomCategory.begin();
					it2!=_chainGraphInferencer->_defaultAppearancePropertyGivenRoomCategory.end(); ++it2)
			{
				if (it2->appearanceProperty == it->second)
				{
					jpv.probability=it2->probability;
					break;
				}
			}
			jpv.variableValues.push_back(roomCategory1RVVPtr);
			jpv.variableValues.push_back(appearancePropertyRVVPtr);
			factor.massFunction.push_back(jpv);
		}

		return factor;
	}


	// room_category1 -> human_asertion_property
	if ((variables.size()==2) &&
		(variables[0]=="room_category1") && (variables[1]=="humanassertion_property"))
	{
		// Set of all pairs category -> assertion
		set< pair<string, string> > catAssertion;
		for ( vector<string>::iterator it = _chainGraphInferencer->_roomCategories.begin();
			  it!=_chainGraphInferencer->_roomCategories.end(); ++it )
		{
			for ( vector<string>::iterator it2 = _chainGraphInferencer->_humanAssertions.begin();
				  it2!=_chainGraphInferencer->_humanAssertions.end(); ++it2 )
			{
				catAssertion.insert(pair<string,string>((*it), (*it2)));
			}
		}

		// Add the connectivities from the data file
		for(list<HumanAssertionPropertyGivenRoomCategory>::iterator it =
				_chainGraphInferencer->_humanAssertionPropertyGivenRoomCategory.begin();
				it!=_chainGraphInferencer->_humanAssertionPropertyGivenRoomCategory.end(); ++it)
		{
			catAssertion.erase(pair<string, string>(it->roomCategory1, it->humanAssertionProperty));

			SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->roomCategory1);
			SpatialProbabilities::StringRandomVariableValuePtr humanAssertionPropertyRVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->humanAssertionProperty);
			SpatialProbabilities::JointProbabilityValue jpv;
			jpv.probability=it->probability;
			jpv.variableValues.push_back(roomCategory1RVVPtr);
			jpv.variableValues.push_back(humanAssertionPropertyRVVPtr);
			factor.massFunction.push_back(jpv);
		}

		// Calculate the probability for the non-matching human assertion
		double defaultNonMatchingHumanAssertionProbability = (1.0-_chainGraphInferencer->_defaultMatchingHumanAssertionProbability) /
				static_cast<double>(_chainGraphInferencer->_humanAssertions.size()-1);

		// Add default potential for those connectivities that we don't know
		for (set< pair<string, string> >::iterator it=catAssertion.begin();
				it!=catAssertion.end(); ++it)
		{
			SpatialProbabilities::StringRandomVariableValuePtr roomCategory1RVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->first);
			SpatialProbabilities::StringRandomVariableValuePtr humanAssertionPropertyRVVPtr =
					new SpatialProbabilities::StringRandomVariableValue(it->second);
			SpatialProbabilities::JointProbabilityValue jpv;

			jpv.probability = (it->first.compare(it->second)==0)?_chainGraphInferencer->_defaultMatchingHumanAssertionProbability:defaultNonMatchingHumanAssertionProbability;
			jpv.variableValues.push_back(roomCategory1RVVPtr);
			jpv.variableValues.push_back(humanAssertionPropertyRVVPtr);
			factor.massFunction.push_back(jpv);
		}

		return factor;
	}


	// Factor not found!!
	throw CASTException("Factor '"+factorStr+"' not found!");
}


} // namespace def
