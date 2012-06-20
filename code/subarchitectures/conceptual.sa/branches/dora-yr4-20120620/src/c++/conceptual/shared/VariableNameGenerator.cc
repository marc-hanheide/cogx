/**
 * @author Andrzej Pronobis
 */

#include "VariableNameGenerator.h"
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/find.hpp>

using namespace std;
using namespace boost;

// -------------------------------------------------------
std::string VariableNameGenerator::getDefaultObjectPropertyVarName(std::string objectCategory,
		SpatialData::SpatialRelation relation, std::string supportObjectCategory)
{
	string objVarName;
	if (relation==SpatialData::INROOM)
		objVarName = "object_"+objectCategory+"_property";
	else
		objVarName = "object_"+objectCategory+
		            "_"+relationToString(relation)+"_"+supportObjectCategory+"_property";

	return objVarName;
}

// -------------------------------------------------------
std::string VariableNameGenerator::getDefaultObjectObservationVarName(std::string objectCategory,
		SpatialData::SpatialRelation relation, std::string supportObjectCategory)
{
	string objVarName;
	if (relation==SpatialData::INROOM)
		objVarName = "object_"+objectCategory+"_observation";
	else
		objVarName = "object_"+objectCategory+
		            "_"+relationToString(relation)+"_"+supportObjectCategory+"_observation";

	return objVarName;
}

// -------------------------------------------------------
std::string VariableNameGenerator::getUnexploredObjectVarName(int roomId, std::string objectCategory,
		SpatialData::SpatialRelation relation, std::string supportObjectCategory, std::string supportObjectId)
{
	string objVarName;
	if (relation==SpatialData::INROOM)
		objVarName = "room"+lexical_cast<string>(roomId)+"_object_"+objectCategory+"_unexplored";
	else
		objVarName = "room"+lexical_cast<string>(roomId)+"_object_"+objectCategory+
		            "_"+relationToString(relation)+"_"+supportObjectCategory+"-"+
		            supportObjectId+"_unexplored";

	return objVarName;
}


// -------------------------------------------------------
std::string VariableNameGenerator::getExploredObjectVarName(int roomId, std::string objectCategory,
		SpatialData::SpatialRelation relation, std::string supportObjectCategory, std::string supportObjectId)
{
	string objVarName;
	if (relation==SpatialData::INROOM)
		objVarName = "room"+lexical_cast<string>(roomId)+"_object_"+objectCategory+"_explored";
	else
		objVarName = "room"+lexical_cast<string>(roomId)+"_object_"+objectCategory+
		            "_"+relationToString(relation)+"_"+supportObjectCategory+"-"+
		            supportObjectId+"_explored";

	return objVarName;
}


// -------------------------------------------------------
std::string VariableNameGenerator::getObjectObservationVarName(int roomId, std::string objectCategory,
		SpatialData::SpatialRelation relation, std::string supportObjectCategory, std::string supportObjectId)
{
	string objVarName;
	if (relation==SpatialData::INROOM)
		objVarName = "room"+lexical_cast<string>(roomId)+"_object_"+objectCategory+"_observation";
	else
		objVarName = "room"+lexical_cast<string>(roomId)+"_object_"+objectCategory+
		            "_"+relationToString(relation)+"_"+supportObjectCategory+"-"+
		            supportObjectId+"_observation";

	return objVarName;
}

// -------------------------------------------------------
std::string VariableNameGenerator::getHumanAssertionPropertyObservationVarName(int roomId)
{
	return "room"+lexical_cast<string>(roomId)+"_humanassertion_property_observation";
}


// -------------------------------------------------------
std::string VariableNameGenerator::relationToString(SpatialData::SpatialRelation relation)
{
	switch(relation)
	{
	case SpatialData::ON:
		return "on";
	case SpatialData::INOBJECT:
		return "inobject";
	case SpatialData::INROOM:
		return "inroom";
	default:
		return "";
	}
}


// -------------------------------------------------------
SpatialData::SpatialRelation VariableNameGenerator::stringToRelation(std::string str)
{
	if (str == "on")
		return SpatialData::ON;
	else if (str == "inroom")
		return SpatialData::INROOM;
	else if (str == "inobject")
		return SpatialData::INOBJECT;
	else
		return SpatialData::INROOM;
}


// -------------------------------------------------------
bool VariableNameGenerator::parseDefaultObjectPropertyVar(std::string varName,
		std::string &objectCategory, SpatialData::SpatialRelation &relation,
		std::string &supportObjectCategory)
{
	// Header and footer
	trim(varName);
	if (!starts_with(varName, "object_"))
		return false;
	if (!ends_with(varName, "_property"))
		return false;

	replace_first(varName, "object_", "");
	replace_last(varName, "_property", "");

	// Detect relation
	string relationOnStr=relationToString(SpatialData::ON);
	string relationInRoomStr = relationToString(SpatialData::INROOM);
	string relationInObjectStr = relationToString(SpatialData::INOBJECT);
	string relationStr;
	if (contains(varName, "_"+ relationInObjectStr+"_"))
	{
		relation = SpatialData::INOBJECT;
		relationStr = relationInObjectStr;
	}
	else if (contains(varName, "_"+relationOnStr+"_"))
	{
		relation = SpatialData::ON;
		relationStr = relationOnStr;
	}
	else
	{
		relation = SpatialData::INROOM;
		objectCategory = varName;
		supportObjectCategory = "";
		return true;
	}

	// Split object with relation
	relationStr = "_"+relationStr+"_";
	int relPos=varName.find(relationStr);
	objectCategory = varName.substr(0, relPos);
	supportObjectCategory = varName.substr(relPos+relationStr.size(), varName.size()-relPos-relationStr.size());

	return true;
}

