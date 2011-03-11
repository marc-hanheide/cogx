/**
 * @author Andrzej Pronobis
 */

#include "VariableNameGenerator.h"
#include <boost/lexical_cast.hpp>

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
