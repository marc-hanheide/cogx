/**
 * @author Andrzej Pronobis
 */

#ifndef CONCEPTUAL_VARIABLENAMEGENERATOR_H
#define CONCEPTUAL_VARIABLENAMEGENERATOR_H

#include "SpatialData.hpp"

#include <string>

class VariableNameGenerator
{
public:

	/** Generates name of the object property variable for default SA. */
	static std::string getDefaultObjectPropertyVarName(std::string objectCategory,
			SpatialData::SpatialRelation relation, std::string supportObjectCategory);

	/**
	 * Generates name of the variable for the event of the object being present in
	 * the unexplored part of space.
	 */
	static std::string getUnexploredObjectVarName(int roomId, std::string objectCategory,
			SpatialData::SpatialRelation relation, std::string supportObjectCategory,
			std::string supportObjectId);


	/** Converts relation type to string. */
	static std::string relationToString(SpatialData::SpatialRelation relation);

	/** Converts string to relation type. */
	static SpatialData::SpatialRelation stringToRelation(std::string str);
};


#endif
