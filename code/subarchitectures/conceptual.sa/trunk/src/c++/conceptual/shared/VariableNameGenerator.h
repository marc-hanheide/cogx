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

	/** Generates name of the object observation variable for default SA. */
	static std::string getDefaultObjectObservationVarName(std::string objectCategory,
			SpatialData::SpatialRelation relation, std::string supportObjectCategory);

	/** Parses name of the object property variable for default SA. */
	static bool parseDefaultObjectPropertyVar(std::string varName,
			std::string &objectCategory, SpatialData::SpatialRelation &relation,
			std::string &supportObjectCategory);

	/**
	 * Generates name of the variable for the event of the object being present in
	 * the unexplored part of space.
	 */
	static std::string getUnexploredObjectVarName(int roomId, std::string objectCategory,
			SpatialData::SpatialRelation relation, std::string supportObjectCategory,
			std::string supportObjectId);

	/**
	 * Generates name of the variable for the event of the number of objects being present in
	 * the explored part of space.
	 */
	static std::string getExploredObjectVarName(int roomId, std::string objectCategory,
			SpatialData::SpatialRelation relation, std::string supportObjectCategory,
			std::string supportObjectId);

	/**
	 * Generates name of the variable for the observation of the event of the number of objects being present in
	 * the explored part of space.
	 */
	static std::string getObjectObservationVarName(int roomId, std::string objectCategory,
			SpatialData::SpatialRelation relation, std::string supportObjectCategory,
			std::string supportObjectId);

	/**
	 * Generates name of the variable for the observation of the human assertion property.
	 */
	static std::string getHumanAssertionPropertyObservationVarName(int roomId);

	/** Converts relation type to string. */
	static std::string relationToString(SpatialData::SpatialRelation relation);

	/** Converts string to relation type. */
	static SpatialData::SpatialRelation stringToRelation(std::string str);
};


#endif
