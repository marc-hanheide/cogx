/*
 * ConceptualQueryMaker.cpp
 *
 *  Created on: Mar 6, 2011
 *      Author: alper
 */

#include "ConceptualQueryMaker.h"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <cast/core/CASTUtils.hpp>
#include <boost/lexical_cast.hpp>
#include "SpatialData.hpp"
#include "SpatialProbabilities.hpp"

using namespace std;
using namespace de::dfki::lt::tr::beliefs::slice;
using namespace de::dfki::lt::tr::beliefs::slice::sitbeliefs;
using namespace de::dfki::lt::tr::beliefs::slice::distribs;
using namespace de::dfki::lt::tr::beliefs::slice::logicalcontent;
using namespace eu::cogx::beliefs::slice;
using namespace cast::cdl;
using namespace cast;
using namespace boost;
using namespace SpatialProbabilities;

extern "C" {
cast::CASTComponentPtr newComponent() {
	return new ConceptualQueryMaker();
}
}

ConceptualQueryMaker::ConceptualQueryMaker() {
	// TODO Auto-generated constructor stub

}

ConceptualQueryMaker::~ConceptualQueryMaker() {
	// TODO Auto-generated destructor stub
}

void ConceptualQueryMaker::start() {
	log("I have started");
	addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject> (cdl::ADD),
			new MemberFunctionChangeReceiver<ConceptualQueryMaker> (this,
					&ConceptualQueryMaker::newVisualObject));

	addChangeFilter(cast::createGlobalTypeFilter<PerceptBelief>(cast::cdl::ADD),
			new cast::MemberFunctionChangeReceiver<ConceptualQueryMaker>(this,
					&ConceptualQueryMaker::newPerceptBelief));

	addChangeFilter(cast::createGlobalTypeFilter<GroundedBelief>(cast::cdl::ADD),
			new cast::MemberFunctionChangeReceiver<ConceptualQueryMaker>(this,
					&ConceptualQueryMaker::newGroundedBelief));

	m_visualObjects.push_back("table");
	m_queryHandlerServerInterfacePrx = getIceServer<
			ConceptualData::QueryHandlerServerInterface> (m_queryHandlerName);
}

void ConceptualQueryMaker::configure(
		const std::map<std::string, std::string>& _config) {

	map<string, string>::const_iterator it;

	// QueryHandler name
	if ((it = _config.find("--queryhandler")) != _config.end()) {
		m_queryHandlerName = it->second;
	}
}

void ConceptualQueryMaker::runComponent() {
	sleepComponent(10000);
	log("I am running");

	while (isRunning()) {
		ObjectQuery();
		sleepComponent(1000);
	}
}

void ConceptualQueryMaker::ObjectQuery() {
	//todo: for all object categories
	// loop through all the rooms
	// loop through all the on/in relations with existing objects
	log("querying over %d objects %d rooms", m_visualObjects.size(),
			m_roomIds.size());
	string querystring;
	string plus = "p(+";
	string closebracket = ")";
	for (unsigned int i = 0; i < m_visualObjects.size(); i++) {
		// look over rooms
		log("query for: %s", m_visualObjects[i].c_str());
		for (unsigned int j = 0; j < m_roomIds.size(); j++) {
			querystring = plus + m_namegenerator.getUnexploredObjectVarName(atoi(
					m_roomIds[j].c_str()), m_visualObjects[i], SpatialData::INROOM, "",
					"").c_str() + closebracket;
			log("querystring: %s", querystring.c_str());
			SpatialProbabilities::ProbabilityDistribution probDistrib =
					m_queryHandlerServerInterfacePrx->query(querystring);
			for (unsigned int k = 0; k < probDistrib.massFunction.size(); k++) {
				for (unsigned int l = 0; l
						< probDistrib.massFunction[k].variableValues.size(); l++) {
					log("here");
					StringRandomVariableValuePtr stringVariable =
							new StringRandomVariableValue;
					stringVariable = StringRandomVariableValuePtr::dynamicCast(
							probDistrib.massFunction[k].variableValues[l]);
					if (stringVariable->value == ConceptualData::EXISTS) {
						double prob = probDistrib.massFunction[k].probability;
						log("probability for %s is %f", querystring.c_str(), prob);
					}
				}
			}
		}
	}
}

void ConceptualQueryMaker::newVisualObject(
		const cast::cdl::WorkingMemoryChange &objID) {
	log("new visual object");
	try {
		VisionData::VisualObjectPtr visualobject(getMemoryEntry<
				VisionData::VisualObject> (objID.address));

		//FIXME: is the below true ?
		string objLabel = visualobject->identLabels[0];
		log("new visual object %s", objLabel.c_str());
		m_visualObjects.push_back(objLabel);

	} catch (const CASTException &e) {
		log("newVisualObject disappeared from WM: %s", e.message.c_str());

	}
}

void ConceptualQueryMaker::newPerceptBelief(
		const cast::cdl::WorkingMemoryChange &objID) {
	dBeliefPtr belief = new dBelief;
	belief = getMemoryEntry<dBelief> (objID.address);
	log("Got new percept belief: id: %s, type: %s", belief->id.c_str(),
			belief->type.c_str());
}

void ConceptualQueryMaker::newGroundedBelief(
		const cast::cdl::WorkingMemoryChange &objID) {
	dBeliefPtr belief = new dBelief;
	belief = getMemoryEntry<dBelief> (objID.address);
	if (belief->type == "ComaRoom") {

		// std::map<string, IntegerFormula> distrib;
		CondIndependentDistribsPtr condIndDistrib = new CondIndependentDistribs;
		BasicProbDistributionPtr basicDistrib = new BasicProbDistribution;
		FormulaValuesPtr formulaValues = new FormulaValues;
		IntegerFormulaPtr integerformula = new IntegerFormula;

		ProbDistributionPtr distptr(belief->content);

		condIndDistrib = CondIndependentDistribsPtr::dynamicCast(distptr);
		basicDistrib = BasicProbDistributionPtr::dynamicCast(
				condIndDistrib->distribs["RoomId"]);
		formulaValues = FormulaValuesPtr::dynamicCast(basicDistrib->values);
		integerformula = IntegerFormulaPtr::dynamicCast(
				formulaValues->values[0].val);

		log("This is a ComaRoom of id: %d", integerformula->val);

		string roomid = lexical_cast<string> (integerformula->val);
		m_roomIds.push_back(roomid);
		m_roomBeliefIds[roomid] = belief->id;
	}
	log("Got new grounded belief: id: %s, type: %s", belief->id.c_str(),
			belief->type.c_str());
}
