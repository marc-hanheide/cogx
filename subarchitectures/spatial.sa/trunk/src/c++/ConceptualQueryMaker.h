/*
 * ConceptualQueryMaker.h
 *
 *  Created on: Mar 6, 2011
 *      Author: alper
 */

#ifndef CONCEPTUALQUERYMAKER_H_
#define CONCEPTUALQUERYMAKER_H_

#include <cast/architecture/ManagedComponent.hpp>
#include "beliefs_cogx.hpp"
#include "VisionData.hpp"
#include "ConceptualData.hpp"
#include "VariableNameGenerator.h"

class ConceptualQueryMaker: public cast::ManagedComponent {

public:
  ConceptualQueryMaker();
  virtual ~ConceptualQueryMaker();
  void start();
  void ObjectQuery();
  void newVisualObject(const cast::cdl::WorkingMemoryChange &objID);
  void newPerceptBelief(const cast::cdl::WorkingMemoryChange &objID);
  void newGroundedBelief(const cast::cdl::WorkingMemoryChange &objID);
  void configure(const std::map<std::string, std::string>& _config);
  void runComponent();
  std::vector<std::string> m_objectsCategories;

  std::vector<std::string> m_roomIds;
  std::map<std::string, std::string> m_roomBeliefIds;

  std::vector<std::string> m_visualObjects;
  std::vector<std::string> m_visualObjectsBeliefIds;

  ConceptualData::QueryHandlerServerInterfacePrx
      m_queryHandlerServerInterfacePrx;
  std::string m_queryHandlerName;
  VariableNameGenerator m_namegenerator;

  std::string m_queryResult;

};

#endif /* CONCEPTUALQUERYMAKER_H_ */
