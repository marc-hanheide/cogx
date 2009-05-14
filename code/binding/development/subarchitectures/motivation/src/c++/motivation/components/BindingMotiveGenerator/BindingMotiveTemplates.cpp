#include "BindingMotiveTemplates.hpp"
#include "BindingMotiveGenerator.hpp"

#include <motivation/components/BindingStateGenerator/BasicTranslators.hpp>

using namespace Binding;
using namespace BindingData;
using namespace BindingFeatures;
using namespace BindingQueries;
using namespace std;
using namespace cast;

void
AbstractMotiveTemplate::motiveProxies(const cast::cdl::WorkingMemoryChange & _wmc,
				      Binding::ProxySet & _proxies) {
  ProxySet allProxies(m_handler.allProxiesFromWM());

  //all relations
  ProxySet allRelations(allProxies | this->m_relationCheck);

  //relations that have not been used before
  ProxySet newRelations;
  for(ProxySet::const_iterator rel = allRelations.begin();
      rel != allRelations.end(); ++rel) {
    //if this relation has not been used before
    if(m_usedPreviously.find(rel->first)  == m_usedPreviously.end()) {
      newRelations[rel->first] = rel->second;
    }      
  }  

  //now get all the proxies these relations point to. this defines the
  //extra things we're allowed to keep
  ProxySet newlyRelatedToProxies(m_handler.extractProxiesFromProxies(newRelations, m_outPortExtractor));

  //for each existing proxy
  for(ProxySet::const_iterator p = allProxies.begin();
      p != allProxies.end(); ++p) {
    //if this proxy has not been used before
    if(m_usedPreviously.find(p->first)  == m_usedPreviously.end()) {      
      _proxies[p->first] = p->second;
    }
    //if it's been used before, but it's in a new relation
    else if(newlyRelatedToProxies.find(p->first)  == newlyRelatedToProxies.end()) {      
      _proxies[p->first] = p->second;      
    }
    else {
      //this should definitely be old
    }
  }
  

}

bool ActorTemplate::instantiate(const cast::cdl::WorkingMemoryChange & _wmc) {
    
  //string id(_wmc.m_address.m_id);
    
  ProxySet relatedSet;
  motiveProxies(_wmc,relatedSet);

  //BEGIN WARNING... NOT GOOD IN LONG TERM
  addAllToSourceProxies(relatedSet);
  //END WARNING... NOT GOOD IN LONG TERM



  ProxySet relationSet(relatedSet | this->m_relationCheck);

  ProxySet actorSet(relationSet | featureCheck<Binding::ProxyPtr,BindingFeatures::RelationLabel>(this->m_actorRegex));
  if(actorSet.size() != 1) {
    //cout<<"no actor"<<endl;
    return false;
  }

  //save actor rel
  this->m_actorRel = actorSet.begin()->second;

  //get the "to" of the actor
  ProxySet actorToSet(this->m_handler.extractProxiesFromProxies(actorSet, this->m_outPortToExtractor));
  if(actorToSet.size() != 1) {
    //cout<<"no actor to"<<endl;
    return false;
  }    
  this->m_actor = actorToSet.begin()->second;
  //cout<<"m_actor: "<<this->m_actor->id()<<endl;

  ProxySet actorFromSet(this->m_handler.extractProxiesFromProxies(actorSet, this->m_outPortFromExtractor));  
  if(actorFromSet.size() != 1) {
    //cout<<"no actor from"<<endl;
    return false;
  }
  this->m_action = actorFromSet.begin()->second;


  addToSourceProxies(m_actorRel);
  addToSourceProxies(m_actor);
  addToSourceProxies(m_action);

  //cout<<"ActorTemplate instantiated"<<endl;
  return true;


}


bool ActorPatientTemplate::instantiate(const cast::cdl::WorkingMemoryChange & _wmc) {
  bool parent(ActorTemplate::instantiate(_wmc)); 
    
  //wait until the top leve is instaniated before going further
  if(!parent) {
    //cout<<"parent failed"<<endl;
    return false;
  }
  
  ProxySet relationSet;
  motiveProxies(_wmc,relationSet);

  relationSet = relationSet | TypeCheck<ProxyPtr>(BindingData::RELATION);  

  //for target and landmark we need a Patient: concept
  ProxySet patientRelSet = relationSet | featureCheck<Binding::ProxyPtr,BindingFeatures::RelationLabel>(m_patientRegex); 
  
//   ////cout<<"action proxies via inports: "<<actionProxies.size()<<endl;

  if(patientRelSet.size() != 1) {
    //cout<<"missing or ambiguous Patient rel "<<endl;
    return false;
  }

  m_patientRel = patientRelSet.begin()->second;

  //patientSet should now contain a single relation proxyy that
  //specifies the "Patient" of the action
  ProxySet patientObjSet(this->m_handler.extractProxiesFromProxies(patientRelSet, this->m_outPortToExtractor));
  
  if(patientObjSet.size() != 1) {
    //cout<<"missing or ambiguous  patient specifier "<<endl;
    return false;
  }  

  m_patient = patientObjSet.begin()->second;
  ////cout<<"target: "<<m_target->id()<<endl;
  ////cout<<"rel: "<<m_relation->id()<<endl; 


  //cout<<"ActorPatientTemplate instantiated"<<endl;

  return true;
}




bool TargetLandmarkTemplate::instantiate(const cast::cdl::WorkingMemoryChange & _wmc) {
  bool parent(ActorPatientDestinationTemplate::instantiate(_wmc)); 
    
  //wait until the top leve is instaniated before going further
  if(!parent) {
    ////cout<<"parent failed"<<endl;
    return false;
  }

  // for the time being, action must be a put
  if(!m_actionChecker.test(this->m_action)) {
    ////cout<<"action type check failed"<<endl;
    return false;
  }

  
  //target is patient from superclass
  m_target = this->m_patient;
  addToSourceProxies(m_target);

  //target is patient from superclass
  m_landmark = this->m_destination;
  addToSourceProxies(m_landmark);

  //target is patient from superclass
  m_relation = this->m_destinationRel;


  return true;
}


std::string 
TargetLandmarkTemplate::goal(ProxyVariableTranslator & _translator,			   
			     TemporaryPlanningState & _state) {
  //assumes instantiated

  //the target position is now a concept
  const RelationLabel & rl(m_relation->getFeature<RelationLabel>());
  string relation(rl.m_label);

  //HACK trim destination stuff
  string dest("Destination:");
  unsigned int pos = relation.find(dest);
  if(pos != string::npos) {
    //the lazy way
    relation.replace(pos, dest.size(), ""); 
  }

  //HACK handle near being different to the others 
  if(relation != "near") {
    relation += "_of";
  }

  const string landmarkVar(_translator.translate(m_landmark));
  const string targetVar(_translator.translate(m_target));

  return "(and (" + relation + " " + targetVar + " " + landmarkVar + "))";
}


void 
AbstractMotiveTemplate::reset(const cast::cdl::TriBool & _succeeded) {
  //allowing instantiation again
  m_instantiated = false;
  

  //clean up source regardless
  for(WorkingMemoryPointerSet::const_iterator wmp = m_source.begin();
      wmp != m_source.end(); ++wmp) {
    //delete them motive proxies!    
//     try {
//       component().log("deleting motive source: %s",string(wmp->m_address.m_id).c_str());
//       component().deleteFromWorkingMemory(string(wmp->m_address.m_id));
//     }
//     catch (const DoesNotExistOnWMException & e) {
//       component().log(e.what());
//     }

    //nope, now store in "used" list
    m_usedPreviously.insert(string(wmp->m_address.m_id));
  }

  m_source.clear();

  //make motive proxies unavailable if required  
  for(ProxySet::const_iterator sp = m_sourceProxies.begin();
      sp != m_sourceProxies.end(); ++sp) {

    //also make source data proxy unavailable 
    if(sp->second->hasFeature<SourceData>()) {
      const SourceData & sd(sp->second->getFeature<SourceData>());      
      MakeProxyUnavailable * mpu = new MakeProxyUnavailable();
      mpu->m_proxyID = CORBA::string_dup(sd.m_address.m_id); 
      component().log("Requesting that proxy <%s> is made unavailable", string(mpu->m_proxyID).c_str());
      component().addToWorkingMemory(component().newDataID(), mpu);
      mpu = NULL;
    }
    
  }

  m_sourceProxies.clear();
}





using namespace BindingQueries;
using namespace boost;



bool 
FeatureRequestTemplate::instantiate(const cast::cdl::WorkingMemoryChange & _wmc) {
  component().log("FeatureRequestTemplate::instantiate");
  shared_ptr<const FeatureRequest> request(component().getWorkingMemoryEntry<FeatureRequest>(_wmc.m_address)->getData());	
  m_type = string(request->m_request.m_featurePointer.m_type);

  //HACK  we can only deal with certain types 
  if(m_type != typeName<BindingFeatures::Colour>()) {
    component().println("FeatureRequestTemplate cannot handle: " + m_type);
    return false;
  } 

  //reset to planning
  m_type = toPlanningFeature<BindingFeatures::Colour>();

    //the proxy (on binding wm) that the proxy is about
  m_proxyID = string(request->m_request.m_proxyID);
  //the id of the feature request on local wm
  m_reqID = string(_wmc.m_address.m_id);
  //the id of the feature involved, factual if "", polar if not
  m_featureID =  string(request->m_request.m_featurePointer.m_address);

  //store the creator sa
  m_creatorSA = string(request->m_fromSA);

  //the feature for planning
  //m_featureType = toPlanningFeature(string(request->m_request.m_featurePointer.m_type));
  
  //component().log("FeatureRequestTemplate instantiate: " + m_featureType);

  //where the query came from
  m_subarchID  =  string(request->m_fromSA);

  return true;

}


std::string 
FeatureRequestTemplate::goal(ProxyVariableTranslator & _translator,			   
			     TemporaryPlanningState & _state) {

  string objectVar(_translator.translate(m_proxyID));
  string subarchVar(_translator.translate(m_subarchID));

  
//   //TODO, go from feature to planning thingy
  string goal("(and (K " + subarchVar + " (" + m_type + " " + objectVar  + ")))"); 

  //string goal("(forall (?v2 - movable) (K " + subarchVar + " (in-answer-set ?v2)))");

  component().log("FeatureRequestTemplate goal: " + goal);
  
  //(not_knows_about__colour ?comp - subarchitecture ?obj - movable)
  Fact fact;
  fact.modality = NO_MODALITY;
  fact.agent = CORBA::string_dup("");
  fact.name = CORBA::string_dup(string("not_knows_about__" + m_type).c_str());
  fact.arguments.length(2);
  fact.arguments[0]  = CORBA::string_dup(subarchVar.c_str());
  fact.arguments[1]  = CORBA::string_dup(objectVar.c_str());
  fact.value = CORBA::string_dup("true");
  _state.m_factList.insert(fact);


  return goal;
  
}




bool 
PolarQuestionTemplate::instantiate(const cast::cdl::WorkingMemoryChange & _wmc) {
  component().log("PolarQuestionTemplate::instantiate");

  ProxySet allSet;
  motiveProxies(_wmc,allSet);

  //BEGIN WARNING... NOT GOOD IN LONG TERM
  addAllToSourceProxies(allSet);
  //END WARNING... NOT GOOD IN LONG TERM

  string id(_wmc.m_address.m_id);
  ProxySet initialSet(this->m_handler.loadProxies(id));

  //need a "Polar-Q"
  initialSet = initialSet | featureCheck<Binding::ProxyPtr,BindingFeatures::RelationLabel>(m_polarQRegex); 

  if(initialSet.size() != 1) {
    component().log("PolarQuestionTemplate::instantiate no Polar-Q");
    return false;
  }
  
  ProxyPtr polarQ = initialSet.begin()->second;

  
  ProxySet questionAboutSet(this->m_handler.extractProxiesFromProxies(initialSet, this->m_outPortFromExtractor));
  if(questionAboutSet.size() != 1) {
    component().log("PolarQuestionTemplate::instantiate no proxy to ask about");
    return false;
  }

  m_askAbout = questionAboutSet.begin()->second;
  addToSourceProxies(m_askAbout);

  ProxySet toAskAboutSet(this->m_handler.extractProxiesFromProxies(questionAboutSet, this->m_inPortsExtractor));
  toAskAboutSet.erase(polarQ->id());
  if(toAskAboutSet.size() != 1) {
    component().log("PolarQuestionTemplate::instantiate no proxy containing questions");
    return false;
  }



  return true;
}


std::string 
PolarQuestionTemplate::goal(ProxyVariableTranslator & _translator,			   
			     TemporaryPlanningState & _state) {


  return "not yet";
  
}














bool ActorDestinationTemplate::instantiate(const cast::cdl::WorkingMemoryChange & _wmc) {
  bool parent(ActorTemplate::instantiate(_wmc)); 
    
  //wait until the top leve is instaniated before going further
  if(!parent) {
    //cout<<"parent failed"<<endl;
    return false;
  }
  
  ProxySet relationSet;
  motiveProxies(_wmc,relationSet);

  relationSet = relationSet | TypeCheck<ProxyPtr>(BindingData::RELATION);  

  //for target and landmark we need a Destination: concept
  ProxySet destinationRelSet = relationSet | featureCheck<Binding::ProxyPtr,BindingFeatures::RelationLabel>(m_destinationRegex); 
  
//   ////cout<<"action proxies via inports: "<<actionProxies.size()<<endl;

  if(destinationRelSet.size() != 1) {
    //cout<<"missing or ambiguous Destination rel "<<endl;
    return false;
  }

  m_destinationRel = destinationRelSet.begin()->second;

  //destinationSet should now contain a single relation proxyy that
  //specifies the "Destination" of the action
  ProxySet destinationObjSet(this->m_handler.extractProxiesFromProxies(destinationRelSet, this->m_outPortToExtractor));
  
  if(destinationObjSet.size() != 1) {
    //cout<<"missing or ambiguous  destination specifier "<<endl;
    return false;
  }  

  m_destination = destinationObjSet.begin()->second;
  ////cout<<"target: "<<m_target->id()<<endl;
  ////cout<<"rel: "<<m_relation->id()<<endl; 


  //cout<<"ActorDestinationTemplate instantiated"<<endl;


  return true;
}


bool ActorPatientDestinationTemplate::instantiate(const cast::cdl::WorkingMemoryChange & _wmc) {
  bool parent(ActorPatientTemplate::instantiate(_wmc)); 
    
  //wait until the top leve is instaniated before going further
  if(!parent) {
    //cout<<"parent failed"<<endl;
    return false;
  }
  ProxySet relationSet;
  motiveProxies(_wmc,relationSet);

  relationSet = relationSet | TypeCheck<ProxyPtr>(BindingData::RELATION);  

  //for target and landmark we need a Destination: concept
  ProxySet destinationRelSet = relationSet | featureCheck<Binding::ProxyPtr,BindingFeatures::RelationLabel>(m_destinationRegex); 
  
//   ////cout<<"action proxies via inports: "<<actionProxies.size()<<endl;

  if(destinationRelSet.size() != 1) {
    //cout<<"missing or ambiguous Destination rel "<<endl;
    return false;
  }

  m_destinationRel = destinationRelSet.begin()->second;

  //destinationSet should now contain a single relation proxyy that
  //specifies the "Destination" of the action
  ProxySet destinationObjSet(this->m_handler.extractProxiesFromProxies(destinationRelSet, this->m_outPortToExtractor));
  
  if(destinationObjSet.size() != 1) {
    //cout<<"missing or ambiguous  destination specifier "<<endl;
    return false;
  }  

  m_destination = destinationObjSet.begin()->second;
  ////cout<<"target: "<<m_target->id()<<endl;
  ////cout<<"rel: "<<m_relation->id()<<endl; 

  //cout<<"ActorPatientDestinationTemplate instantiated"<<endl;

  return true;
}





bool GoToDestinationTemplate::instantiate(const cast::cdl::WorkingMemoryChange & _wmc) {
  bool parent(ActorDestinationTemplate::instantiate(_wmc)); 
  if(!parent) {
    //cout<<"parent failed"<<endl;
    return false;
  }

  // for the time being, action must be a go
  if(!m_actionChecker.test(this->m_action)) {
    //cout<<"action type check failed"<<endl;
    return false;
  }

  //store source for removal
  addToSourceProxies(m_destination);
  addToSourceProxies(m_destinationRel);
  addToSourceProxies(m_actorRel);
  addToSourceProxies(m_actor);
  addToSourceProxies(m_action);

  return true;

}

///get the name for the robot agent
std::string 
ActorTemplate::agentName(ProxyVariableTranslator & _translator) {
  return _translator.translate(m_actor);
}

///get the name for the robot agent
std::string 
GoToDestinationTemplate::agentName(ProxyVariableTranslator & _translator) {
  return _translator.translate(m_actor);
}


std::string 
GoToDestinationTemplate::goal(ProxyVariableTranslator & _translator,			   
			     TemporaryPlanningState & _state) {

  
  const string & robotID(m_actor->id());
  const string & robot(_translator.translate(m_actor));
  if(robotID == robot) {
    component().log("no robot variable yet");    
    return "";
  }

  const string & destinationID(m_destination->id());
  const string & destination(_translator.translate(m_destination));
  if(destinationID == destination) {
    component().log("no destination variable yet");    
    return "";
  }

  //dummy for now
  return "(and (perceived-position " + robot + " : " + destination + "))";
  //return "(and (perceived-position " + robot + " : " + "dummy" + "))";
  
}

bool FindTemplate::instantiate(const cast::cdl::WorkingMemoryChange & _wmc) {
  bool parent(ActorPatientTemplate::instantiate(_wmc)); 

  if(!parent) {
    //cout<<"parent failed"<<endl;
    return false;
  }

  // for the time being, action must be a go
  if(!m_actionChecker.test(this->m_action)) {
    //cout<<"action type check failed"<<endl;
    return false;
  }

  //store source for removal
  addToSourceProxies(m_patient);
  addToSourceProxies(m_patientRel);
  addToSourceProxies(m_actorRel);
  addToSourceProxies(m_actor);
  addToSourceProxies(m_action);

  return true;

}

std::string 
FindTemplate::goal(ProxyVariableTranslator & _translator,			   
		   TemporaryPlanningState & _state) {

  const string & robotID(m_actor->id());
  const string & robot(_translator.translate(m_actor));
  if(robotID == robot) {
    component().log("no robot variable yet");    
    return "";
  }

  const string & objectID(m_patient->id());
  const string & object(_translator.translate(m_patient));
  if(objectID == object) {
    component().log("no object variable yet");    
    return "";
  }

  //for a find the robot must know the perceived position of the
  //object
  return "(and (K " + robot + " (perceived-position " + object + ")))";
  
}


bool PlayTheGameTemplate::instantiate(const cast::cdl::WorkingMemoryChange & _wmc) {
  bool parent(ActorPatientTemplate::instantiate(_wmc)); 

  if(!parent) {
    //cout<<"parent failed"<<endl;
    return false;
  }

  // for the time being, action must be a go
  if(!m_actionChecker.test(this->m_action)) {
    //cout<<"action type check failed"<<endl;
    return false;
  }

  //check for the patient concept
  if(!m_patient->hasFeature<BindingFeatures::Concept>()) {
    return false;
  }

  const BindingFeatures::Concept & concept(m_patient->getFeature<BindingFeatures::Concept>());

  //different game types
  if(strcmp(concept.m_concept, "color_game") == 0) {
    ostringstream goalStream;
    goalStream<<"(and ";
    goalStream<<"(forall (?obj - movable)  " ;
    goalStream<<"(forall (?col - colour)  " ;
    goalStream<<      "(imply (colour ?obj : ?col)	 " ;
    goalStream<<      "(forall (?flg - flag)  " ;
    goalStream<<      "(imply (colour ?flg : ?col)	 " ;
    goalStream<<"(near ?obj ?flg) " ;
    goalStream<<      ") " ;
    goalStream<<") " ;
    goalStream<<      ") " ;
    goalStream<<") " ;
    goalStream<<      ") " ;
    goalStream<<") ";
    m_goal = goalStream.str();
  }
  else if(strcmp(concept.m_concept, "shape_game") == 0) {
    ostringstream goalStream;
    m_goal =  "	(and";
    goalStream<<"(exists (?flg - flag) " ;
    goalStream<<"(and";
    goalStream<<      "(on_table_right ?flg) " ;
    goalStream<<"(forall (?obj - movable) " ;
    goalStream<<      "(imply (shape ?obj : square)	" ;
    goalStream<<"(near ?obj ?flg) " ;
    goalStream<<")" ;
    goalStream<<")" ;
    goalStream<<")" ;
    goalStream<<")" ;
    goalStream<<"(exists (?flg - flag) " ;
    goalStream<<"(and" ;
    goalStream<<"(on_table_left ?flg) " ;
    goalStream<<"(forall (?obj - movable) " ; 
    goalStream<<"(imply (shape ?obj : triangle) " ;	
    goalStream<<"(near ?obj ?flg) " ;
    goalStream<<")" ;
    goalStream<<")" ;
    goalStream<<")" ;
    goalStream<<")" ;
    goalStream<<"(exists (?flg - flag) " ;
    goalStream<<"(and" ;
    goalStream<<"(on_table_left ?flg) " ;
    goalStream<<"(forall (?obj - movable) " ; 
    goalStream<<"(imply (shape ?obj : circle) " ;
    goalStream<<"(near ?obj ?flg) " ;
    goalStream<<")" ;
    goalStream<<")" ;
    goalStream<<")" ;
    goalStream<<")" ;
    goalStream<<")";
    m_goal = goalStream.str();
  }
  else if(strcmp(concept.m_concept, "game") == 0) {
    m_goal = "";
  }
  else {
    return false;
  }


  //store source for removal
  addToSourceProxies(m_patient);
  addToSourceProxies(m_patientRel);
  addToSourceProxies(m_actorRel);
  addToSourceProxies(m_actor);
  addToSourceProxies(m_action);

  return true;

}

std::string 
PlayTheGameTemplate::goal(ProxyVariableTranslator & _translator,			   
		   TemporaryPlanningState & _state) {

  return m_goal;
  
}


bool 
FactualQuestionTemplate::instantiate(const cast::cdl::WorkingMemoryChange & _wmc) {
  component().log("FactualQuestionTemplate::instantiate");


  //BEGIN WARNING... NOT GOOD IN LONG TERM
  ProxySet relationSet;
  motiveProxies(_wmc,relationSet);
  addAllToSourceProxies(relationSet);
  //END WARNING... NOT GOOD IN LONG TERM


  string id(_wmc.m_address.m_id);
  ProxySet initialSet(this->m_handler.loadProxies(id));

  //need a "Fact-Q"
  initialSet = initialSet | featureCheck<Binding::ProxyPtr,BindingFeatures::RelationLabel>(m_factualQRegex); 

  if(initialSet.size() != 1) {
    component().log("FactualQuestionTemplate::instantiate no Fact-Q");
    return false;
  }
  
  m_factQRel = initialSet.begin()->second;
  addToSourceProxies(m_factQRel);

  ProxySet questionedObjectSet(this->m_handler.extractProxiesFromProxies(initialSet, this->m_outPortToExtractor));
  questionedObjectSet.erase(m_factQRel->id());
  if(questionedObjectSet.size() != 1) {
    component().log("FactualQuestionTemplate::instantiate no proxy for the object being asked about");
    return false;
  }

  m_questionedObject = questionedObjectSet.begin()->second;  
  addToSourceProxies(m_questionedObject);

  ProxySet questionedPropertySet(this->m_handler.extractProxiesFromProxies(initialSet, this->m_outPortFromExtractor));
  questionedPropertySet.erase(m_factQRel->id());
  if(questionedPropertySet.size() != 1) {
    component().log("FactualQuestionTemplate::instantiate no proxy for what to ask about");
    return false;
  }

  m_questionedProperty = questionedPropertySet.begin()->second;
  addToSourceProxies(m_questionedProperty);

  return true;
}

bool 
FactualWHQuestionTemplate::instantiate(const cast::cdl::WorkingMemoryChange & _wmc) {
  component().log("FactualWHQuestionTemplate::instantiate");
  bool parent(FactualQuestionTemplate::instantiate(_wmc));   
  if(!parent) {
    return false;
  }
  
  //just process directly because the results are reused
  //check for the questionedProperty concept
  if(!m_questionedProperty->hasFeature<BindingFeatures::Concept>()) {
    component().log("FactualWHQuestionTemplate::instantiate no concept for questionedProperty");
    return false;
  }

//just process directly because the results are reused
  //check for the questionedProperty concept
  if(!m_questionedProperty->hasFeature<BindingFeatures::SourceID>()) {
    component().log("FactualWHQuestionTemplate::instantiate no source id for questionedProperty");
    return false;
  }

  const BindingFeatures::Concept & concept(m_questionedProperty->getFeature<BindingFeatures::Concept>());
  m_conceptString = string(concept.m_concept);

  string prefix("WH:?");
  unsigned int pos = m_conceptString.find(prefix);
  if(pos != string::npos) {
    m_conceptString.replace(pos, prefix.size(), ""); 
  }
  else {
    component().log("FactualWHQuestionTemplate::instantiate no wh:? prefix in: " + m_conceptString);
    return false;
  }
  
  //mega HACK for US spelling!
  if(m_conceptString == "color") {
    m_conceptString = "colour";
  }
  
  const BindingFeatures::SourceID & sourceID(m_questionedProperty->getFeature<BindingFeatures::SourceID>());
  m_askingSA = string(sourceID.m_sourceID);

  return true;

}


std::string 
FactualWHQuestionTemplate::goal(ProxyVariableTranslator & _translator,
				TemporaryPlanningState & _state) {
  
  
//   if(!variableExistsFor(m_questionedObject,_translator)) {
//     component().log("FactualWHQuestionTemplate::instantiate no variable for object: " + m_questionedObject->id());
//     return "";
//   }
//   if(!variableExistsFor(m_askingSA, _translator)) {
//     component().log("FactualWHQuestionTemplate::instantiate no variable for sa: " + m_askingSA);
//     return "";
//   }

  string objectVar = _translator.translate(m_questionedObject);
  string saVar = _translator.translate(m_askingSA);

#warning No addressee for the questions, adding self to additional state
  //we also need to define the robot!
  ObjectDeclaration self;
  self.name = CORBA::string_dup(agentName(_translator).c_str());
  self.type = CORBA::string_dup("robot");
  _state.m_objectList.insert(self);    

    //(not_knows_about__colour ?comp - subarchitecture ?obj - movable)
  Fact fact;
  fact.modality = NO_MODALITY;
  fact.agent = CORBA::string_dup("");
  fact.name = CORBA::string_dup(string("not_knows_about__" + m_conceptString).c_str());
  fact.arguments.length(2);
  fact.arguments[0]  = CORBA::string_dup(saVar.c_str());
  fact.arguments[1]  = CORBA::string_dup(objectVar.c_str());
  fact.value = CORBA::string_dup("true");
  _state.m_factList.insert(fact);



  return "(and (K " + saVar + " (" + m_conceptString  + " " + objectVar + ")))";
  
}



/////////////////begin what do you see

bool 
FactualPerceiveQuestionTemplate::instantiate(const cast::cdl::WorkingMemoryChange & _wmc) {

  component().log("FactualPerceiveQuestionTemplate::instantiate");
  bool parent(FactualQuestionTemplate::instantiate(_wmc));   
  if(!parent) {
    return false;
  }
  
  //just process directly because the results are reused
  //check for the questionedProperty concept
  if(!m_questionedProperty->hasFeature<BindingFeatures::Concept>()) {
    component().log("FactualPerceiveQuestionTemplate::instantiate no concept for questionedProperty");
    return false;
  }

  //just process directly because the results are reused
  //check for the questionedProperty concept
  if(!m_questionedProperty->hasFeature<BindingFeatures::SourceID>()) {
    component().log("FactualPerceiveQuestionTemplate::instantiate no source id for questionedProperty");
    return false;
  }

  const BindingFeatures::Concept & concept(m_questionedProperty->getFeature<BindingFeatures::Concept>());
  string con(concept.m_concept);
  if(con != "see") {
    component().log("FactualPerceiveQuestionTemplate::instantiate no \"see\" concept");
    return false;
  }  
  
  const BindingFeatures::SourceID & sourceID(m_questionedProperty->getFeature<BindingFeatures::SourceID>());
  m_askingSA = string(sourceID.m_sourceID);

  return true;

}


std::string 
FactualPerceiveQuestionTemplate::goal(ProxyVariableTranslator & _translator,
				      TemporaryPlanningState & _state)  {
  //"see" is what movables exist 
  string saVar = _translator.translate(m_askingSA);
  return "(and (forall (?m - movable) (K " + saVar + " (in-answer-set ?m))))";  
  
}

